#!/usr/bin/env bash
# Interop matrix: conductor <-> real ROS 2 over rmw_zenoh.
#
# Every leg matters. Testing only Go<->rclpy once hid a bug for a whole
# milestone: conductor's zenoh querier used the default consolidation mode,
# which swallowed replies whenever BOTH peers were conductor processes.
#
#   ./.tools/interop.sh            run every leg
#   ./.tools/interop.sh services   run one group (services | actions | lifecycle)
#
# Requires a ROS 2 install plus the user-space rmw_zenoh overlay in .tools
# (see env.sh). Starts and stops its own zenoh router.
set -o pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
source .tools/env.sh # ROS setup.bash reads unset vars, so no `set -u` here
GROUP="${1:-all}"
WORK="$(mktemp -d)"
BIN="$WORK/bin"
mkdir -p "$BIN"
PIDS=()
PASS=0
FAIL=0

cleanup() {
  for pid in "${PIDS[@]:-}"; do kill -9 "$pid" 2>/dev/null; done
  rm -rf "$WORK"
}
trap cleanup EXIT

LAST_PID=""
bg() { # bg <logfile> <cmd...>; sets LAST_PID
  local log="$1"; shift
  setsid nohup "$@" >"$log" 2>&1 </dev/null &
  LAST_PID=$!
  PIDS+=("$LAST_PID")
}

check() { # check <name> <logfile> <expected substring>
  if grep -qF "$3" "$2"; then
    echo "  PASS  $1"
    PASS=$((PASS + 1))
  else
    echo "  FAIL  $1 (wanted: $3)"
    sed 's/^/        /' "$2" | tail -12
    FAIL=$((FAIL + 1))
  fi
}

# Leftover processes from an earlier run are poison: a second /navigator on
# the graph answers the same service names, so queries hit either one at
# random and results look nondeterministic. Start from a clean graph.
echo "clearing any leftover processes..."
pkill -9 -x patrol 2>/dev/null
pkill -9 -x fibonacci 2>/dev/null
pkill -9 -x mission 2>/dev/null
pkill -9 -f pyfib_server.py 2>/dev/null
pkill -9 -x rmw_zenohd 2>/dev/null
sleep 1

echo "building..."
go build -tags zenoh -o "$BIN/patrol" ./examples/patrol
go build -tags zenoh -o "$BIN/fibonacci" ./examples/fibonacci
go build -tags zenoh -o "$BIN/mission" ./examples/mission

echo "starting zenoh router..."
bg "$WORK/router.log" "$CONDUCTOR_OVERLAY/lib/rmw_zenoh_cpp/rmw_zenohd"
sleep 2

# The ros2 CLI daemon caches the graph and keeps connections to whichever
# router it first saw. Left over from an earlier run it makes graph queries
# (ros2 lifecycle/action/node) hang past their timeout, so restart it against
# the router we just started.
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1

# ros2 CLI calls can wedge in a way that ignores SIGTERM, so every invocation
# below goes through ros2run, which escalates to SIGKILL.
ros2run() { # ros2run <seconds> <args...>
  local secs="$1"; shift
  timeout --kill-after=5 "$secs" ros2 "$@"
}

if [[ "$GROUP" == all || "$GROUP" == lifecycle ]]; then
  echo "lifecycle:"
  bg "$WORK/lifecycle.log" "$BIN/patrol" -transport zenoh -trace
  lifecycle_pid="$LAST_PID"
  sleep 3

  ros2run 20 lifecycle nodes >"$WORK/lc_nodes.log" 2>&1
  check "ros2 lifecycle nodes lists conductor nodes" "$WORK/lc_nodes.log" "/navigator"

  ros2run 20 lifecycle get /navigator >"$WORK/lc_get.log" 2>&1
  check "conductor node reports active" "$WORK/lc_get.log" "active [3]"

  ros2run 20 lifecycle set /navigator deactivate >"$WORK/lc_set.log" 2>&1
  check "ros2 lifecycle set deactivate" "$WORK/lc_set.log" "Transitioning successful"

  # Deactivated publishers must go quiet: echo should time out with no data.
  ros2run 6 topic echo --once /cmd_vel >"$WORK/lc_quiet.log" 2>&1
  if grep -q "linear:" "$WORK/lc_quiet.log"; then
    echo "  FAIL  deactivated node stops publishing"
    FAIL=$((FAIL + 1))
  else
    echo "  PASS  deactivated node stops publishing"
    PASS=$((PASS + 1))
  fi

  ros2run 20 lifecycle set /navigator activate >"$WORK/lc_reactivate.log" 2>&1
  sleep 1
  ros2run 10 topic echo --once /cmd_vel >"$WORK/lc_resume.log" 2>&1
  check "reactivated node publishes again" "$WORK/lc_resume.log" "linear:"

  # Tracing: one causal chain across three nodes shares a trace id.
  trace_id=$(grep -m1 'kind=subscription name=cmd_vel' "$WORK/lifecycle.log" | sed 's/.*trace=\([0-9a-f]*\).*/\1/')
  if [[ -n "$trace_id" ]] && grep -q "trace=$trace_id.*kind=timer" "$WORK/lifecycle.log"; then
    echo "  PASS  trace context propagates across nodes"
    PASS=$((PASS + 1))
  else
    echo "  FAIL  trace context propagates across nodes"
    FAIL=$((FAIL + 1))
  fi

  kill -9 "$lifecycle_pid" 2>/dev/null
  sleep 1
fi

if [[ "$GROUP" == all || "$GROUP" == services ]]; then
  echo "services:"
  bg "$WORK/patrol.log" "$BIN/patrol" -transport zenoh
  sleep 3

  ros2run 20 service call /engage_estop std_srvs/srv/SetBool "{data: true}" \
    >"$WORK/svc_ros_to_go.log" 2>&1
  check "ros2 service call -> conductor server" "$WORK/svc_ros_to_go.log" "success=True"

  ros2run 20 topic echo --once /cmd_vel >"$WORK/topic.log" 2>&1
  check "conductor publisher -> ros2 topic echo" "$WORK/topic.log" "linear:"
fi

if [[ "$GROUP" == all || "$GROUP" == actions ]]; then
  echo "actions:"
  bg "$WORK/fib.log" "$BIN/fibonacci" -transport zenoh
  fib_pid="$LAST_PID"
  sleep 3

  # rclpy client -> conductor action server
  ros2run 40 action send_goal --feedback /fibonacci \
    example_interfaces/action/Fibonacci "{order: 6}" >"$WORK/act_ros_to_go.log" 2>&1
  check "ros2 action send_goal -> conductor server" "$WORK/act_ros_to_go.log" "SUCCEEDED"

  # conductor client -> conductor action server (the leg that caught the
  # consolidation bug: both peers are conductor processes)
  timeout 40 "$BIN/mission" -transport zenoh >"$WORK/act_go_to_go.log" 2>&1
  check "conductor client -> conductor server" "$WORK/act_go_to_go.log" "status=SUCCEEDED"

  # conductor client -> rclpy action server
  kill -9 "$fib_pid" 2>/dev/null # stop the Go server so python serves the name
  sleep 1
  bg "$WORK/pyfib.log" python3 .tools/pyfib_server.py
  sleep 4
  timeout 40 "$BIN/mission" -transport zenoh >"$WORK/act_go_to_py.log" 2>&1
  check "conductor client -> rclpy server" "$WORK/act_go_to_py.log" "status=SUCCEEDED"
fi

echo
echo "$PASS passed, $FAIL failed"
[[ "$FAIL" -eq 0 ]]
