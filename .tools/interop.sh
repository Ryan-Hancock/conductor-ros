#!/usr/bin/env bash
# Interop matrix: conductor <-> real ROS 2 over rmw_zenoh.
#
# Every leg matters. Testing only Go<->rclpy once hid a bug for a whole
# milestone: conductor's zenoh querier used the default consolidation mode,
# which swallowed replies whenever BOTH peers were conductor processes.
#
#   ./.tools/interop.sh            run every leg
#   ./.tools/interop.sh services   run one group (services | actions)
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

echo "building..."
go build -tags zenoh -o "$BIN/patrol" ./examples/patrol
go build -tags zenoh -o "$BIN/fibonacci" ./examples/fibonacci
go build -tags zenoh -o "$BIN/mission" ./examples/mission

echo "starting zenoh router..."
bg "$WORK/router.log" "$CONDUCTOR_OVERLAY/lib/rmw_zenoh_cpp/rmw_zenohd"
sleep 2

if [[ "$GROUP" == all || "$GROUP" == services ]]; then
  echo "services:"
  bg "$WORK/patrol.log" "$BIN/patrol" -transport zenoh
  sleep 3

  timeout 20 ros2 service call /engage_estop std_srvs/srv/SetBool "{data: true}" \
    >"$WORK/svc_ros_to_go.log" 2>&1
  check "ros2 service call -> conductor server" "$WORK/svc_ros_to_go.log" "success=True"

  timeout 20 ros2 topic echo --once /cmd_vel >"$WORK/topic.log" 2>&1
  check "conductor publisher -> ros2 topic echo" "$WORK/topic.log" "linear:"
fi

if [[ "$GROUP" == all || "$GROUP" == actions ]]; then
  echo "actions:"
  bg "$WORK/fib.log" "$BIN/fibonacci" -transport zenoh
  fib_pid="$LAST_PID"
  sleep 3

  # rclpy client -> conductor action server
  timeout 40 ros2 action send_goal --feedback /fibonacci \
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
