#!/usr/bin/env bash
# Interop matrix: conductor <-> real ROS 2 over rmw_zenoh.
#
# Every leg matters. Testing only Go<->rclpy once hid a bug for a whole
# milestone: conductor's zenoh querier used the default consolidation mode,
# which swallowed replies whenever BOTH peers were conductor processes.
#
#   ./.tools/interop.sh            run every leg
#   ./.tools/interop.sh services   one group
#                                  (services | actions | lifecycle | params |
#                                   frames | nav2 | moveit | turtlesim)
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
pkill -9 -x turtlesim 2>/dev/null
pkill -9 -x turtlesim_node 2>/dev/null
pkill -9 -x tf2_echo 2>/dev/null
pkill -9 -x nav2 2>/dev/null
pkill -9 -x nav2stub 2>/dev/null
pkill -9 -x moveit 2>/dev/null
pkill -9 -x moveitstub 2>/dev/null
pkill -9 -x rmw_zenohd 2>/dev/null
sleep 1

echo "building..."
go build -tags zenoh -o "$BIN/patrol" ./examples/patrol
go build -tags zenoh -o "$BIN/fibonacci" ./examples/fibonacci
go build -tags zenoh -o "$BIN/mission" ./examples/mission
go build -tags zenoh -o "$BIN/turtlesim" ./examples/turtlesim
go build -tags zenoh -o "$BIN/nav2" ./examples/nav2
go build -tags zenoh -o "$BIN/nav2stub" ./examples/nav2stub
go build -tags zenoh -o "$BIN/conductor" ./cmd/conductor
go build -tags zenoh -o "$BIN/moveit" ./examples/moveit
go build -tags zenoh -o "$BIN/moveitstub" ./examples/moveitstub

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
  bg "$WORK/lifecycle.log" "$BIN/patrol" -transport zenoh -trace -frames examples/patrol/frames.json
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

if [[ "$GROUP" == all || "$GROUP" == params ]]; then
  echo "parameters:"
  # A base file plus a sim overlay: the overlay must win.
  cat >"$WORK/params.yaml" <<'YAML'
navigator:
  ros__parameters:
    max_speed: 1.5
YAML
  cat >"$WORK/params.sim.yaml" <<'YAML'
navigator:
  ros__parameters:
    max_speed: 0.75
YAML
  bg "$WORK/params_node.log" "$BIN/patrol" -transport zenoh -params "$WORK/params.yaml" -env sim -frames examples/patrol/frames.json
  params_pid="$LAST_PID"
  sleep 3

  ros2run 25 param list /navigator >"$WORK/p_list.log" 2>&1
  check "ros2 param list -> conductor node" "$WORK/p_list.log" "max_speed"

  ros2run 25 param get /navigator max_speed >"$WORK/p_get.log" 2>&1
  check "environment overlay wins over base file" "$WORK/p_get.log" "0.75"

  ros2run 25 param set /navigator max_speed 0.25 >"$WORK/p_set.log" 2>&1
  check "ros2 param set -> conductor node" "$WORK/p_set.log" "Set parameter successful"

  sleep 1
  ros2run 25 param get /navigator max_speed >"$WORK/p_get2.log" 2>&1
  check "value read back after set" "$WORK/p_get2.log" "0.25"

  # The new limit must actually change behaviour, not just the stored value.
  ros2run 10 topic echo --once /cmd_vel >"$WORK/p_clamp.log" 2>&1
  if python3 - "$WORK/p_clamp.log" <<'PY'
import re, sys
t = open(sys.argv[1]).read()
xs = re.findall(r'x: (-?[0-9.e-]+)', t)
ys = re.findall(r'y: (-?[0-9.e-]+)', t)
sys.exit(0 if xs and ys and (float(xs[0])**2 + float(ys[0])**2) ** 0.5 <= 0.2501 else 1)
PY
  then
    echo "  PASS  new max_speed clamps published velocity"
    PASS=$((PASS + 1))
  else
    echo "  FAIL  new max_speed clamps published velocity"
    FAIL=$((FAIL + 1))
  fi

  ros2run 25 param describe /navigator max_speed >"$WORK/p_desc.log" 2>&1
  check "ros2 param describe reports the type" "$WORK/p_desc.log" "Type: double"

  kill -9 "$params_pid" 2>/dev/null
  sleep 1
fi

if [[ "$GROUP" == all || "$GROUP" == services ]]; then
  echo "services:"
  bg "$WORK/patrol.log" "$BIN/patrol" -transport zenoh -frames examples/patrol/frames.json
  services_pid="$LAST_PID"
  sleep 3

  ros2run 20 service call /engage_estop std_srvs/srv/SetBool "{data: true}" \
    >"$WORK/svc_ros_to_go.log" 2>&1
  check "ros2 service call -> conductor server" "$WORK/svc_ros_to_go.log" "success=True"

  ros2run 20 topic echo --once /cmd_vel >"$WORK/topic.log" 2>&1
  check "conductor publisher -> ros2 topic echo" "$WORK/topic.log" "linear:"

  # Every group has to leave the graph as it found it. This one used not to,
  # and a patrol left publishing /cmd_vel as a plain Twist put two types on one
  # topic for every later group — which is the poison the header warns about,
  # and which the nav2 discovery leg is the first thing sensitive enough to
  # notice.
  { kill -9 "$services_pid"; wait "$services_pid"; } 2>/dev/null
  sleep 1
fi

if [[ "$GROUP" == all || "$GROUP" == frames ]]; then
  echo "frames:"
  bg "$WORK/frames_node.log" "$BIN/patrol" -transport zenoh -frames examples/patrol/frames.json \
    -description examples/patrol/patrol.urdf
  frames_pid="$LAST_PID"
  sleep 3

  # The declared tree has to be readable as tf2, not merely published: the
  # whole point is that rviz, tf2_ros and everything else consume it.
  ros2run 20 topic echo --once /tf_static >"$WORK/tf_static.log" 2>&1
  check "declared transforms reach /tf_static" "$WORK/tf_static.log" "child_frame_id: laser"

  ros2run 20 topic info /tf_static >"$WORK/tf_type.log" 2>&1
  check "tf_static carries tf2_msgs/msg/TFMessage" "$WORK/tf_type.log" "Type: tf2_msgs/msg/TFMessage"

  # tf2 itself composing our transform: translation and yaw as declared.
  bg "$WORK/tf_echo.log" ros2 run tf2_ros tf2_echo base_link laser
  tf_echo_pid="$LAST_PID"
  sleep 6
  { kill -9 "$tf_echo_pid"; wait "$tf_echo_pid"; } 2>/dev/null
  # `ros2 run` forks the real binary, so killing the wrapper leaves the node on
  # the graph — nine of them accumulated before discovery grew sharp enough to
  # notice.
  pkill -9 -x tf2_echo 2>/dev/null
  check "tf2_echo resolves base_link -> laser" "$WORK/tf_echo.log" "Translation: [0.120, 0.000, 0.190]"
  check "the declared yaw survives the round trip" "$WORK/tf_echo.log" "in RPY (degree) [0.000, -0.000, 180.000]"

  # A frame tag is what stamps the header on the wire.
  ros2run 20 topic echo --once /amcl_pose >"$WORK/tf_frame.log" 2>&1
  check "frame tag stamps the published header" "$WORK/tf_frame.log" "frame_id: map"

  # Durability: /tf_static is published exactly once, when the node goes active,
  # and latched. This waits well past any republish interval before asking, so a
  # reply can only come from the publisher's cache — which is what ROS means by
  # transient-local, and what conductor used to fake with a 1 Hz repeat.
  sleep 8
  ros2run 20 topic echo --once --qos-durability transient_local --qos-reliability reliable \
    /tf_static >"$WORK/tf_latched.log" 2>&1
  check "a late subscriber still gets the latched tree" "$WORK/tf_latched.log" "child_frame_id: laser"

  # The robot's model belongs on /robot_description, latched, so a tool started
  # at any point can draw the robot. This application owns the transform tree —
  # patrol has no robot_state_publisher — so it publishes the description the
  # tree was derived from.
  # --full-length because echo truncates a long string, and the interesting
  # part of a URDF is not in its first hundred characters.
  ros2run 20 topic echo --once --full-length --qos-durability transient_local \
    --qos-reliability reliable /robot_description >"$WORK/description.log" 2>&1
  check "the robot description is published and latched" "$WORK/description.log" "<robot name="
  ros2run 20 topic info -v /robot_description >"$WORK/description_info.log" 2>&1
  check "it is a std_msgs/String, transient-local" "$WORK/description_info.log" "Durability: TRANSIENT_LOCAL"

  { kill -9 "$frames_pid"; wait "$frames_pid"; } 2>/dev/null
  sleep 1

  # The other direction: a real ROS latched publisher, and a conductor
  # subscriber that starts afterwards. Its estop subscription is transient, so a
  # value published before it existed has to reach it — and that message arrives
  # while the node is still unconfigured, which is the case that makes latching
  # more than a transport flag.
  bg "$WORK/estop_pub.log" ros2 topic pub -r 0.05 \
    --qos-durability transient_local --qos-reliability reliable \
    /estop std_msgs/msg/Bool "{data: true}"
  estop_pub_pid="$LAST_PID"
  sleep 6

  timeout 12 "$BIN/patrol" -transport zenoh -frames examples/patrol/frames.json \
    >"$WORK/estop_late.log" 2>&1
  check "conductor receives a latched message published before it started" \
    "$WORK/estop_late.log" "estop state changed engaged=true"

  { kill -9 "$estop_pub_pid"; wait "$estop_pub_pid"; } 2>/dev/null
  pkill -9 -f "topic pub.*estop" 2>/dev/null
  sleep 1
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

if [[ "$GROUP" == all || "$GROUP" == nav2 ]]; then
  echo "nav2:"
  # Discovery reads the whole graph, so this group is the one that notices any
  # process an earlier group left behind. Start from a clean one.
  pkill -9 -x patrol 2>/dev/null
  pkill -9 -x fibonacci 2>/dev/null
  pkill -9 -x mission 2>/dev/null
  sleep 1
  # Nav2's interfaces are vendored .action/.srv text hashed locally, and
  # nav2_msgs is NOT installed here. So these checks answer the question that
  # matters: does the rest of ROS see the same interfaces a real Nav2 would
  # offer? Type names and RIHS01 hashes come from our definitions; ros2 reads
  # them off the graph.
  cat >"$WORK/nav2stub.yaml" <<'YAML'
bt_navigator:
  ros__parameters:
    fail_every: 2
    speed: 1.5
controller_server:
  ros__parameters:
    speed: 1.5
YAML
  # The stand-in starts with its lifecycle manual, which is what
  # `autostart:=False` does to a real Nav2: its six nodes sit Unconfigured
  # until something brings them up. That something is the application.
  bg "$WORK/nav2stub.log" "$BIN/nav2stub" -transport zenoh -lifecycle manual \
    -frames examples/nav2stub/frames.json -params "$WORK/nav2stub.yaml"
  stub_pid="$LAST_PID"
  sleep 4

  ros2run 25 action list -t >"$WORK/nav2_actions.log" 2>&1
  check "action server advertises nav2_msgs/action/NavigateToPose" \
    "$WORK/nav2_actions.log" "/navigate_to_pose [nav2_msgs/action/NavigateToPose]"
  check "the recovery behaviours advertise their nav2 types" \
    "$WORK/nav2_actions.log" "/spin [nav2_msgs/action/Spin]"

  # The stand-in's nodes are managed nodes as far as ROS is concerned, which is
  # what makes the application's lifecycle client worth having: ros2 lifecycle
  # is talking to the same protocol conductor serves and now also drives.
  ros2run 25 lifecycle nodes >"$WORK/nav2_lc_nodes.log" 2>&1
  check "ros2 lifecycle lists the stack's managed nodes" "$WORK/nav2_lc_nodes.log" "/bt_navigator"
  check "every server in the declared list is a managed node" "$WORK/nav2_lc_nodes.log" "/behavior_server"

  ros2run 25 lifecycle get /bt_navigator >"$WORK/nav2_lc_before.log" 2>&1
  check "the stack starts unconfigured, as autostart:=False leaves it" \
    "$WORK/nav2_lc_before.log" "unconfigured"

  # A latched pose is what amcl offers, so it is what the stand-in offers, and
  # ros2 has to agree about the profile or a real subscriber would not match.
  ros2run 25 topic info -v /amcl_pose >"$WORK/nav2_qos.log" 2>&1
  check "amcl_pose is advertised RELIABLE + TRANSIENT_LOCAL" "$WORK/nav2_qos.log" "Durability: TRANSIENT_LOCAL"

  # The declarations are meant to describe the real system, so the strongest
  # check available is to derive them from the running graph and require that
  # the committed file already says the same thing. A wrong type or QoS in
  # conductor.json is silence at runtime; this is what catches it.
  "$BIN/conductor" externals examples/nav2 -check >"$WORK/nav2_externals.log" 2>&1
  if [[ $? -eq 0 ]]; then
    echo "  PASS  committed externals match the live graph"
    PASS=$((PASS + 1))
  else
    echo "  FAIL  committed externals match the live graph"
    sed 's/^/        /' "$WORK/nav2_externals.log" | head -12
    FAIL=$((FAIL + 1))
  fi
  check "the graph supplies nav2's action types, not conductor.json" \
    "$WORK/nav2_externals.log" "nav2_msgs/action/NavigateToPose"

  # Then the application itself against it: it brings six managed nodes up
  # through the real lifecycle protocol, reaches a waypoint, and takes the
  # recovery branch when a goal is aborted.
  bg "$WORK/nav2_app.log" "$BIN/nav2" -transport zenoh -frames examples/nav2/frames.json
  app_pid="$LAST_PID"
  sleep 12

  # The claim that matters: a conductor application drove a ROS lifecycle
  # transition on somebody else's node, and ros2 agrees it happened.
  ros2run 25 lifecycle get /bt_navigator >"$WORK/nav2_lc_after.log" 2>&1
  check "the application drove the stack to active" "$WORK/nav2_lc_after.log" "active"
  ros2run 25 lifecycle get /map_server >"$WORK/nav2_lc_first.log" 2>&1
  check "the whole declared list came up, not just the node it talks to" \
    "$WORK/nav2_lc_first.log" "active"

  # Nav2 publishes TwistStamped on cmd_vel now (enable_stamped_cmd_vel), in the
  # costmap's base frame: both are readable by a stock ros2 CLI. This runs only
  # now because an inactive managed node publishes nothing — conductor gates
  # publishers on Active, so before the bringup above there was silence here,
  # which is the lifecycle working rather than a fault. The type is named
  # explicitly so this asserts the type rather than whatever is on the topic.
  ros2run 25 topic echo --once /cmd_vel geometry_msgs/msg/TwistStamped >"$WORK/nav2_cmd.log" 2>&1
  check "cmd_vel decodes as TwistStamped in base_link" "$WORK/nav2_cmd.log" "frame_id: base_link"

  # Long enough for a goal to be aborted and recovered from: the stand-in fails
  # every second goal, and a goal takes a couple of seconds at this speed.
  sleep 18
  { kill -9 "$app_pid"; wait "$app_pid"; } 2>/dev/null
  check "conductor drives the stack's lifecycle to active" "$WORK/nav2_app.log" "navigation stack active"
  check "conductor action client reaches a waypoint" "$WORK/nav2_app.log" "arrived waypoint=0"
  check "an aborted goal takes the recovery branch" "$WORK/nav2_app.log" "recovering because="
  check "the recovery retries the same waypoint" "$WORK/nav2_app.log" "recovered, retrying the waypoint"

  { kill -9 "$stub_pid"; wait "$stub_pid"; } 2>/dev/null
  pkill -9 -x nav2stub 2>/dev/null
  sleep 1
fi

if [[ "$GROUP" == all || "$GROUP" == moveit ]]; then
  echo "moveit:"
  # moveit_msgs is NOT installed here: these definitions are vendored upstream
  # text, hashed locally. So the question is whether the rest of ROS sees the
  # same action a real move_group offers — and whether the largest nested
  # message in common use survives the wire, which is what the application
  # driving it proves.
  bg "$WORK/moveitstub.log" "$BIN/moveitstub" -transport zenoh
  moveit_stub_pid="$LAST_PID"
  sleep 4

  ros2run 25 action list -t >"$WORK/moveit_actions.log" 2>&1
  check "move_group advertises moveit_msgs/action/MoveGroup" \
    "$WORK/moveit_actions.log" "/move_action [moveit_msgs/action/MoveGroup]"

  # The application: every planning request carries a MotionPlanRequest, whose
  # goal constraints come from the robot's SRDF.
  timeout 90 "$BIN/moveit" -transport zenoh -groups examples/moveit/groups.json \
    >"$WORK/moveit_app.log" 2>&1
  check "conductor plans against a real action server" "$WORK/moveit_app.log" "plan executed"
  check "the mission completes the job" "$WORK/moveit_app.log" "job complete"
  check "a failed plan takes the recovery branch" "$WORK/moveit_app.log" "returning to a known configuration"
  # The stand-in echoes the group it was asked to plan for, which is the name
  # the SRDF declares and the checker resolved.
  check "the planning group reaches move_group" "$WORK/moveitstub.log" "group=panda_arm"

  { kill -9 "$moveit_stub_pid"; wait "$moveit_stub_pid"; } 2>/dev/null
  pkill -9 -x moveitstub 2>/dev/null
  sleep 1
fi

if [[ "$GROUP" == all || "$GROUP" == turtlesim ]]; then
  echo "turtlesim:"
  # The whole tutorial against the real C++ turtlesim_node in one process:
  # subscription, publisher, parameter, three service clients and an action
  # client, all talking to a node that knows nothing about conductor.
  # turtlesim needs a display (WSLg provides one at :0); skip without it.
  if [[ -z "${DISPLAY:-}" ]] || ! ros2 pkg prefix turtlesim >/dev/null 2>&1; then
    echo "  SKIP  turtlesim tutorial (needs the turtlesim package and a display)"
  else
    bg "$WORK/turtlesim_node.log" ros2 run turtlesim turtlesim_node
    turtle_pid="$LAST_PID"
    sleep 4

    timeout 150 "$BIN/turtlesim" -transport zenoh >"$WORK/turtle.log" 2>&1
    check "turtlesim pose -> conductor subscription" "$WORK/turtle.log" "first pose from turtlesim"
    check "conductor cmd_vel drives the turtle in a square" "$WORK/turtle.log" "completed edge edge=4"
    check "conductor service client -> turtlesim spawn" "$WORK/turtle.log" "name=turtle2"
    check "conductor action client -> turtlesim rotate_absolute" "$WORK/turtle.log" "status=SUCCEEDED"
    check "tutorial runs to completion" "$WORK/turtle.log" "turtlesim tutorial complete"

    kill -9 "$turtle_pid" 2>/dev/null
    pkill -9 -x turtlesim_node 2>/dev/null # ros2 run forks the real node
    sleep 1
  fi
fi

echo
echo "$PASS passed, $FAIL failed"
[[ "$FAIL" -eq 0 ]]
