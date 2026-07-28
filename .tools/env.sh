# User-space rmw_zenoh overlay (no sudo needed): extracted from the
# ros-lyrical-rmw-zenoh-cpp / zenoh-cpp-vendor debs in .tools/debs.
# Usage: source .tools/env.sh
export CONDUCTOR_OVERLAY=/home/ryan/ros2-framework/.tools/overlay/opt/ros/lyrical
export ZENOH_C_HOME=$CONDUCTOR_OVERLAY/opt/zenoh_cpp_vendor
source /opt/ros/lyrical/setup.bash
export AMENT_PREFIX_PATH=$CONDUCTOR_OVERLAY:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=$CONDUCTOR_OVERLAY/lib:$ZENOH_C_HOME/lib:${LD_LIBRARY_PATH:-}
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# cgo flags for conductor's zenoh transport: build against prebuilt zenoh-c
# 1.9.0 (zenoh-go v1.9.0 needs symbols the vendored 1.8.0 lacks; the zenoh 1.x
# wire protocol is cross-compatible). DT_RPATH (--disable-new-dtags) pins our
# binaries to 1.9 even though LD_LIBRARY_PATH points ROS processes at 1.8.
export ZENOH_C_GO=/home/ryan/ros2-framework/.tools/zenoh-c-1.9.0
export CGO_CFLAGS="-I$ZENOH_C_GO/include"
export CGO_LDFLAGS="-L$ZENOH_C_GO/lib -Wl,-rpath,$ZENOH_C_GO/lib -Wl,--disable-new-dtags"
