# Heartbeat published by the safety monitor: current safety mode and
# whether the command stream has gone stale.
uint8 MODE_PATROLLING=0
uint8 MODE_ESTOPPED=1

std_msgs/Header header
uint8 mode
bool cmd_stale
