# This message is to start and stop recording a rosbag of the system.
# It takes a bool that indicates the start/stop, and the corporation name, the condition, and subject number.
bool rec # Switch to start or stop recording
string corp # Corporation handle (GNR, RSBL, ERIEE)
string cond # Condition handle (CTRL, HHAFD, FULL)
int32 subnum # Suject number of 00 type