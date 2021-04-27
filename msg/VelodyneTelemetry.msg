uint8 PPS_ABSENT=0
uint8 PPS_SYNC=1
uint8 PPS_LOCKED=2
uint8 PPS_ERR=3

std_msgs/Header header
int8 temp_top
int8 temp_bot
uint32 usec_since_toh
uint8 pps_status
uint8 thermal_status
int8 temp_at_last_shutdown
int8 temp_at_power_up
nmea_msgs/Gprmc gprmc
