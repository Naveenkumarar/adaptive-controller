std_msgs/Header header
# IMU
float32 ax  # accelometer longitudinal reading
float32 ay  # accelometer lateral reading
float32 az  # accelometer updown reading
float32 roll_rate   # gyroscope roll rate
float32 pitch_rate  # gyroscope pitch rate
float32 yaw_rate    # gyroscope yaw rate

# encoder
float32 encoderCounts

# power
float32 current
float32 batteryVoltage

# time
float32 filter_time

# commands
float32 throttle_cmd
float32 steering_angle