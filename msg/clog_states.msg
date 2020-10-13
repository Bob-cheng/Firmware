uint64 timestamp # time since system start (microseconds)

# Position in local NED frame
float32 x				# North position in NED earth-fixed frame, (metres)
float32 y				# East position in NED earth-fixed frame, (metres)
float32 z				# Down position (negative altitude) in NED earth-fixed frame, (metres)

# Velocity in NED frame
float32 vx 				# North velocity in NED earth-fixed frame, (metres/sec)
float32 vy				# East velocity in NED earth-fixed frame, (metres/sec)
float32 vz				# Down velocity in NED earth-fixed frame, (metres/sec)

float32[3] gyroxyz				# Bias corrected angular velocity about X, Y, Z body axis in rad/s
float32[4] q				# Quaternion rotation from XYZ body frame to NED earth frame.
float32[4] actuators			# Actuator values in range [0,1]
