# target land position in WGS84 coordinates and calculated shift in Body NED coordinate system.
uint64 timestamp			# Timestamp (microseconds since system boot)

float64 lat				# Latitude in degrees
float64 lon				# Longitude in degrees
float32 alt				# Altitude in meters (AMSL)

float32 x				# x offset to target in vehicle NED frame in m
float32 y				# y offset to target in vehicle NED frame in m
float32 z				# z offset to target in vehicle NED frame in m

float32 yaw				# yaw angle to target in vehicle NED frame
