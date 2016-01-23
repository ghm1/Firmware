# target land position in WGS84 coordinates and calculated shift in Body NED coordinate system.
uint64 timestamp			# Timestamp (microseconds since system boot)

float64 lat				# target Latitude in degrees
float64 lon				# target Longitude in degrees
float32 alt				# target Altitude in meters (AMSL)

float32 x				# local target x position in NED frame in m
float32 y				# local target y position in vehicle NED frame in m
float32 z				# local target z position NED frame in m

float32 yaw				# yaw angle to target in vehicle NED frame
