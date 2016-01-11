# detected points from pixy camera
uint8 PIXY_CAM_PTS_MAX_POINTS = 10

uint64 timestamp		# in microseconds since system start
uint32 count    		# number of actual points
float32[10] x 			# normalize x image coordinate
float32[10] y 			# normalize y image coordinate
float32[10] width 		# normalize x image coordinate
float32[10] heigth 		# normalize y image coordinate