### Used to control the head motors to adjust the angles
### two motors on head: horizontal & vertical

## enums for motor index
int8 VERTICAL		= 0
int8 HORIZONTAL		= 1

## msg data
bool[2] updated
int32[2] angle	# horizontal and vertical angles which between -150 to 150