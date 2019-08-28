from math import radians, degrees, sin, cos, atan2, sqrt

def Direction (p1_lat, p1_lon, p2_lat, p2_lon):
	'''
	This is a function to calculate the direction from p1 (point1) to p2 (point2)
	It uses the Haverside formulas
	Inputs are p1 and p2 latitudes and longitudes in degrees
	Output is the direction from p1 to p2 in degrees ranging from 0 to 360
	Measured from the N-S line, left-handed rule is used (E=90, W=270)
	'''

	p1_lat = radians(p1_lat)
	p1_lon = radians(p1_lon)
	p2_lat = radians(p2_lat)
	p2_lon = radians(p2_lon)

	delta_lat = p2_lat - p1_lat
	delta_lon = p2_lon - p1_lon

	X = cos(p2_lat) * sin(delta_lon)
	Y = cos(p1_lat) * sin(p2_lat) - sin(p1_lat) * cos(p2_lat) * cos(delta_lon)

	Real_dir = atan2(X,Y)
	Real_dir = degrees(Real_dir)

	if Real_dir < 0:
		Real_dir += 360
	return Real_dir

def Distance (p1_lat, p1_lon, p2_lat, p2_lon):
	'''
	This is a function to calculate the distance between p1 (point1) to p2 (point2)
	It uses the Haverside formulas
	Inputs are p1 and p2 latitudes and longitudes in degrees
	Output is the distance between p1 and p2 in meters
	'''

	R = 6371000 #radius of Earth

	p1_lat = radians(p1_lat)
	p1_lon = radians(p1_lon)
	p2_lat = radians(p2_lat)
	p2_lon = radians(p2_lon)

	delta_lat = p2_lat - p1_lat
	delta_lon = p2_lon - p1_lon

	A = ((sin(delta_lat / 2)) ** 2) + cos(p2_lat) * cos(p1_lat) * ((sin(delta_lon / 2)) ** 2)
	C = 2 * atan2( sqrt(A), sqrt(1 - A) )
	return R * C

def Turning (Dest_dir, heading):
	'''
	This function is to determine the needed adjustment in direction to go to destination
	Input is the direction to destination and rover heading in degrees
	Output is the needed adjustment in degrees
	if the output is positive number, it means the rover needs to turn right
	if the output is negative number, it means the rover needs to turn left
	'''

	shift = Dest_dir - heading
	if shift > 0 and shift < 180:
		return shift
	elif shift > 180 and shift < 360:
		return shift - 360
	elif shift < 0 and shift > -180:
		return shift
	elif shift < -180 and shift > -360:
		return shift + 360
	elif shift == 360 or shift == -360:
		return 0

#BS = {'lat':0 , 'lon':0}
#Rover = {'lat':0 , 'lon':0}

#BS['lat'] = float(input("Enter BS latitude :"))
#BS['lon'] = float(input("Enter BS longitude :"))
#Rover['lat'] = float(input("Enter Rover latitude :"))
#Rover['lon'] = float(input("Enter Rover longitude :"))

#Dir = Direction(BS['lat'],BS['lon'],Rover['lat'],Rover['lon'])
#Dis = Distance(BS['lat'],BS['lon'],Rover['lat'],Rover['lon'])

#print('The direction is : {} degrees'.format(round(Dir)))
#print('The Distance is : {} meters'.format(round(Dis)))
