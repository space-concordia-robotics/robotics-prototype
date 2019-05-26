import math

def Direction (p1_lat,p1_lon,p2_lat,p2_lon):
	'''
	This is a function to calculate the direction from p1 (point1) to p2 (point2)
	Inputs are p1 and p2 latitudes and longitudes in degrees
	Output is the direction from p1 to p2 in degrees
	'''
	
	p1_lat = p1_lat * (math.pi / 180)
	p1_lon = p1_lon * (math.pi / 180)
	p2_lat = p2_lat * (math.pi / 180)
	p2_lon = p2_lon * (math.pi / 180)
	
	delta_lat = p2_lat - p1_lat
	delta_lon = p2_lon - p1_lon
	
	X = math.cos(p2_lat) * math.sin(delta_lon)
	Y = math.cos(p1_lat) * math.sin(p2_lat) - math.sin(p1_lat) * math.cos(p2_lat) * math.cos(delta_lon)
	
	Real_dir = math.atan2(X,Y)
	
	Real_dir = Real_dir * (180 / math.pi)
	
	if Real_dir < 0:
		Real_dir = Real_dir + 360
		
	return Real_dir
	
def Distance (p1_lat,p1_lon,p2_lat,p2_lon):
	'''
	This is a function to calculate the distance between p1 (point1) to p2 (point2)
	Inputs are p1 and p2 latitudes and longitudes in degrees
	Output is the distance between p1 and p2 in meters
	'''
	R = 6371000
	p1_lat = p1_lat * (math.pi / 180)
	p1_lon = p1_lon * (math.pi / 180)
	p2_lat = p2_lat * (math.pi / 180)
	p2_lon = p2_lon * (math.pi / 180)
	
	delta_lat = p2_lat - p1_lat
	delta_lon = p2_lon - p1_lon
	
	A = ((math.sin(delta_lat / 2)) ** 2) + math.cos(p2_lat) * math.cos(p1_lat) * ((math.sin(delta_lon / 2)) ** 2)
	C = 2 * math.atan2(math.sqrt(A) , math.sqrt(1 - A))
	return R * C

def Turning (Dest_dir , heading):
	'''
	This function is to determine the needed adjustment in direction to go to destination
	Input is the direction to destination and rover heading in degrees
	Output is the needed adjustment in degrees
	if the output is positive number , it means the rover needs to turn right
	if the output is negative number , it means the rover needs to turn left
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

