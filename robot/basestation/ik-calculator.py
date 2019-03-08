############################################################
############SCRB 2019 INVERSE KINEMATICS PACKAGE############
############################################################

#NOTE: The first angle in the code (a1a) is the angle that the base makes
#with the horizontal. Since the base is immobile, this angle is assumed to
#be constant. In fact, the properties of the base are not taken into
#account in this code but are simply added to the final figure.
#For this reason, each angle has a higher coefficient than its
#corresponding link. E.g. link 1 corresponds to angle computed_proximal_angle.

#This code uses a closed form solution to determine the orientation of the
#joints of a planar RRR arm for its end effector to reach a given point.For
#more information about workings of the code, contact Maxim Kaller.

import math
import pygame


############MANIPULATOR PHYSICAL PARAMETERS############


#LENGTHS:
rover_height = 0.366 #m
base_length = 0.103 #m
proximal_length = 0.413 #m
distal_length = 0.406 #m
wrist_length = 0.072 + 0.143 #m (until tip of fingers)
length_array = [proximal_length, distal_length, wrist_length]

#ANGLES (IN RADIANS):
proximal_max_angle = math.pi
proximal_min_angle = -math.pi
distal_max_angle = math.pi
distal_min_angle = -math.pi
wrist_max_angle = math.pi
wrist_min_angle = -math.pi
minmax = [[proximal_min_angle,proximal_max_angle], [distal_min_angle, distal_max_angle], [wrist_min_angle, wrist_max_angle]]

#INITIALIZE GLOBAL VARIABLES
computed_angles = [proximal_min_angle, distal_min_angle, wrist_min_angle]
print(computed_angles)
sample_size = 50

###################PYGAME PARAMETERS###################
Window_X = 640
Window_Y = 480

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)

###############PYGAME FUNCTIONS###############
def draw(self, surf, array):
	pygame.draw.line(surf, self.color, array[0][0], array[0][1], self.width)
	pygame.draw.line(surf, self.color, array[1][0], array[1][1], self.width)
	pygame.draw.line(surf, self.color, array[2][0], array[2][1], self.width)
	pygame.draw.line(surf, self.color, array[3][0], array[3][1], self.width)

def OffsetPoint(Point):
	return [Point[0] + Window_X/2, Point[1] + Window_Y/2]

def ResizePoint(Point):
	return [Point[0] * Window_X/2, Point[1] * Window_Y/2]

def GenerateRepresentativeCoordinates(PointArray, size = 4):
	Scale_Factor = 0.5

	if size is 0:
		resized_array = [0,0]
		resized_array[0] = int(PointArray[0] * Window_X * Scale_Factor + Window_X/2)
		resized_array[1] = int(- PointArray[1] * Window_Y * Scale_Factor + Window_Y/2)

	else:
		resized_array = [ [[0,0],[0,0]] for x in range(size)]
		for i in range(0,size):
			for j in range(0,1):
				resized_array[i][0][0] = PointArray[i][0][0] * Window_X * Scale_Factor + Window_X/2
				resized_array[i][1][0] = PointArray[i][1][0] * Window_X * Scale_Factor + Window_X/2
				resized_array[i][0][1] = PointArray[i][0][1] * Window_Y * Scale_Factor + Window_Y/2
				resized_array[i][1][1] = PointArray[i][1][1] * Window_Y * Scale_Factor + Window_Y/2

	return resized_array


################PYGAME CLASSES################
class ProjectionView:
	def __init__(self, color, X, Y, width = 6):
		self.color = color
		self.width = width
		self.X = X
		self.Y = Y
        #MISSING FUNCTIONALITY FOR X AND Y



#######################FUNCTIONS#######################


def ComputeWorkspace(joint_array, joint_num,minmax_array, length_array):
	global sample_size

	for j1 in range(minmax_array[0][0],minmax_array[0][1], (minmax_array[0][1] - minmax_array[0][0])/sample_size):
		for j2 in range(minmax_array[1][0],minmax_array[1][1], (minmax_array[1][1] - minmax_array[1][0])/sample_size):
			for j3 in range(minmax_array[2][0],minmax_array[2][1], (minmax_array[2][1] - minmax_array[2][0])/sample_size):
				pt[ 50 * 50 * j1 + 50 * j2 + j3][0] = length_array[0] * math.cos(j1) + length_array[1] * math.cos(j2) + length_array[2] * math.cos(j3)
				pt[ 50 * 50 * j1 + 50 * j2 + j3][1] = length_array[0] * math.sin(j1) + length_array[1] * math.sin(j2) + length_array[2] * math.sin(j3)
	return pt



def ComputeIK(X, Y, Z):

	global computed_angles
	solution_angles = [[0,0,0], [0,0,0]]
	solution_usable = [True, True]

	#Check to see if point is too far
	#REPLACE LATER WITH FANCIER SHIT

	beta = math.atan2(Y - base_length , X) #Calculates beta, which is the sum of the angles of all links
	Wrist_X = X - wrist_length * math.cos(beta) #Calculates wrist X coordinate
	Wrist_Y = (Y - base_length) - wrist_length * math.sin(beta) #Calculates wrist Y coordinate

	if math.sqrt(pow(Wrist_X,2) + pow(Wrist_Y,2)) > proximal_length + distal_length:
		print('ERROR: SET POINT BEYOND ARM WORKSPACE')
		return 0


	cosine_distal_angle = (pow(Wrist_X,2) + pow(Wrist_Y,2) - pow(proximal_length,2) - pow(distal_length,2))/(2 * proximal_length * distal_length)
	solution_angles[0][1] = math.acos(cosine_distal_angle)
	solution_angles[1][1] = -solution_angles[0][1]


	#Calculate phi: Angle between tangent line and proximal link
	aphi = (pow(Wrist_X,2) + pow(Wrist_Y,2) + pow(proximal_length,2) - pow(distal_length,2))/(2 * math.sqrt(pow(Wrist_X,2) + pow(Wrist_Y,2)) * proximal_length)
	phi = math.acos(aphi)

	#Depending on the configuration of the arm, phi and beta can be used to
	#find the second angle

	if solution_angles[0][1] <= 0:
		solution_angles[0][0] = beta + phi
		solution_angles[1][0] = beta - phi
	else:
		solution_angles[0][0] = beta - phi
		solution_angles[1][0] = beta + phi

	#WRIST ANGLE CALCULATION: REPLACE WITH FANCY SHIT LATER
	solution_angles[0][2] = beta - (solution_angles[0][0] + solution_angles[0][1])
	solution_angles[1][2] = beta - (solution_angles[1][0] + solution_angles[1][1])


	#ELIMIATION OF ONE OF THE ANGLE SETS

	for i in range(3):
		if solution_angles[0][i] > minmax[i][1] or solution_angles[0][1] < minmax[i][0]:
			solution_usable[0] = False
			solution = 1
			print('One Solution is not possible')
		if solution_angles[1][i] > minmax[i][1] or solution_angles[1][1] < minmax[i][0]:
			solution_usable[1] = False
			solution = 0
			print('One Solution is not possible')

	if (solution_usable[0] is False) and (solution_usable[1] is False):
		print('Error: No arm configuration available for specified set point')
		return 0

	elif (solution_usable[0] is True) and (solution_usable[1] is True):
		error0 = pow(solution_angles[0][0] - computed_angles[0],2) + pow(solution_angles[0][1] - computed_angles[1],2) + pow(solution_angles[0][2] - computed_angles[2],2)
		error1 = pow(solution_angles[1][0] - computed_angles[0],2) + pow(solution_angles[1][1] - computed_angles[1],2) + pow(solution_angles[1][2] - computed_angles[2],2)
		print(solution_angles[0])
		print(solution_angles[1])
		print(error0)
		print(error1)
		if error0 > error1:
			solution = 1
			print('Solution 1 chosen')

		else:
			solution = 0
			print('Solution 0 chosen')

	return solution_angles[solution]


def UpdateArmSetValues(angles):

	base_start_point = [0,0]
	base_end_point = [base_start_point[0], base_start_point[1] - base_length]
	proximal_start_point = base_end_point
	proximal_end_point =  [proximal_start_point[0] + proximal_length * math.cos(angles[0]) , proximal_start_point[1] - proximal_length * math.sin(angles[0])]
	distal_start_point = proximal_end_point
	distal_end_point =  [distal_start_point[0] + distal_length * math.cos(angles[0] + angles[1]) , distal_start_point[1] - distal_length * math.sin(angles[0] + angles[1])]
	wrist_start_point = distal_end_point
	wrist_end_point = [wrist_start_point[0] + wrist_length * math.cos(angles[0] + angles[1] + angles[2]) , wrist_start_point[1] - wrist_length * math.sin(angles[0] + angles[1] + angles[2])]

	return [ [base_start_point, base_end_point], [proximal_start_point, proximal_end_point], [distal_start_point, distal_end_point] , [wrist_start_point, wrist_end_point] ]


#########################MAIN#########################

#################ARM HOME INIT#################
Arm_Actual_Position = UpdateArmSetValues(computed_angles)
Arm_Representative_Position = GenerateRepresentativeCoordinates(Arm_Actual_Position)

#################PYGAME INIT#################
pygame.init()
pygame.font.init()
Sideview = ProjectionView(RED, Window_X/2, Window_Y/2)
screen = pygame.display.set_mode((Window_X,Window_Y))
pygame.display.set_caption("Inverse Kinematics Module")
mode = 1

done = False
clock = pygame.time.Clock()


#################PYGAME LOOP#################
while not done:


	#Check if user closed window
    clock.tick(10)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done=True


	#Update arm view
    screen.fill(WHITE)

	#INVERSE KINEMATICS MODE
    if mode is 1:
    	set_point = [0.4,-0.5,0.3]
    	set_point_2D = [set_point[0], set_point[1]]
    	computed_angles = ComputeIK(set_point[0], set_point[1], set_point[2])
    	Arm_Actual_Position = UpdateArmSetValues(computed_angles)
    	Arm_Representative_Position = GenerateRepresentativeCoordinates(Arm_Actual_Position)

		#Draw the arm
    	#print(Arm_Actual_Position)
    	#print(Arm_Representative_Position)
    	draw(Sideview, screen, Arm_Representative_Position)

		#Draw a circle around target point
    	desired_point = GenerateRepresentativeCoordinates(set_point_2D , 0)
    	#print(desired_point)
    	pygame.draw.circle(screen, BLUE, desired_point , 15, 3)

	#WORKSPACE MODE
    if mode is 0:
    	for i in range(0, math.pow(sample_size,3) ):
    		pygame.draw.circle(screen, BLUE, desired_point , 1)


	#This is required to update the display
    pygame.display.flip()

pygame.quit()
