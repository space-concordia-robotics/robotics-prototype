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
computed_proximal_angle = proximal_min_angle
computed_distal_angle = distal_min_angle
computed_wrist_angle = wrist_min_angle
sample_size = 50


#base_start_point = [0,0]
#base_end_point = [0,base_length]
#proximal_start_point = base_end_point
#proximal_end_point = [proximal_length * math.cos(computed_proximal_angle),base_length + proximal_length * math.sin(computed_proximal_angle)]
#distal_start_point = proximal_end_point
#distal_end_point = [distal_start_point[0] + distal_length * math.cos(computed_distal_angle), distal_start_point[1] + distal_length * math.sin(computed_distal_angle)]
#wrist_start_point = distal_end_point
#wrist_end_point = [wrist_start_point[0] + wrist_length * math.cos(computed_wrist_angle),wrist_start_point[1] + wrist_length * math.sin(computed_wrist_angle)]

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

def GenerateRepresentativeCoordinates(PointArray, size = 3):
	Scale_Factor = 0.5
	
	for i in range(0,size):
		resized_array[i][0] *= Window_X * Scale_Factor
		resized_array[i][0] += Window_X/2
		resized_array[i][1] *= Window_Y * Scale_Factor
		resized_array[i][0] -= Window_Y/2
	
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

	global computed_proximal_angle
	global computed_distal_angle
	global computed_wrist_angle

	beta = math.atan2(Y - base_length , X) #Calculates beta, which is the sum of the angles of all links
	Wrist_X = X - wrist_length * math.cos(beta) #Calculates wrist X coordinate
	Wrist_Y = (Y - base_length) - wrist_length * math.sin(beta) #Calculates wrist Y coordinate

	if math.sqrt(pow(Wrist_X,2) + pow(Wrist_Y,2)) > proximal_length + distal_length: #Check to see if point is too far
		print('Error! Cannot reach point!')
		return 0

	cosine_distal_angle = (pow(Wrist_X,2) + pow(Wrist_Y,2) - pow(proximal_length,2) - pow(distal_length,2))/(2 * proximal_length * distal_length) #Calculates the cosine of the angle of link 2
	computed_distal_angle = math.acos(cosine_distal_angle) #Proper angle is reached. NOTE: Depending on speed of function acos()
						#an approximation or different calculation method should be
						#used to speed computation time

	if computed_distal_angle > distal_max_angle or computed_distal_angle < distal_min_angle: #Check to see if third angle is within bounds
		disp('Error! Angle 3 is not within bounds')
		return 0

	aphi = (pow(Wrist_X,2) + pow(Wrist_Y,2) + pow(proximal_length,2) - pow(distal_length,2))/(2 * math.sqrt(pow(Wrist_X,2) + pow(Wrist_Y,2)) * proximal_length) #Determine cos of phi
	phi = math.acos(aphi) #Phi is the angle between the first link and the straight line
				  #from the base to the end effector

	#Depending on the configuration of the arm, phi and beta can be used to
	#find the second angle

	if computed_distal_angle <= 0:
		computed_proximal_angle = beta + phi

	if computed_distal_angle > 0:
		computed_proximal_angle = beta - phi

	#If computed_proximal_angle is too small, calculations will restart with computed_distal_angle being negative
	if computed_proximal_angle > proximal_max_angle or computed_proximal_angle < proximal_min_angle:
		#disp(computed_proximal_angle)
		computed_distal_angle = -computed_distal_angle

	if computed_distal_angle < 0:
		computed_proximal_angle = beta + phi

	if computed_distal_angle > 0:
		computed_proximal_angle = beta - phi

	#Check to see if second angle is within bounds
	if computed_proximal_angle > proximal_max_angle or computed_proximal_angle < proximal_min_angle:
		print('Error! Angle 2 is not within bounds')
		return 0

	#Knowing the sum of all angles (beta) and two of the three angles,
	#the final angle (angle four) can be found:

	computed_wrist_angle = beta - (computed_proximal_angle + computed_distal_angle)
	return 1


def UpdateArmSetValues():

	base_start_point = [0,0]
	base_end_point = [base_start_point[0], base_start_point[1] - base_length]
	proximal_start_point = base_end_point
	proximal_end_point =  [proximal_start_point[0] + proximal_length * math.cos(computed_proximal_angle) , proximal_start_point[1] - proximal_length * math.sin(computed_proximal_angle)]
	distal_start_point = proximal_end_point
	distal_end_point =  [distal_start_point[0] + distal_length * math.cos(computed_distal_angle) , distal_start_point[1] - distal_length * math.sin(computed_distal_angle)]
	wrist_start_point = distal_end_point
	wrist_end_point = [wrist_start_point[0] + wrist_length * math.cos(computed_wrist_angle) , wrist_start_point[1] - wrist_length * math.sin(computed_wrist_angle)]
	
	return [ [base_start_point, base_end_point], [proximal_start_point, proximal_end_point], [distal_start_point, distal_end_point] , [wrist_start_point, wrist_end_point] ]






#########################MAIN#########################


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
		set_point = [0.4,0.8,0.3]
	
		error = ComputeIK(set_point[0], set_point[1], set_point[2])
		Arm_Actual_Position = UpdateArmSetValues()
		Arm_Representative_Position = GenerateRepresentativeCoordinates(Arm_Actual_Position)
		
		#Draw the arm
		draw(Sideview, screen, Arm_Representative_Position)
		
		#Draw a circle around target point
		desired_point = GenerateRepresentativeCoordinates(set_point , 0)
		pygame.draw.circle(screen, BLUE, desired_point , 15, 6)
		
	#WORKSPACE MODE
	if mode is 0:
		for i in range(0, math.pow(sample_size,3) ):
			pygame.draw.circle(screen, BLUE, desired_point , 1)
	
	
	#This is required to update the display
    pygame.display.flip()

pygame.quit()
