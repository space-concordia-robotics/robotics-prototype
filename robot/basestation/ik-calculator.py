############################################################
############SCRB 2019 INVERSE KINEMATICS PACKAGE############
############################################################

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
length_array = [base_length, proximal_length, distal_length, wrist_length]

#ANGLES (IN RADIANS):
proximal_max_angle = math.pi
proximal_min_angle = -math.pi
distal_max_angle = math.pi
distal_min_angle = -math.pi
wrist_max_angle = math.pi
wrist_min_angle = -math.pi
minmax = [[0,0], [proximal_min_angle,proximal_max_angle], [distal_min_angle, distal_max_angle], [wrist_min_angle, wrist_max_angle]]

#INITIALIZE GLOBAL VARIABLES
computed_angles = [0, proximal_min_angle, distal_min_angle, wrist_min_angle]
sample_size = 10
workspace_rep = [[0,0] for x in range(pow(sample_size,3))]

###################PYGAME PARAMETERS###################
Window_X = 640
Window_Y = 480
increment = 0.02

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)

###############PYGAME FUNCTIONS###############
# expected input: array of points for path from start to end
def draw(self, pts):
	for i in range(pts):
		pygame.draw.line(self.screen, self.color, self.projected_position[i][0], self.projected_position[i][1], self.width)


def drawText(screen, text, color, pos):
	font = pygame.font.Font(None, 36)
	text_surface = font.render(text, True, color)
	screen.blit(text_surface, pos)

def GenerateRepresentativeCoordinates(PointArray, offset_x, offset_y, size = 4):
	Scale_Factor = 0.5

	if size is 0:
		resized_array = [0,0]
		resized_array[0] = int(PointArray[0] * Window_X * Scale_Factor + offset_x)
		resized_array[1] = int(- PointArray[1] * Window_Y * Scale_Factor + offset_y)
	else:
		resized_array = [ [[0,0],[0,0]] for x in range(size)]
		for i in range(0,size):
			resized_array[i][0][0] = PointArray[i][0][0] * Window_X * Scale_Factor + offset_x
			resized_array[i][0][1] = PointArray[i][0][1] * Window_Y * Scale_Factor + offset_y
			resized_array[i][1][0] = PointArray[i][1][0] * Window_X * Scale_Factor + offset_x
			resized_array[i][1][1] = PointArray[i][1][1] * Window_Y * Scale_Factor + offset_y

	return resized_array


################PYGAME CLASSES################
class ProjectionView:
	def __init__(self, color, X, Y, screen, actual_position = 0, projected_position = 0, width = 6, font = 'verdana12'):
		self.color = color
		self.width = width
		self.X = X
		self.Y = Y
		self.screen = screen
        #MISSING FUNCTIONALITY FOR X AND Y



#######################FUNCTIONS#######################


def ComputeWorkspace(joint_num,minmax_array, length_array):
	global sample_size
	pt = [[0,0] for x in range(pow(sample_size,3))]
	incremented_angle = [0,0,0]

	for j1 in range(sample_size):
		incremented_angle[0] = minmax_array[1][0] + j1*(minmax_array[1][1] - minmax_array[1][0])/(sample_size - 1)
		for j2 in range(sample_size):
			incremented_angle[1] = minmax_array[2][0] + j2*(minmax_array[2][1] - minmax_array[2][0])/(sample_size - 1)
			for j3 in range(sample_size):
					incremented_angle[2] = minmax_array[3][0] + j3*(minmax_array[3][1] - minmax_array[3][0])/(sample_size - 1)

					pt[ sample_size * sample_size * j1 + sample_size * j2 + j3][0] = length_array[1] * math.cos(incremented_angle[0]) + length_array[2] * math.cos(incremented_angle[1] + incremented_angle[0]) + length_array[3] * math.cos(incremented_angle[2] + incremented_angle[1] + incremented_angle[0])
					pt[ sample_size * sample_size * j1 + sample_size * j2 + j3][1] = length_array[0]-length_array[0] - length_array[1] * math.sin(-math.pi + incremented_angle[0]) - length_array[2] * math.sin(incremented_angle[1] + incremented_angle[0]) - length_array[3] * math.sin(incremented_angle[2] + incremented_angle[1] + incremented_angle[0])
	return pt


def ComputeIK(X, Y, Z, wrist_angle = -1):

	global computed_angles
	solution_angles = [[0,0,0,0], [0,0,0,0]]
	solution_usable = [True, True]

	#Base rotation CALCULATION
	solution_angles[0][0] = solution_angles[1][0] = math.atan( Z/X)
	R = math.sqrt( pow(X,2) + pow(Z,2) )
	if (X <= 0 or Z <=0 ):
		R *= -1
		solution_angles[0][0] *= -1
		solution_angles[1][1] *= -1

	if(wrist_angle is -1):
		beta = math.atan2(Y - base_length , R) #Calculates beta, which is the sum of the angles of all links
		Wrist_X = R - wrist_length * math.cos(beta) #Calculates wrist X coordinate
		Wrist_Y = (Y - base_length) - wrist_length * math.sin(beta) #Calculates wrist Y coordinate
	else:
		Wrist_X = R - wrist_length * math.cos(wrist_angle) #Calculates wrist X coordinate
		Wrist_Y = (Y - base_length) - wrist_length * math.sin(wrist_angle) #Calculates wrist Y coordinate
		beta = math.atan2(Wrist_Y  , Wrist_X) #Calculates beta, which is the sum of the angles of all links

	if math.sqrt(pow(Wrist_X,2) + pow(Wrist_Y,2)) > proximal_length + distal_length:
		print('ERROR: SET POINT BEYOND ARM WORKSPACE')
		return computed_angles


	cosine_distal_angle = (pow(Wrist_X,2) + pow(Wrist_Y,2) - pow(proximal_length,2) - pow(distal_length,2))/(2 * proximal_length * distal_length)
	solution_angles[0][2] = math.acos(cosine_distal_angle)
	solution_angles[1][2] = -solution_angles[0][2]


	#Calculate phi: Angle between tangent line and proximal link
	aphi = (pow(Wrist_X,2) + pow(Wrist_Y,2) + pow(proximal_length,2) - pow(distal_length,2))/(2 * math.sqrt(pow(Wrist_X,2) + pow(Wrist_Y,2)) * proximal_length)
	phi = math.acos(aphi)

	#Depending on the configuration of the arm, phi and beta can be used to
	#find the second angle

	if solution_angles[0][2] <= 0:
		solution_angles[0][1] = beta + phi
		solution_angles[1][1] = beta - phi
	else:
		solution_angles[0][1] = beta - phi
		solution_angles[1][1] = beta + phi

	if(wrist_angle is -1):
		solution_angles[0][3] = beta - (solution_angles[0][1] + solution_angles[0][2])
		solution_angles[1][3] = beta - (solution_angles[1][1] + solution_angles[1][2])
	else:
		solution_angles[0][3] = wrist_angle - (solution_angles[0][1] + solution_angles[0][2])
		solution_angles[1][3] = wrist_angle - (solution_angles[1][1] + solution_angles[1][2])

	#ELIMIATION OF ONE OF THE ANGLE SETS

	for i in range(1,4):
		if solution_angles[0][i] > minmax[i][1] or solution_angles[0][1] < minmax[i][0]:
			solution_usable[0] = False
			solution = 1
		if solution_angles[1][i] > minmax[i][1] or solution_angles[1][1] < minmax[i][0]:
			solution_usable[1] = False
			solution = 0

	if (solution_usable[0] is False) and (solution_usable[1] is False):
		print('Error: No arm configuration available for specified set point')
		return 0

	elif (solution_usable[0] is True) and (solution_usable[1] is True):
		error0 = pow(solution_angles[0][1] - computed_angles[1],2) + pow(solution_angles[0][2] - computed_angles[2],2) + pow(solution_angles[0][3] - computed_angles[3],2)
		error1 = pow(solution_angles[1][1] - computed_angles[1],2) + pow(solution_angles[1][2] - computed_angles[2],2) + pow(solution_angles[1][3] - computed_angles[3],2)
		if error0 > error1:
			solution = 1

		else:
			solution = 0

	return solution_angles[solution]

# Side view
def UpdateArmSideValues(angles):

	base_start_point = [0,0]
	base_end_point = [base_start_point[0], base_start_point[1] - base_length]
	proximal_start_point = base_end_point
	proximal_end_point =  [proximal_start_point[0] + proximal_length * math.cos(angles[1]) , proximal_start_point[1] - proximal_length * math.sin(angles[1])]
	distal_start_point = proximal_end_point
	distal_end_point =  [distal_start_point[0] + distal_length * math.cos(angles[1] + angles[2]) , distal_start_point[1] - distal_length * math.sin(angles[1] + angles[2])]
	wrist_start_point = distal_end_point
	wrist_end_point = [wrist_start_point[0] + wrist_length * math.cos(angles[1] + angles[2] + angles[3]) , wrist_start_point[1] - wrist_length * math.sin(angles[1] + angles[2] + angles[3])]

	return [ [base_start_point, base_end_point], [proximal_start_point, proximal_end_point], [distal_start_point, distal_end_point] , [wrist_start_point, wrist_end_point] ]

# top view
def UpdateArmTopValues(angles):
	arm_start_point = [0,0]
	arm_radius = proximal_length * math.cos(angles[1]) + distal_length * math.cos(angles[1] + angles[2]) + wrist_length * math.cos(angles[1] + angles[2] + angles[3])
	arm_end_point = [arm_radius * math.cos(angles[0]), -arm_radius * math.sin(angles[0])]

	return [[arm_start_point, arm_end_point]]

#########################MAIN#########################
#################PYGAME INIT#################
pygame.init()
pygame.font.init()
screen = pygame.display.set_mode((Window_X,Window_Y))
pygame.display.set_caption("Inverse Kinematics Module")

Sideview = ProjectionView(RED, Window_X/4, 3*Window_Y/4, screen)
Topview = ProjectionView(RED, 3*Window_X/4, 3*Window_Y/8, screen)

done = False
clock = pygame.time.Clock()

set_point = [0.5, 0.6, 0.5]
wrist_angle = 0

workspace_enable = True
if workspace_enable:
	workspace = ComputeWorkspace(4,minmax, length_array)
	for i in range(0, int(math.pow(sample_size,3))):
	    workspace_rep[i] = GenerateRepresentativeCoordinates(workspace[i] , Sideview.X, Sideview.Y, 0)
#################PYGAME LOOP#################
while not done:

	#Check if user closed window
    clock.tick(10)
    for event in pygame.event.get():
    	if event.type == pygame.QUIT:
            done=True
    	if event.type == pygame.KEYDOWN:
            	if event.key == pygame.K_UP:
            		set_point[1] += increment
            	if event.key == pygame.K_DOWN:
            		set_point[1] -= increment
            	if event.key == pygame.K_RIGHT:
            		set_point[0] += increment
            	if event.key == pygame.K_LEFT:
            		set_point[0] -= increment
            	if event.key == pygame.K_PERIOD:
            		set_point[2] += increment
            	if event.key == pygame.K_COMMA:
            		set_point[2] -= increment

		#Update arm view
    	screen.fill(WHITE)

	#INVERSE KINEMATICS MODE
    	set_point_2D = [math.sqrt( pow(set_point[0],2) + pow(set_point[2],2) ), set_point[1]]
    	if (set_point[0] <= 0 and set_point[2] <= 0):
    		set_point_2D[0] *= -1
    	top_point = [set_point[0], set_point[2]]
    	computed_angles = ComputeIK(set_point[0], set_point[1], set_point[2] , wrist_angle)


		#Sideview
    	Sideview.actual_position = UpdateArmSideValues(computed_angles)
    	Sideview.projected_position = GenerateRepresentativeCoordinates(Sideview.actual_position, Sideview.X, Sideview.Y)
    	draw(Sideview, 4)
    	pygame.draw.circle(screen, RED, [int(Sideview.projected_position[0][0][0]),int(Sideview.projected_position[0][0][1])] , 10)
    	drawText(screen, "Side View", BLACK, [Sideview.X, Sideview.Y + 50])


		#Topview
    	Topview.actual_position = UpdateArmTopValues(computed_angles)
    	Topview.projected_position = GenerateRepresentativeCoordinates(Topview.actual_position, Topview.X, Topview.Y, 1)
    	draw(Topview, 1)
    	pygame.draw.circle(screen, RED, [int(Topview.projected_position[0][0][0]),int(Topview.projected_position[0][0][1])] , 10)
    	drawText(screen, "Top View", BLACK, [Topview.X, Topview.Y + 50])
    	#drawText(screen, computed_angles, BLACK, [0,0])

		#Draw a circle around target point
    	desired_side_point = GenerateRepresentativeCoordinates(set_point_2D , Sideview.X, Sideview.Y, 0)
    	pygame.draw.circle(screen, BLUE, desired_side_point , 15, 3)

    	desired_top_point = GenerateRepresentativeCoordinates(top_point , Topview.X, Topview.Y, 0)
    	pygame.draw.circle(screen, BLUE, desired_top_point , 15, 3)

	#WORKSPACE MODE
    	if workspace_enable is True:
    		for i in range(0, int(math.pow(sample_size,3))):
    			pygame.draw.circle(screen, BLUE, workspace_rep[i] , 3)


	#This is required to update the display
    pygame.display.flip()

pygame.quit()
