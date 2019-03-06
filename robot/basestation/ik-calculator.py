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

#ANGLES (IN RADIANS):
proximal_max_angle = math.pi 
proximal_min_angle = -math.pi
distal_max_angle = math.pi
distal_min_angle = -math.pi
wrist_max_angle = math.pi
wrist_min_angle = -math.pi

#INITIALIZE GLOBAL VARIABLES
computed_proximal_angle = proximal_min_angle
computed_distal_angle = distal_min_angle
computed_wrist_angle = wrist_min_angle

base_start_point = [0,0]
base_end_point = [0,base_length]
proximal_start_point = base_end_point
proximal_end_point = proximal_start_point + [proximal_length * math.cos(computed_proximal_angle),proximal_length * math.sin(computed_proximal_angle)]
distal_start_point = proximal_end_point
distal_end_point = distal_start_point + [distal_length * math.cos(computed_distal_angle),distal_length * math.sin(computed_distal_angle)]
wrist_start_point = distal_end_point
wrist_end_point = wrist_start_point + [wrist_length * math.cos(computed_wrist_angle),wrist_length * math.sin(computed_wrist_angle)]


###################PYGAME PARAMETERS###################
VOLUME_COL = [210,255,210]

###############PYGAME FUNCTIONS###############
def draw(self):
	pygame.draw.line(self.surf, self.color, base_start_point, base_end_point, self.width)
	pygame.draw.line(self.surf, self.color, proximal_start_point, proximal_end_point, self.width)
	pygame.draw.line(self.surf, self.color, distal_start_point, distal_end_point, self.wideth)
	pygame.draw.line(self.surf, self.color, wrist_start_point, wrist_end_point, self.width)


################PYGAME CLASSES################
class ProjectionView:
	def __init__(self, color, X, Y, width = 6):
		self.color = color
		self.line_width = width
		self.positioning = positioning



#######################FUNCTIONS#######################


def ComputeWorkspace():
	#INSERT SOME CLEVER CODE HERE
	return 1



def ComputeIK(X, Y, Z):
	beta = math.atan2(Y - base_length , X) #Calculates beta, which is the sum of the angles of all links
	Wrist_X = X - wrist_length * math.cos(beta) #Calculates wrist X coordinate
	Wrist_Y = (Y - base_length) - wrist_length * math.sin(beta) #Calculates wrist Y coordinate

	if math.sqrt(pow(Wrist_X,2) + pow(Wrist_Y,2)) > proximal_length + distal_length: #Check to see if point is too far
		print('Error! Cannot reach point!')
		return 0

	else:
		cosine_distal_angle = (pow(Wrist_X,2) + pow(Wrist_Y,2) - pow(proximal_length,2) - pow(distal_length,2))/(2 * proximal_length * distal_length) #Calculates the cosine of the angle of link 2
		computed_distal_angle = math.acos(cosine_distal_angle) #Proper angle is reached. NOTE: Depending on speed of function acos()
						#an approximation or different calculation method should be
						#used to speed computation time

	if computed_distal_angle > distal_max_angle or computed_distal_angle < distal_min_angle: #Check to see if third angle is within bounds
		disp('Error! Angle 3 is not within bounds')


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

	#Knowing the sum of all angles (beta) and two of the three angles,
	#the final angle (angle four) can be found:

	computed_wrist_angle = beta - (computed_proximal_angle + computed_distal_angle)

	return 1


def UpdateArmSetValues():
	base_start_point = [0,0]
	base_end_point = [0,base_length]
	proximal_start_point = base_end_point
	proximal_end_point = proximal_start_point + [proximal_length * math.cos(computed_proximal_angle),proximal_length * math.sin(computed_proximal_angle)]
	distal_start_point = proximal_end_point
	distal_end_point = distal_start_point + [distal_length * math.cos(computed_distal_angle),distal_length * math.sin(computed_distal_angle)]
	wrist_start_point = distal_end_point
	wrist_end_point = wrist_start_point + [wrist_length * math.cos(computed_wrist_angle),wrist_length * math.sin(computed_wrist_angle)]
	return 1
	
	
	
#########################BODY#########################
pygame.init()
pygame.font.init()
screen = pygame.display.set_mode((640,480))


Sideview = ProjectionView(OLUME_COL, 300, 500)
ComputeIK(0.1,0.2,0.3)
UpdateArmSetValues()
draw(Sideview)
