############################################################
############SCRB 2019 INVERSE KINEMATICS PACKAGE############
############################################################

#NOTE: The first angle in the code (a1a) is the angle that the base makes
#with the horizontal. Since the base is immobile, this angle is assumed to
#be constant. In fact, the properties of the base are not taken into
#account in this code but are simply added to the final figure.
#For this reason, each angle has a higher coefficient than its
#corresponding link. E.g. link 1 corresponds to angle a2a.

#This code uses a closed form solution to determine the orientation of the
#joints of a planar RRR arm for its end effector to reach a given point.For
#more information about workings of the code, contact Maxim Kaller.

import math
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#MANIPULATOR PHYSICAL PARAMETERS

base_length = 0.1 #Length of base
proximal_length = 0.5 #length of proximal link
distal_length = 0.5 #length of distal link
wrist_length = 0.2 #length of wrist link (until tip of fingers)

amax2 = math.pi #amaxi = maximum angle link i + 1 can reach
amin2 = 0 #amini = minimum angle link i + 1 can reach
amax3 = math.pi
amin3 = -math.pi
amax4 = math.pi
amin = -math.pi


#DESIRED COORDINATE point
X = 1
Y = 2
Z = 3


beta = math.atan2(Y - base_length , X) #Calculates beta, which is the sum of the angles of all links
xn = X - wrist_length * math.cos(beta) #Calculates xn, which is the x coordinate of the wrist
yn = (Y - base_length) - wrist_length * math.sin(beta) #Calculates yn, which is the y coordinate of the wrist

if math.sqrt(pow(xn,2) + pow(yn,2)) > proximal_length + distal_length: #Check to see if point is too far
    print('Error! Cannot reach point!')

else:
    a3 = (pow(xn,2) + pow(yn,2) - pow(proximal_length,2) - pow(distal_length,2))/(2 * proximal_length * distal_length) #Calculates the cosine of the angle of link 2
    a3a = math.acos(a3) #Proper angle is reached. NOTE: Depending on speed of function acos()
                    #an approximation or different calculation method should be
                    #used to speed computation time

if a3a > amax3 or a3a < amin3: #Check to see if third angle is within bounds
    disp('Error! Angle 3 is not within bounds')


aphi = (pow(xn,2) + pow(yn,2) + pow(proximal_length,2) - pow(distal_length,2))/(2 * math.sqrt(pow(xn,2) + pow(yn,2)) * proximal_length) #Determine cos of phi
phi = math.acos(aphi) #Phi is the angle between the first link and the straight line
              #from the base to the end effector

#Depending on the configuration of the arm, phi and beta can be used to
#find the second angle

if a3a <= 0:
    a2a = beta + phi

if a3a > 0:
    a2a = beta - phi

#If a2a is too small, calculations will restart with a3a being negative
if a2a > amax2 or a2a < amin2:
    #disp(a2a)
    a3a = -a3a

if a3a < 0:
    a2a = beta + phi

if a3a > 0:
    a2a = beta - phi

#Check to see if second angle is within bounds
if a2a > amax2 or a2a < amin2:
    print('Error! Angle 2 is not within bounds')

#Knowing the sum of all angles (beta) and two of the three angles,
#the final angle (angle four) can be found:

a4a = beta - (a2a + a3a)

#The following line creates a circle around the specified point. This point
#is used to confirm that the end effector is well positioned:
#scatter(X,Y, 200, 'LineWidth' , 2.5)

#The following code generates a graphical representation of the arm:
#plot( [0 0] , [0 d1] , 'k' , 'LineWidth' , 75)
#plot( [0 l1 * cos(a2a)] , [ d1 d1 + l1 * sin(a2a)], 'k' , 'LineWidth' , 3)
#plot( [l1 * cos(a2a) l1 * cos(a2a) + l2 * cos(a2a + a3a)] , [d1 + l1 * sin(a2a) d1 + l1 * sin(a2a) + l2 * sin(a2a + a3a)], 'k', 'LineWidth' , 3)
#plot( [ l1 * cos(a2a) + l2 * cos(a2a + a3a) l1 * cos(a2a) + l2 * cos(a2a + a3a) + l3 * cos(a2a + a3a + a4a)] , [ d1 + l1 * sin(a2a) + l2 * sin(a2a + a3a) d1 + l1 * sin(a2a) + l2 * sin(a2a + a3a) + l3 * sin(a2a + a3a + a4a) ] , 'k' , 'LineWidth' , 3)

#axis equal #To avoid weird graphs. NOTE: X axis and Y axis may still be of different lengths.
        #This will make links seem smaller/bigger than they actually
        #are. Fear not. As long as the end effector touches the circle
        #it is all good.
