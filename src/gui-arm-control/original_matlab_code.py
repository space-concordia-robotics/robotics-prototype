###########################################################
#INVERSE KINEMATICS FOR SPACE CONCORDIA ROBOTICS TEAM 2017%
###########################################################

#This code uses the Monte Carlo method to find the planar workspace of a 3R
#arm. This configuration is similar to the one the team seeks to use for
#its own arm. The code basically generates a large cloud of points
#reachable by the arm to get an idea of the shape of the workspace
# used matplotlib for scatter and plot https://matplotlib.org/tutorials/introductory/pyplot.html
# matplotlib also uses matlab syntax https://www.mathworks.com/help/matlab/ref/plot.html
import random
from math import sin, cos, pi
import matplotlib.pyplot as plt
##### INITIALIZATION OF VARIABLES #####

d1 = 0.5 #Length of base (could include rover length)
l1 = 1 #li = length of link i
l2 = 1
l3 = 0.25
amax = (4 * pi) / 6 #amaxi = maximum angle link i can reach
amin = pi / 6 #amini = minimum angle link i can reach
amax2 = pi / 2
amin2 = -(4 * pi / 6)
amax3 = pi / 2
amin3 = -2 * pi / 3
#aai is the angle that will be displayed for link i.
#For now it is set to be the average of the angle limits.
aa = (amax + amin) / 2
aa2 = (amax2 + amin2) / 2
aa3 = (amax3 + amin3) / 2
adif = amax - amin#adifi is the span of angles of joint i
adif2 = amax2 - amin2
adif3 = amax3 - amin3

##### BULK OF THE CODE #####

# Simple command to make sure graphs are not erased upon the creation of new ones
# hold() is not defined in python, need to find a workaround
#hold(mstring('on'))

#MAIN MONTE CARLO LOOP:
#This loop generates a very large amount of points in the arm's workspace.
#The more points generated, the more visible the workspace.
#reduced point scatter from 666666 to 60000, any more was crashing the system
numberOfScatterPoints = 60000
x = [0] * numberOfScatterPoints
y = [0] * numberOfScatterPoints
for i in range(0,numberOfScatterPoints):
    #The following lines finds a random angle for joints within its span
    t2 = random.uniform(0,adif) + amin
    t3 = random.uniform(0,adif2) + amin2
    t4 = random.uniform(0,adif3) + amin3
    #The following 2 lines compute the x and y coordinates corresponding to the angles generated:
    x[i] = l1 * cos(t2) + l2 * cos(t2 + t3) + l3 * cos(t2 + t3 + t4)
    y[i] = d1 + l1 * sin(t2) + l2 * sin(t2 + t3) + l3 * sin(t2 + t3 + t4)


#The following lines add black circles where the arm joints should be:
joint1x = 0
joint1y = d1
joint2x = l1*cos(aa)
joint2y = d1+l1*sin(aa)
joint3x = l1*cos(aa)+l2*cos(aa+aa2)
joint3y = d1+l1*sin(aa)+l2*sin(aa+aa2)
plt.plot( joint1x , joint1y , 'ko' , markersize=7)
plt.plot( joint2x , joint2y , 'ko' , markersize=7)
plt.plot( joint3x , joint3y , 'ko' , markersize=7)

# This line is used to graph all the points found in the above loop:
#plt.plot(ax,ay,'b.', markersize=2) #alternate function to scatter, figure out which is faster, or not
plt.scatter(x,y,color='b',s=8,alpha=0.5)

# Coordinates for each arams portions arm#x and arm#y (represented as lines)
# arm1 is the base itsel, arm2 is closest to the base
# arm4 is the furthest reach of the arm
# pairs of coordinates are in the form
# armX [start coordinate for X,end coordinate for X] , armY [start coordinate for Y,end coordinate for Y]
arm1x = [0 , 0];
arm1y = [0 , d1];
arm2x = [0 , l1*cos(aa)];
arm2y = [d1 , d1+l1*sin(aa)];
arm3x = [l1*cos(aa) , l1*cos(aa)+l2*cos(aa+aa2)]
arm3y = [d1+l1*sin(aa) , d1+l1*sin(aa)+l2*sin(aa+aa2)]
arm4x = [l1*cos(aa)+l2*cos(aa+aa2) , l1*cos(aa)+l2*cos(aa+aa2)+l3*cos(aa+aa2+aa3)]
arm4y = [d1+l1*sin(aa)+l2*sin(aa+aa2) , d1+l1*sin(aa)+l2*sin(aa+aa2)+l3*sin(aa+aa2+aa3)]

#The following lines plot a figure representing the arm's configuration:
plt.plot( arm1x , arm1y ,'k' , linewidth=10)
plt.plot( arm2x , arm2y , 'k' , linewidth=3)
plt.plot( arm3x , arm3y , 'k' , linewidth= 3);
plt.plot( arm4x , arm4y , 'k' , linewidth=3);
plt.show()
