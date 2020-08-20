#!/usr/bin/env python3

from ik_calculator import *

asimov=Arm(4,length_array,minmax,[0,math.pi/2,0,0])
print( asimov.getCurrentAngles() )

position = asimov.computeFK()
print( asimov.computeFK() )

position[1]-=0.0001 #distance a bit from the extreme endpoint

angles = asimov.computeIK(position)
print( asimov.computeIK(position) )

print( asimov.computeFK(angles[0]) )
