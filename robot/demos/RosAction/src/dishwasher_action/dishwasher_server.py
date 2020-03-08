#! /usr/bin/env python

import rospy
import actionlib
import dishwasher_action.msg

class DishwasherAction(object):
    # create messages that are used to publish feedback/result
    _feedback = dishwasher_action.msg.DishwasherFeedback()
    _result = dishwasher_action.msg.DishwasherResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, dishwasher_action.msg.DishwasherAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(2)
        success = True
	clean_dishes = 0
        
        # append the seeds for the Dishwasher sequence
        self._feedback.percentComplete = 0.0
        
        # publish info to the console for the user
        rospy.loginfo('%s: Mom is washing %i of your dirty dishes.' % (self._action_name, goal.dirtyDishes))
        
        # start executing the action
        for i in range(0, goal.dirtyDishes+1):
		clean_dishes += 1
		
            	# check that preempt has not been requested by the client
            	if self._as.is_preempt_requested():
               	 	rospy.loginfo('%s: Preempted' % self._action_name)
                	self._as.set_preempted()
                	success = False
                	break
            	if clean_dishes%10 == 0:
			#print("Number of clean dishes: " + str(clean_dishes))
			#self._feedback.percentComplete = 10.0
			self._feedback.percentComplete = (float(clean_dishes)/float(goal.dirtyDishes))*100.0

			#print("Percent of dirty dishes washed: " + str(self._feedback.percentComplete) + "%")


            		# publish the feedback
            		self._as.publish_feedback(self._feedback)
            	# this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            	r.sleep()
          
        if success:
            self._result.dishesWashed = "You should be ashamed with yourself. The dishes are clean!"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('dishwasher')
    server = DishwasherAction(rospy.get_name())
    rospy.spin()
