#!/usr/bin/env python
# Copyright 2017 HyphaROS Workshop.
# Developer: HaoChih, LIN (hypha.ros@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import string
import math
import time
import sys

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped

class MultiGoals:
    def __init__(self, goalListX, goalListY, retry, map_frame):
        self.sub = rospy.Subscriber('/tianracer/move_base/result', MoveBaseActionResult, self.statusCB, queue_size=10)
        self.pub = rospy.Publisher('/tianracer/move_base_simple/goal', PoseStamped, queue_size=10)   
        # params & variables
        self.goalListX = goalListX
        self.goalListY = goalListY
        self.goalListZ = goalListZ
        self.goalListW = goalListW
        self.retry = retry
        self.goalId = 0
        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = map_frame
        # Publish the first goal
        time.sleep(0.3)
        self.goalMsg.header.stamp = rospy.Time.now()
        self.goalMsg.pose.position.x = self.goalListX[self.goalId]
        self.goalMsg.pose.position.y = self.goalListY[self.goalId]
        self.goalMsg.pose.orientation.z =self.goalListZ[self.goalId]
        self.goalMsg.pose.orientation.w =self.goalListW[self.goalId]
        self.pub.publish(self.goalMsg) 
        rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId) 
        self.goalId = self.goalId + 1 

    def statusCB(self, data):
        if data.status.status == 3: # reached
            self.goalMsg.header.stamp = rospy.Time.now()                
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z =self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w =self.goalListW[self.goalId]
            self.pub.publish(self.goalMsg)  
            rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId)              
            if self.goalId < (len(self.goalListX)-1):
                self.goalId = self.goalId + 1

                


if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('multi_goals', anonymous=True)
        

        # Get params                                         1                                2                          3                         4                          5                         6                            7                           8                          9                            10                       11                        12
        goalListX = rospy.get_param('~goalListX', '[-2.6430845260620117           ,  -2.9835569858551025    ,  -0.10187184810638428  ,   -2.6430845260620117      ,  -2.9835569858551025      ,  -0.10187184810638428  ,   -2.6430845260620117      ,  -2.9835569858551025      ,  -0.10187184810638428          ,   0      ,    -1.2615047693252563     ,          0]')
        goalListY = rospy.get_param('~goalListY', '[-0.011949002742767334         ,  -7.167231559753418     ,   2.4401564598083496   ,    -0.011949002742767334   ,  -7.167231559753418       ,   2.4401564598083496   ,    -0.011949002742767334   ,  -7.167231559753418       ,   2.4401564598083496           ,   0      ,    -5.644099235534668      ,          0]')
        goalListW = rospy.get_param('~goalListW', '[0.9993973267                  ,   0.720860362104342     ,   0.7118367911284649   ,     0.9993973267821008     ,   0.720860362104342       ,   0.7118367911284649   ,    0.9993973267821008      , 0.720860362104342         ,   0.7118367911284649           ,   0      ,    0.7118367492654443      ,    0.7036231908541771]')
        goalListZ = rospy.get_param('~goalListZ', '[-0.01718562240495513          ,  -0.6930803260422249    ,   0.702344917256422    ,    -0.01718562240495513    , -0.6930803260422249       ,   0.702344917256422    ,   -0.01718562240495513     ,  -0.6930803260422249      ,   0.702344917256422            ,   0      ,     0.7131103775852133     , 0.7105732933992005]')
        map_frame = rospy.get_param('~map_frame', 'map' )
        retry = rospy.get_param('~retry', '1') 

        goalListX = goalListX.replace("[","").replace("]","")
        goalListY = goalListY.replace("[","").replace("]","")
        goalListZ = goalListZ.replace("[","").replace("]","")
        goalListW = goalListW.replace("[","").replace("]","")
        goalListX = [float(x) for x in goalListX.split(",")]
        goalListY = [float(y) for y in goalListY.split(",")]
        goalListZ = [float(z) for z in goalListZ.split(",")]
        goalListW = [float(w) for w in goalListW.split(",")]

        if (len(goalListX) == len(goalListY) & len(goalListY) >=2):          
            # Constract MultiGoals Obj
            rospy.loginfo("Multi Goals Executing...")
            mg = MultiGoals(goalListX, goalListY, retry, map_frame)      
            rospy.spin()
        else:
            rospy.errinfo("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")

