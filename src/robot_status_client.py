#!/usr/bin/env python

import sys 
import rospy 
from smart_movement.srv import *



def request_robot_status(x):
    rospy.loginfo("Waiting for the server")
    rospy.wait_for_service('web_status_speed_srv')
    try:
        update_robot_status = rospy.ServiceProxy('web_status_speed_srv', RequestStatus)
        resps = update_robot_status(x)
        return resps.statusrep
    except rospy.ServiceException,e:
        print "Service call failed: %s"%e

           

if __name__ == '__main__':
    rospy.init_node('node')
    # req:[1] for status
    # speed:[1/2/3]
    # status:[0/1] 1 for brakes engadged 

    if len(sys.argv) == 2:
        x = str(sys.argv[1])
        rospy.loginfo(x)
        print (request_robot_status(x))
    else : 
        print 'failed'
        sys.exit()


    