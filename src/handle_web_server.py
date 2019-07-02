#!/usr/bin/env python
import sys
import rospy
from smart_movement.srv import *
from std_msgs.msg import String



class handle_web_speed_status():
    def __init__(self):

        self.flag = False
        rospy.init_node('web_status_speed_server')
        
        # subscribe to the server topic 
        rospy.Subscriber('/robot_status', String, self.callback)
        self.status_speed_pub = rospy.Publisher('/web_status_speed', String, queue_size=10)
        rospy.loginfo('Waiting for subscriber')
        rospy.loginfo(self.status_speed_pub.get_num_connections())
        while (self.status_speed_pub.get_num_connections() == 0 ):
            pass
        rospy.loginfo('waiting for the service to be established')
        s = rospy.Service('web_status_speed_srv', RequestStatus, self.web_speed_status)
        
        rospy.spin()
    
    def web_speed_status(self, req):
        # publish the service coming from the web_server to the multithreaded-server

        rospy.loginfo(req.statusreq)
        if req.statusreq[0:3] == "Sta":
            self.status_speed_pub.publish(req.statusreq)
            self.status_speed_pub.publish('Req:[1]')
        
        elif req.statusreq[0:3] =="Spe":
            self.status_speed_pub.publish(req.statusreq)
            self.status_speed_pub.publish('Req:[1]')
        
        elif req.statusreq[0:3] == "Req" : 
            self.status_speed_pub.publish(req.statusreq)
        
        elif req.statusreq[0:3] == "Las" :  
            self.status_speed_pub.publish(req.statusreq)
        
        elif req.statusreq[0:3] == "Cur" : 
            self.status_speed_pub.publish(req.statusreq)
        
        # elif req.statusreq[0:3] == "Joi":
        #     rospy.loginfo('waiting')
        #     rospy.wait_for_service('joint_status_srv')
        #     rospy.loginfo('Waiting for the joints_service')
        #     try:
        #         get_joint = rospy.ServiceProxy('joint_status_srv', RequestStatus)
        #         resp = get_joint('get_joint')
        #         return resp.statusrep
        #         self.recieved = resp.statusrep
        #         self.flag = True
        #     except rospy.ServiceException, e:
        #         print "Service call failed: %s"%e
            
        else :
            rospy.loginfo('Please send the correct command')
        
        rospy.loginfo("Waiting for the response")

        while self.flag == False:
            pass 

        rospy.loginfo("Web client updated") 
        self.flag = False
        return RequestStatusResponse(self.recieved)
        
        

        

        
    def callback(self, data):
        rospy.loginfo("Request from Web client") 
        self.recieved = data.data 
        rospy.loginfo(self.recieved)
        self.flag = True

        

if __name__ == '__main__':
    try:
        handle_web_speed_status()
    
    except rospy.ROSInitException:
        rospy.loginfo("Terminated")
        pass