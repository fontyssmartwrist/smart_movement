#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class codesys_dynamix():

    def __init__(self):
# // /move_group/fake_controller_joint_state
        self.flag = 0 
        rospy.Subscriber('/codesys_encoder', JointState, self.callback_codesys)
        rospy.Subscriber('/dynamix_encoder', Float64, self.callback_dynamix)
        self.moveit_pub = rospy.Publisher('/move_group/fake_controller_joint_state', JointState, queue_size=20)
        rospy.spin()


    def callback_codesys(self, codesys_data):
        rospy.loginfo('Codesys data recieved')
        self.temp = codesys_data
        rospy.loginfo(self.temp)
        self.flag = 1 
    
    def callback_dynamix(self, dynamix_data ):

        rospy.loginfo("recieved")
        #   codesys encoder data recieved
        if self.flag == 1 :
            rospy.loginfo('dynamix data recieved')
            current_pos = JointState()
            current_pos.header.seq = 0
            current_pos.header.stamp = rospy.Time.now()
            current_pos.header.frame_id = "/world"
            current_pos.name = ["end_link" , "joint0", "joint1", "joint3", "joint4"]
            current_pos.position = [dynamix_data.data, self.temp.position[1],self.temp.position[2],self.temp.position[3],self.temp.position[4]]
            rospy.loginfo(current_pos)
            self.moveit_pub.publish(current_pos)
            self.flag = 0 
        else:
        
            rospy.loginfo('waiting for codesys data')


            




if __name__ == '__main__':
    rospy.init_node('codesys_dynamix_listener', anonymous=True)


    while True:
        try:
            codesys_dynamix()
        except rospy.ROSInitException:
            rospy.loginfo("Terminated")
            pass 