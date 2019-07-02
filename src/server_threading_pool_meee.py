#!/usr/bin/python

import rospy
import Queue 
import socket 
import struct
import binascii
import sys
from threading import Thread 
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from smart_movement.msg import codesys_joint 
from std_msgs.msg import String
from math import pi
import time 
from smart_movement.srv import *

# Child class to create the instance of a thread_client and assign a task to it 

class ClientThread(Thread):

	def run(self):

		while True:
			print 'inside the thread'
			#check if the client data is stored in queue
			self.client = client_queue.get()

			if self.client is not None:
				print 'Connection made', self.client[1]
				self.recieved = self.client[0].recv(1024)
				
				
				# The client wants to recieve the waypoints, the status, the speed recieved from the codesys_waypoints 
				if self.recieved[0:4] == "read":
					
					rospy.loginfo("Recieved read")
					self.send_speed = True 
					self.flag_callback = True
					# Send webserver commands to Codesys
					self.sub_web_status = rospy.Subscriber('/web_status_speed', String, self.web_status_speed_handle)
					# send waypoint to Codesys
					self.sub = rospy.Subscriber('/codesys_waypoints', codesys_joint, self.callback)
					# publish the info to Webserver handler
					self.done_pub = rospy.Publisher('/goal_reached', String, queue_size=10)
					
					

				# The clients wants to send the enocder data to the fake_joint_controller 	
						
				if self.recieved[0:5] == "write":
					
					rospy.loginfo("Recieved write")
					self.pub_encoder = rospy.Publisher('/codesys_encoder', JointState, queue_size=20)
					while True:
						self.recieved_encoder_motor()
				
				# The clients wants to send the status of the SmartWrist 

				if self.recieved[0:4] == "info":
					rospy.loginfo("Recieved info")
					self.status_pub = rospy.Publisher('/robot_status', String, queue_size=10)
					# create service object server to update the webserver of the robot status 
					while True:
						self.handle_info_client()
					    

										

				print 'my name is ',self.getName()
				print 'Communication Closed', self.client[1]
			
	
	def web_status_speed_handle(self, data):

		rospy.loginfo(data.data)
		self.client[0].sendall(data.data)
		rospy.sleep(0.01)


	
	def callback(self, message):
		 
			
		scale = 2
		waypoint = 'ns:(' + str(int((message.waypoints-1)/scale)) + ')'
		self.client[0].sendall(waypoint)
		rospy.loginfo(waypoint)
		counter = 0	
		for wp_nr in range(1, message.waypoints, scale):
			if wp_nr + scale > message.waypoints-1:
				wp_nr = message.waypoints-1
			waypoint = 'w' + str(counter)+ ':[' 
			waypoint += str(message.theta0[wp_nr]) + ',' + str(message.theta1[wp_nr]) + ','
			waypoint += str(message.theta2[wp_nr]) + ',' + str(message.theta3[wp_nr]) + ']'
			rospy.loginfo(waypoint)
			self.client[0].sendall(waypoint)
			counter += 1
			rospy.loginfo("next")
			rospy.sleep(0.05)
		self.flag = True
		
		rospy.loginfo("ok")
		self.update_the_position()
		
		
	def	update_the_position(self):
		while self.flag == True:
			rospy.loginfo("waiting for done")
			message = self.client[0].recv(1024)
			rospy.loginfo(message)
			if (message[0:4] == "Done"):
			# 	while (self.done_pub.get_num_connections() == 0):
			# 		pass
			# 	self.done_pub.publish("update")
				rospy.loginfo("updated")
				self.flag = False
				self.flag_callback = True


	def recieved_encoder_motor(self):

		
		while not rospy.is_shutdown():
			data = self.client[0].recv(1024)
			rospy.loginfo(data)
			if data is not None:
				start = data.find ('[')
				comma1 = data.find (',')
				comma2 = data.find (',', comma1+1)
				comma3 = data.find (',', comma2+1)
				end = data.find(']')
				current_pos = JointState()
				current_pos.header.seq = 0
				current_pos.header.stamp = rospy.Time.now()
				current_pos.header.frame_id = "/world"
				current_pos.name = ["end_link" , "joint0", "joint1", "joint3", "joint4"]
				current_pos.position = [0.0 , int(data[start+1:comma1]) * pi/3140,(int(data[comma1+1:comma2]) * pi / 3140) + (int(data[start+1:comma1]) * pi/3140), int(data[comma2+1:comma3]) * pi / 3140  , (int(data[comma3+1:end]) * pi / 3140) + (int(data[comma2+1:comma3]) * pi / 3140)]
				
	
				self.pub_encoder.publish(current_pos)

			else:
				break



	def handle_info_client(self):
		while not rospy.is_shutdown():
			recieved = self.client[0].recv(1024)
			self.status_pub.publish(self.client[0].recv(1024))
			

        
		

		

		


# Define Server socket 	
HOST = '169.254.141.118'
# HOST = '127.0.0.1'
PORT = 65431

# Create a queu to store the client data 
client_queue = Queue.Queue(0)

#Create three threads through 
for x in range (4): 
	ClientThread().start()

# Create the server object 
server_thread = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_thread.bind((HOST, PORT))
server_thread.listen(5)

#
rospy.loginfo((HOST,PORT))

if __name__ == '__main__' : 
	#create a node 
	rospy.init_node('server_thread_test', anonymous=True)

	while True:

		try: 
			client_queue.put(server_thread.accept())

			

		except rospy.ROSException:
			server_thread.close()




