#!/usr/bin/env python3


import socket
import json
import rospy

import time
from time import sleep
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam
from waterlinked_a50_ros_driver.msg import DVLDR

from tf_quaternion.transformations import euler_from_quaternion as efq
from tf_quaternion.transformations import quaternion_from_euler as qfe

import select



theDVL = DVL()
beam0 = DVLBeam()
beam1 = DVLBeam()
beam2 = DVLBeam()
beam3 = DVLBeam()

DVLDeadReckoning = DVLDR()


class DVL_A50(object):
    #Constructor
    def __init__(self):
        
        self.ip_address='192.168.194.95'
        rospy.loginfo('IP_ADDRESS: %s' % self.ip_address)
        self.dvl_publisher_ = rospy.Publisher('/dvl/data', DVL, queue_size=1)
        self.dvl_publisher_pos = rospy.Publisher('/dvl/position', DVLDR,  queue_size=1)
        self.dvl_kimera = rospy.Publisher('/dvl/dvl_odom', Odometry, queue_size=1)
        timer_period = rospy.Duration(0.05)  # seconds -> 10Hz
        self.stamp = rospy.Time.now()
        self.data = None
        self.oldJson = ""
        self.current_altitude = 0.0
        self.old_altitude = 0.0
        self.dvl_odom = Odometry()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rospy.loginfo("Connecting...")
        self.connect()
        #self.timer = rospy.Timer(timer_period, self.timer_callback())
        #rospy.sleep(1)
        rate=rospy.Rate(10)
        while True:
            self.timer_callback()
            rate.sleep()
        
        



    #Destructor
    def __del__(self): 
        print("Exiting...")
        self.sock.close()

	#SOCKET
    def connect(self):
        count=0
        try:
            rospy.loginfo("Trying to connect")
            server_address = (self.ip_address, 16171)
            self.sock.settimeout(3)
            self.sock.connect(server_address)
            
            rospy.loginfo("Socket is connected")
        except socket.timeout as err:
            rospy.loginfo("No route to host, DVL might be booting? {}".format(err))
            sleep(1)
            count += 1
            if count <=6:
                self.sock.settimeout(3)
                self.connect()
            else:
                rospy.signal_shutdown("No connection after 5 tries")

    def getData(self):
        raw_data = ""
        data = ""

        while not '\n' in raw_data:
            try:
                rec = self.sock.recv(1) # Add timeout for that
                data = str(rec, 'utf-8')
                if len(rec) == 0:
                    rospy.loginfo("Socket closed by the DVL, reopening")
                    self.connect()
                    continue
                else:
                    raw_data = raw_data + data

            except socket.timeout as err:
                rospy.loginfo("Lost connection with the DVL, reinitiating the connection: {}".format(err))
                self.connect()
                continue
    

        #raw_data = self.oldJson + raw_data
        #self.oldJson = ""
        #raw_data = raw_data.split('\n')
        #self.oldJson = raw_data[1]
        #raw_data = raw_data[0]
        #self.get_logger().info("Data: {}".format(raw_data))
        return raw_data

    #ROS
    def timer_callback(self):
        self.stamp = rospy.Time.now()
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        

        raw_data = self.getData()
        data = json.loads(raw_data)

        self.publish_data(data)
	
    def publish_data(self, data):

        theDVL.header.stamp = self.stamp
        theDVL.header.frame_id = "dvl_link"
        dvl_odom = self.dvl_odom
        #dvl_odom.header=theDVL.header
        dvl_odom.header.frame_id = "map"
        dvl_odom.child_frame_id="dvl_link"

        if 'time' in data:
            #recieved a velocity dvl message
            theDVL.time = float(data["time"])
            theDVL.velocity.x = float(data["vx"])
            theDVL.velocity.y = float(data["vy"])
            theDVL.velocity.z = float(data["vz"])
            theDVL.fom = float(data["fom"])
            self.current_altitude = float(data["altitude"])
            theDVL.velocity_valid = data["velocity_valid"]
            
            #adds velocity to the most recent pose message
            dvl_odom.twist.twist.linear.x=theDVL.velocity.x
            dvl_odom.twist.twist.linear.y=theDVL.velocity.y
            dvl_odom.twist.twist.linear.z=theDVL.velocity.z
            
            dvl_odom.header.stamp=rospy.Time.now() #rospy.Time.from_sec(theDVL.time)
            
            if (self.current_altitude >= 0.0) and theDVL.velocity_valid:
                theDVL.altitude = self.current_altitude
                self.old_altitude = self.current_altitude
            else:
                theDVL.altitude = self.old_altitude


            theDVL.status = data["status"]
            theDVL.form = data["format"]
		
            beam0.id = data["transducers"][0]["id"]
            beam0.velocity = float(data["transducers"][0]["velocity"])
            beam0.distance = float(data["transducers"][0]["distance"])
            beam0.rssi = float(data["transducers"][0]["rssi"])
            beam0.nsd = float(data["transducers"][0]["nsd"])
            beam0.valid = data["transducers"][0]["beam_valid"]
		
            beam1.id = data["transducers"][1]["id"]
            beam1.velocity = float(data["transducers"][1]["velocity"])
            beam1.distance = float(data["transducers"][1]["distance"])
            beam1.rssi = float(data["transducers"][1]["rssi"])
            beam1.nsd = float(data["transducers"][1]["nsd"])
            beam1.valid = data["transducers"][1]["beam_valid"]
		
            beam2.id = data["transducers"][2]["id"]
            beam2.velocity = float(data["transducers"][2]["velocity"])
            beam2.distance = float(data["transducers"][2]["distance"])
            beam2.rssi = float(data["transducers"][2]["rssi"])
            beam2.nsd = float(data["transducers"][2]["nsd"])
            beam2.valid = data["transducers"][2]["beam_valid"]
		
            beam3.id = data["transducers"][3]["id"]
            beam3.velocity = float(data["transducers"][3]["velocity"])
            beam3.distance = float(data["transducers"][3]["distance"])
            beam3.rssi = float(data["transducers"][3]["rssi"])
            beam3.nsd = float(data["transducers"][3]["nsd"])
            beam3.valid = data["transducers"][3]["beam_valid"]
		
            theDVL.beams = [beam0, beam1, beam2, beam3]
		
            #publish the messages
            self.dvl_publisher_.publish(theDVL)
            self.dvl_kimera.publish(dvl_odom)
            
        if 'ts' in data:
            #received a position message
            DVLDeadReckoning.time = float(data["ts"])
            DVLDeadReckoning.position.x = float(data["x"])
            DVLDeadReckoning.position.y = float(data["y"])
            DVLDeadReckoning.position.z = float(data["z"])
            DVLDeadReckoning.pos_std = float(data["std"])
            DVLDeadReckoning.roll = float(data["roll"])
            DVLDeadReckoning.pitch = float(data["pitch"])
            DVLDeadReckoning.yaw = float(data["yaw"])
            DVLDeadReckoning.type = data["type"]
            DVLDeadReckoning.status = data["status"]
            DVLDeadReckoning.format = data["format"]
            
            #change stored value for the position rather than publishing
            self.dvl_odom.pose.pose.position.x=DVLDeadReckoning.position.x
            self.dvl_odom.pose.pose.position.y=DVLDeadReckoning.position.y
            self.dvl_odom.pose.pose.position.z=DVLDeadReckoning.position.z
            
            [x,y,z,w]=qfe(DVLDeadReckoning.roll,DVLDeadReckoning.pitch,DVLDeadReckoning.yaw)
            
            self.dvl_odom.pose.pose.orientation.x=x
            self.dvl_odom.pose.pose.orientation.y=y
            self.dvl_odom.pose.pose.orientation.z=z
            self.dvl_odom.pose.pose.orientation.w=w
            
            self.dvl_publisher_pos.publish(DVLDeadReckoning)
        


def main(args=None):

    rospy.init_node('dvl_a50_node')

    try:
        node = DVL_A50()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('DVLNode::Exception')
    print('Leaving DVLNode') 
    


if __name__ == '__main__':
    main()
