#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import dynamic_reconfigure.client

xthres = 0.25
ythres = 0.25
zthres = 0.25

def callback(data):
    client = dynamic_reconfigure.client.Client('cropbox')
    params= {'min_x': data.x - xthres,'max_x': data.x + xthres,'min_y': data.y - ythres,'max_y': data.y + ythres,'min_z': data.z - zthres,'max_z': data.z + zthres}
    config = client.update_configuration(params)         
      
    
    
def listener():

    rospy.init_node('cropbox_control', anonymous=True)

    rospy.Subscriber("obj_Asus", Point, callback)

   
    rospy.spin()

if __name__ == '__main__':
    listener()
