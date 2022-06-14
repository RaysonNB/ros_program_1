#!/usr/bin/env python
import rospy
from mr_voice.msg import Voice
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
#def move_which(x,y,z):
 #   while abs(w) < 0.9:
  #      rospy.Rate(20).sleep()
   #     msg_cmd_vel.linear.x = x    
    #    msg_cmd_vel.angular.z = z
     #   msg_cmd_vel.linear.y =   
      #  pub_cmd_vel.publish(msg_cmd_vel)

def callback_yolo(msg):
    global boxes
    boxes = msg.bounding_boxes
def callback_voice(msg):
    global s
    s = msg.text
def say(g):
    publisher_speaker.publish(g)

if __name__ == "__main__":
    rospy.init_node("voice_move")
    rospy.loginfo("start")
    s = ""
    rospy.Subscriber("/voice/text", Voice, callback_voice)

    publisher_speaker = rospy.Publisher("/speaker/say", String, queue_size=10)
    pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        if s == "": 
            rospy.loginfo("nothing")
            continue
        else:
            say("I am ready")
            msg_cmd_vel = Twist()
            if "front" in s or "French" in s or "Front" in s:
                say("delay")
                msg_cmd_vel.linear.x = 0.5 
            elif "back" in s or "black" in s:
                say("I going to back")
                msg_cmd_vel.linear.x = -0.5
                pub_cmd_vel.publish(msg_cmd_vel)

        s = ""
