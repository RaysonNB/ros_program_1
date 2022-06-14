#!/usr/bin/env python
import rospy
from mr_voice.msg import Voice
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
import time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def callback_imu(msg):
    global yaw
    x = msg.orientation.x
    y = msg.orientation.y
    z = msg.orientation.z
    w = msg.orientation.w
    q = [x, y, z, w]
    roll, pitch, yaw = euler_from_quaternion(q)

def callback_voice(msg):
    global s , d
    s = msg.text
    d = msg.direction
    rospy.loginfo(s)

def move_to(x,y,z,t):
    service_name = "/goal_task_space_path_position_only"
    rospy.wait_for_service(service_name)

    try:
        service = rospy.ServiceProxy(service_name, SetKinematicsPose)

        request = SetKinematicsPoseRequest()
        request.end_effector_name = "gripper"
        request.kinematics_pose.pose.position.x = x
        request.kinematics_pose.pose.position.y = y
        request.kinematics_pose.pose.position.z = z
        request.path_time = t

        response = service(request)
        return response
    except Exception as e:
        rospy.loginfo("%s" % e)
        return False

def set_gripper(angle, t):
    service_name = "/goal_tool_control"
    try:
        service = rospy.ServiceProxy(service_name, SetJointPosition)

        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["gripper"]
        request.joint_position.position = [angle]
        request.path_time = t

        response = service(request)
        return response
    except Exception as e:
        rospy.loginfo("%s" % e)
        return False

def open_gripper(t):
    return set_gripper(0.01,t)
def close_gripper(t):
    return set_gripper(-0.01,t)
def set_joints(joint1,joint2,joint3,joint4,tt):
    service_name = "/goal_joint_space_path"
    rospy.wait_for_service(service_name)

    try:
        service = rospy.ServiceProxy(service_name, SetJointPosition)

        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        request.joint_position.position = [joint1,joint2,joint3,joint4]
        request.path_time = tt

        response = service(request)
        return response
    except Exception as e:
        rospy.loginfo("%s" %e)
        return False

if __name__ == "__main__":
    rospy.init_node("vioce_turn")
    rospy.loginfo("start!")
    yaw = 0
    d = 0
    s = ""
    y = 0
    tt = 1.0
    frame= None
    boxes = None
    topic_name="/camera/rgb/image_raw"
    rospy.Subscriber(topic_name, Image, callback_Image)
    rospy.wait_for_message(topic_name,Image)
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_Image)
    rospy.Subscriber("/imu/data",Imu,callback_imu)
    rospy.Subscriber("/voice/text", Voice,callback_voice)
    pub_cmd_vel = rospy.Publisher("/cmd_vel",Twist,queue_size = 10)
    pub_speaker = rospy.Publisher("/speaker/say", String,queue_size = 10)
    
    rospy.sleep(1)
    pub_speaker.publish("I am ready")
    
    msg_cmd_vel = Twist()
    pub_cmd_vel.publish(msg_cmd_vel

    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        #rospy.loginfo(s)
        if len(s) == 0:
            continue
        else:
            while True:
                s= ""
                rospy.loginfo("I am ready")
                if "get" in s:
                    pub_speaker.publish("I am going to get the bottle")
                    xyz_init = (0.287, 0.0, 0.196)
                    xyz_home = (0.134, 0.0, 0.240)
                    time.sleep(tt)

                    rospy.loginfo("Goto home pose!")
                    move_to(xyz_home[0], xyz_home[1], xyz_home[2],time)
                    time.sleep(tt)

                    rospy.loginfo("Goto init pose!")
                    move_to(xyz_init[0],xyz_init[1],xyz_init[2],tt)
                    time.sleep(tt)

                    set_gripper(0.0,tt)
                    time.sleep(tt)

                    open_gripper(tt)
                    time.sleep(tt)

                    close_gripper(tt)
                    time.sleep(tt)

                    rospy.loginfo("Goto home pose!")
                    move_to(xyz_home[0], xyz_home[1], xyz_home[2],tt)
                    time.sleep(tt)
                    pub_speaker.publish("I am finish")
                    break
                else:
                    s=""

