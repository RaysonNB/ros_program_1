#!/usr/bin/env python
import rospy
from mr_voice.msg import Voice
from std_msgs.msg import String
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest
import time
def say(g):
    publisher_speaker.publish(g)
    rospy.sleep(1)
def callback_voice(msg):
    global s
    s = msg.text
def answer(text, aaa):
    for d in aaa:
        res_AND = True
        for keys in d["K"]:
            res_OR = False
            for key in keys:
                if key[0] == "!":
                    res_OR = res_OR or key[1:] not in text
                else:
                    res_OR = res_OR or key in text
            res_AND = res_OR and res_AND
        if res_AND:
            return d["A"]
    return ""


def load_data(f_n):
    data = []
    f = open(f_n, "r")
    lines = [x.strip().lower() for x in f.readlines()]
    status = "N"
    for i in range(len(lines)):
        if len(lines[i]) == 0:
            data.append({"Q": "", "A": "", "K": []})
            status = "Q"
        elif status == "Q":
            data[-1]["Q"] = lines[i]
            status = "A"
        elif status == "A":
            data[-1]["A"] = lines[i]
            status = "K"
        elif status == "K":
            keys = [x.strip() for x in lines[i].split(",")]
            data[-1]["K"].append(keys)
    f.close()
    return data
def set_gripper(angle, t):
    service_name = "/goal_tool_control"
    rospy.wait_for_service(service_name)
    
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
    return set_gripper(0.01, t)

if __name__ == "__main__":
    rospy.init_node("speak_node")
    rospy.loginfo("speak_node started!")
    data = load_data("wh.txt")
    t=1.0
    aaaa=""
    rospy.Subscriber("/voice/text", Voice, callback_voice)
    publisher_speaker = rospy.Publisher("/speaker/say", String, queue_size=10)
    s=""
    xyz_init = (0.288,0.0,0.194)
    xyz_home = (0.134,0.0,0.240)
    open_gripper(1.0)
    move_to(xyz_home[0], xyz_home[1], xyz_home[2], 3.0)
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        if len(s)== 0:
            continue
        else:
            aaaa=answer(s, data)
            if "*" in aaaa:
                if "open" in s:
                    open_gripper(1.0)
                    say("open")
                elif "close" in s:
                    close_gripper(1.0)
                    say("close")
                elif "help" in s:
                    move_to(xyz_init[0], xyz_init[1], xyz_init[2], 3.0)
                elif "OK" in s:
                    move_to(xyz_home[0], xyz_home[1], xyz_home[2], 3.0)
                else:
                    say("I don't know how to do the action")
            elif aaaa != "":
                say(aaaa)
                rospy.loginfo(s)
            else:
                say("please say it again")
            s = ""
            aaaa=""
                
