#!/usr/bin/env python
import rospy
from mr_voice.msg import Voice
from std_msgs.msg import String
def say(g):
    publisher_speaker.publish(g)
    rospy.sleep(1)

def callback_voice(msg):
    global s
    s = msg.text

def answer(text, data):
    for d in data:
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
    f = open(f_n,"r")
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

