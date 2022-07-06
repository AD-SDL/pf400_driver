#!/usr/bin/env python3


# # ROS nodes
import rclpy
from rclpy.node import Node

import zmq
import time

def command_convert(job, robot_1, robot_2):
    if robot_1 and robot_2:
        return job + '@' + robot_1 + '@' + robot_2

    elif robot_1 != None and robot_2 == None:
        return job + '@' + robot_1

    else:
        return job


def pf400_transfer(job:str, target_1:str = None, target_2:str = None):

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.connect("tcp://127.0.0.1:8089")

    print("Starting PF400 command transfer client ...")
    while True:
        full_command = command_convert(job, target_1, target_2)
        print(full_command)
        #TODO: NEED A DELAY IF MULTIPLE COMMANDS WERE SENT. OTHERWISE ROBOT WILL SKIP PARTS
        sock.send_string(full_command)
        msg = sock.recv_string()
        print(msg)
        time.sleep(1)
        msg = msg.split('@')
        # msg_output, msg_error, msg_errorcode = msg[0], msg[1], msg[2]
        if msg:
            print("Client recived the output message from the completed protocol. Sending message to the ROS Arm node")
            return msg
    sock.close()
        
  
    

def main_null():
    print("This function is not meant to have a main function")

if __name__ == '__main__':
    pf400_transfer("transfer","plate_rack","bob")
    # pf400_transfer_command("transfer","bob","alex")
    # pf400_transfer_command("complete")

