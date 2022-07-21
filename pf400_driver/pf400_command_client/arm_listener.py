#!/usr/bin/env python3

import time

import zmq
import struct

from rpl_pf400 import RPL_PF400

def listener(host, port):
    
    try:
        ctx = zmq.Context()
        sock = ctx.socket(zmq.REP)
        sock.bind("tcp://"+host+":"+ port)
        print("Starting PF400 listener")
        # logger.info("Starting the command transfer listener")

        i = 1
        while True:
            msg = sock.recv_string()
            i += 1
            time.sleep(1)
            if msg != None:
                robot = RPL_PF400()
                msg_output = robot.command_handler(msg)
                sock.send_string(str(msg_output))
                # sock.send_string(msg_output + '@' + msg_error + '@' + str(msg_returncode))
    
    except struct.error as e:
        # self.logger.error('Lost connection from:', sock)
        # sockClient.shutdown(socket.SHUT_RDWR)
        sock.close()

    except KeyboardInterrupt:
        # self.logger.warn('Shutting down socket')
        # sockClient.shutdown(socket.SHUT_RDWR)
        sock.close()
        exit()
        
def main_null():
    print("This function is not meant to have a main function")

if __name__ == "__main__":
    listener("*", "8089")
