#!/usr/bin/env python3
import rclpy
import sys, signal, os
from rclpy.node import Node
from std_msgs.msg import Int32
from getkey import getkey

MAX_VEL = 1000
VEL_STEP_SIZE = 10

msg = """
-------------------------------------------------
CCW : [ CW : ]   

CTRL-C to quit
-------------------------------------------------
"""

class TeleopKey(Node):
    def __init__(self):
        super().__init__("md_teleop_key_node")

        qos_profile = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)

        self.cmd_rpm_pub = self.create_publisher(Int32, "/cmd_rpm", qos_profile)
        signal.signal(signal.SIGINT, self.signal_handler) #callback if ctrl+C signal is input.

        target_rpm = 0
        os.system('clear')
        print(msg)

        while(1):
            key = getkey()
            if key == "[":
                target_rpm = self.CheckLRPMLimit(target_rpm - VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("target_RPM = ", target_rpm)
            elif key == "]":
                target_rpm = self.CheckLRPMLimit(target_rpm + VEL_STEP_SIZE)
                os.system('clear')
                print(msg)
                print("target_RPM = ", target_rpm)
            
            rpm = Int32()
            rpm.data = target_rpm
            self.cmd_rpm_pub.publish(rpm)
    

    def signal_handler(self, sig, frame):
        rpm = Int32()
        rpm.data = 0
        print("target_RPM = ", rpm.data)

        self.cmd_rpm_pub.publish(rpm)
            
        sys.exit(0)

    def CheckLRPMLimit(self, vel):
        if vel <= -MAX_VEL:
            vel = -MAX_VEL
        elif vel >= MAX_VEL:
            vel = MAX_VEL
        
        return vel

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()