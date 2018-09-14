#!/usr/bin/env python3
# -*- coding: utf-8 -*-+

import roslib
roslib.load_manifest('ddpg')
import rospy

from lib.strategy import Strategy

def main():
    rospy.init_node('ddpg_strategy', anonymous=True)
    robot = Strategy()

    # 30 hz
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if(robot._param.start):
            robot.Process()
        else:
            print("Don't start up Strategy")    
        rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
