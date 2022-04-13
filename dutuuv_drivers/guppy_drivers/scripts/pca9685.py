#!/usr/bin/python3

import rospy
import numpy as np
from guppy_drivers.pca9685 import PCA9685
from guppy_msgs.msg import PwmStamped

def pwm_callback(pwm_msg, pwm):

    pwm.setServoPulse_propeller(0,np.clip(pwm_msg.pwm.horizontal_port,1350,1650))
    pwm.setServoPulse_propeller(1,np.clip(pwm_msg.pwm.horizontal_stbd,1350,1650))
    pwm.setServoPulse_propeller(2,np.clip(pwm_msg.pwm.vertical_port,1350,1650))
    pwm.setServoPulse_propeller(3,np.clip(pwm_msg.pwm.vertical_stbd,1350,1650))


if __name__ == '__main__':
    rospy.init_node('pca9685')
    pwm = PCA9685(0x40, debug=False)
    pwm.setPWMFreq(50)
    rospy.Subscriber('/command/pwm', PwmStamped, callback=pwm_callback, callback_args=pwm, queue_size=1)
    rospy.spin()



