#!/home/smurolap/.conda/envs/imitation_env/bin/python3
# the first line is very important, it tells which python is to be used
# since terminal cannot find python interpreter after you source devel/setup.zsh
"""
 * File Name : messagePublisher.py
 * Author    : ZEFFIRETTI, HESH
 * College   : Beijing Institute of Technology
 * E-Mail    : zeffiretti@bit.edu.cn, hiesh@mail.com
"""

import rospy
import torch
from std_msgs.msg import String
from genmsg import EXT_MSG

pub = rospy.Publisher('topic_name', String, queue_size=10)
rospy.init_node('pub_node', anonymous=True)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish('hello world')
    rate.sleep()
