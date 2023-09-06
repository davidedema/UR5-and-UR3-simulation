import rospy
from std_msgs.msg import Float64MultiArray

import trajectory as traj
import numpy as np
import kinematics as kin
import kine as kine

LOOP_RATE = 1000

JOINT_TOPIC = '/robot1/joint_group_position_controller/command'

GRIPPER_OPEN = 0.5
GRIPPER_CLOSE = -0.25


class Publisher:
    def __init__(self):
        self.pubRobot = rospy.Publisher(JOINT_TOPIC, Float64MultiArray, queue_size=10)
        self.q0 = np.array([ 0, 0, 2.78, -2.94, 2.45, -9.18])
        self.rate = rospy.Rate(LOOP_RATE)
        self.gripper = [GRIPPER_OPEN, GRIPPER_OPEN]

    def publish_point(self, q):
        msg = Float64MultiArray()
        msg.data = q
        msg.data.extend(self.gripper)
        self.pubRobot.publish(msg)
        self.rate.sleep()
    
    def move(self, pos, orientation):
        rotm = kin.eul2Rot(orientation)
        T = np.identity(4)
        T[:3,:3] = rotm
        T[:3,3] = pos
        possibleQ = kine.invKine(T)
        possibleQT = np.transpose(possibleQ)
        qf = self.findClosestQ(possibleQT)
        qi, qdi, qddi = traj.cubic_trajectory_planning(self.q0, qf, np.zeros(6), np.zeros(6))
        for i in range(len(qi[0,:])):
            self.publish_point(qi[:,i].flatten().tolist())
        self.q0 = qf
        print(qf)

    def eucledianDistance(self, q1, q2):
        q2f = np.zeros(6)
        for i in range(6):
            q2f[i] = q2[0,i]
        return np.linalg.norm(q1-q2)

    def closeGripper(self):
        self.gripper = [GRIPPER_CLOSE, GRIPPER_CLOSE]
        self.publish_point(self.q0.tolist())
    
    def openGripper(self):
        self.gripper = [GRIPPER_OPEN, GRIPPER_OPEN]
        self.publish_point(self.q0.tolist())
    
    def findClosestQ(self, q):
        minDistance = float('inf')
        closestQ = None
        qf_ = np.zeros(6)
        for q_ in q:
            distance = self.eucledianDistance(self.q0, q_)
            if distance < minDistance:
                minDistance = distance
                for i in range(6):
                    qf_[i] = q_[0,i]
                closestQ = qf_
        return closestQ 


def main():
    rospy.init_node('publisher', anonymous=True)
    pointPub = Publisher()
    while True:
        pointPub.move([0.43, -0.31, 0.6], [0, 0, 0])
        rospy.sleep(2)
        pointPub.move([0.43, -0.31, 0.77], [0, 0, 0])
        rospy.sleep(2)
        pointPub.closeGripper()
        rospy.sleep(2)
        pointPub.move([0.43, -0.31, 0.6], [0, 0, 0])
        rospy.sleep(2)
        pointPub.move([0.43, -0.31, 0.77], [0, 0, 0])
        rospy.sleep(2)
        pointPub.openGripper()
        rospy.sleep(2)
        
if __name__ == '__main__':
    main()