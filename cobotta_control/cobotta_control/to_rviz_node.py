# -*- coding:utf-8 -*-
# Send "Move" command to RC8
#b-cap Lib URL 
# https://github.com/DENSORobot/orin_bcap

from .pybcapclient.bcapclient import BCAPClient as bcapclient
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class JointReadNode(Node):
# DENSO ROBOT status
    def __init__(self):
        # set IP Address , Port number and Timeout of connected RC8
        self.host = "192.168.0.2"
        self.port = 5007
        self.timeout = 2000

        # set Parameter
        self.Name = ""
        self.Provider="CaoProv.DENSO.VRC"
        self.Machine = ("localhost")
        self.Option = ("")

        self.comp = 1
        self.loopflg = True
        self.ESC = 0x1B  # [ESC] virtual key code

        # Connection processing of tcp communication
        self.m_bcapclient = bcapclient(
            self.host,self.port,self.timeout
        )
        # start b_cap Service
        self.m_bcapclient.service_start("")

        # Connect to RC8 (RC8(VRC)provider)
        self.hCtrl = self.m_bcapclient.controller_connect(
            self.Name,self.Provider,self.Machine,self.Option
        )

        self.HRobot = self.m_bcapclient.controller_getrobot(self.hCtrl,"Arm","")

        # publish ROS msg of JointState
        super().__init__('joint_state_publish')
        self.pub_joint_state = self.create_publisher(JointState, '/joint_states', 30)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joint_state_msg = JointState()

        self.joint_state_msg.header.frame_id = 'joint_frame'
        self.joint_state_msg.name = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6",
            "joint_gripper_right", "joint_gripper_left",
        ]
        self.joint_state_msg.position = [0.0]*8  # joint_state_msg.position is Float 

        self.name_length = len(self.joint_state_msg.name)

    def get_param(self):      
        ### Get Position
        # CurJnt = J1,J2,J3,J4,J5,J6
        # CurPos = x,y,z,rx,ry,rz,fig
        curjnt = self.m_bcapclient.robot_execute(self.HRobot,"CurJnt")
        curpos = self.m_bcapclient.robot_execute(self.HRobot,"CurPos")
        torque = self.m_bcapclient.robot_execute(self.HRobot,"GetSrvData",4)
        speed = self.m_bcapclient.robot_execute(self.HRobot,"GetSrvData",17)

        #debag
        self.get_logger().info("jnt data:{}\n".format(curjnt[1]))

        return curjnt, curpos, torque, speed


    def timer_callback(self):

        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        curjnt, _, _, _ = self.get_param()

        for i in range (self.name_length):
            rviz_param_b = 0.0
            if i == self.joint_state_msg.name.index("joint_1"):
                rviz_param_a = -2.62/150
            elif i == self.joint_state_msg.name.index("joint_2"):
                rviz_param_a = 2.8/160
            elif i == self.joint_state_msg.name.index("joint_3"):
                rviz_param_a = -2.13/122
                rviz_param_b = 1.5713
            elif i == self.joint_state_msg.name.index("joint_4"):
                rviz_param_a = 2.97/170
            elif i == self.joint_state_msg.name.index("joint_5"):
                rviz_param_a = -4.02/230
                rviz_param_b = 1.5730
            elif i == self.joint_state_msg.name.index("joint_6"):
                rviz_param_a = 2.97/180            
            self.joint_state_msg.position[i] = rviz_param_a*curjnt[i]+rviz_param_b


        self.get_logger().info('pub joint state name:{}, pos:{}'.format(self.joint_state_msg.name, self.joint_state_msg.position))
        
        self.pub_joint_state.publish(self.joint_state_msg)


def main(args=None):
    rclpy.init(args=args)

    joint_state_pub = JointReadNode()

    rclpy.spin(joint_state_pub)

    joint_state_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
