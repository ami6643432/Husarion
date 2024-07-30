#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import WrenchStamped, Point, PoseStamped, TwistStamped, Accel, Vector3
from nav_msgs.msg import *
import dynamixel_sdk
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tty
import termios
import select
import numpy as np
import tf
import time
import os
from collections import deque
from scipy.signal import firwin

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class RFTForceListener:
    def __init__(self):
        self.num_taps = 10
        self.cutoff_frequency = 0.2
        self.coefficients = firwin(self.num_taps, self.cutoff_frequency)
        
        # Initialize deques for x and z force values
        self.force_x_values = deque(maxlen=100)
        self.force_z_values = deque(maxlen=100)
        
        # Initialize deques for filtered force values
        self.filtered_force_x_values = deque(maxlen=100)
        self.filtered_force_z_values = deque(maxlen=100)
        
        # Initialize current force values
        self.current_x_force = 0.0
        self.current_z_force = 0.0
        
        rospy.Subscriber("/RFT_FORCE", WrenchStamped, self.callback)

    def callback(self, data):
        # Get the current x and z force values
        self.current_x_force = data.wrench.force.x
        self.current_z_force = data.wrench.force.z
        
        # Append the current forces to their respective deques
        self.force_x_values.append(self.current_x_force)
        self.force_z_values.append(self.current_z_force)
        
        # Filter the force values if there are enough samples
        if len(self.force_x_values) >= self.num_taps:
            filtered_force_x = np.dot(self.coefficients, list(self.force_x_values)[-self.num_taps:])
            self.filtered_force_x_values.append(filtered_force_x)
        
        if len(self.force_z_values) >= self.num_taps:
            filtered_force_z = np.dot(self.coefficients, list(self.force_z_values)[-self.num_taps:])
            self.filtered_force_z_values.append(filtered_force_z)
        
        # Uncomment the following lines if you want to log the filtered forces
        # rospy.loginfo("Filtered x force: %f", filtered_force_x)
        # rospy.loginfo("Filtered z force: %f", filtered_force_z)


class DynamixelControl:
    def __init__(self, id):
        self.PRESENT_POSITION = 0
        self.OPERATING_MODE = 5  # Current-based position control mode

        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_GOAL_CURRENT = 102
        self.ADDR_PRESENT_VELOCITY = 128
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_PRESENT_CURRENT = 126
        self.PROTOCOL_VERSION = 2.0
        self.DXL_ID = id
        self.BAUDRATE = 57600
        self.DEVICENAME = '/dev/ttyUSB1'
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        self.DXL_MINIMUM_POSITION_VALUE = 0
        self.DXL_MAXIMUM_POSITION_VALUE = 4095  # Full range for a 12-bit servo
        self.DXL_MOVING_STATUS_THRESHOLD = 20
        self.init_motor_connection()

    def init_motor_connection(self):
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            getch()
            quit()
        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            getch()
            quit()
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("DYNAMIXEL disabled")
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("DYNAMIXEL set to current-based position control mode")
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("DYNAMIXEL enabled")
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            getch()
            quit()
        else:
            print("DYNAMIXEL has been successfully connected")

    def shutdown(self):
        print('shutdown')
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def get_motor_data(self):
        self.PRESENT_POSITION, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        self.PRESENT_VELOCITY, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
        self.PRESENT_CURRENT, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_CURRENT)
        self.PRESENT_POSITION = self.PRESENT_POSITION * (360 / 4095)  # Convert ticks to degrees
        if self.PRESENT_VELOCITY > 4294967296 / 2:
            self.PRESENT_VELOCITY = self.PRESENT_VELOCITY - 4294967296
        if self.PRESENT_CURRENT > 65536 / 2:
            self.PRESENT_CURRENT = self.PRESENT_CURRENT - 65536
        self.PRESENT_CURRENT = self.PRESENT_CURRENT * 2.69
        self.PRESENT_VELOCITY = self.PRESENT_VELOCITY * 0.229 * (360 / 60)

    def set_goal_pos_callback(self, position):
        pos_ticks = int(round(position * (4095 / 360)))
        # print("Position in ticks : ", pos_ticks)
        if 0 <= pos_ticks <= 4095:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, pos_ticks)
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[RxPacketError] The data value exceeds the limit value!")

    def set_goal_curr_callback(self, curr_ticks):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, curr_ticks)

def compute_force_jacobian(theta1, theta2, L1, L2):
    # Compute the elements of the Jacobian matrix
    J11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    J12 = -L2 * np.sin(theta1 + theta2)
    J21 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    J22 = L2 * np.cos(theta1 + theta2)
    # Form the Jacobian matrix
    J = np.array([[J11, J12], [J21, J22]])
    # The force Jacobian J_f is the transpose of the geometric Jacobian J
    J_f = J.T
    return J_f

def get_transformation_matrix(theta2):
    return np.array([[np.cos(theta2), -np.sin(theta2)], [np.sin(theta2), np.cos(theta2)]])

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def main():
    rospy.init_node('dynamixel_control', anonymous=True)
    print("All Good")
    dynamixel_obj1 = DynamixelControl(1)
    time.sleep(0.5)
    dynamixel_obj2 = DynamixelControl(2)
    time.sleep(0.5)
    force_listener = RFTForceListener()
    time.sleep(0.5)

    # posd = 225
    # posd2 = 250

    posd = 200
    posd2 = 270

    posd_new = posd
    posd2_new = posd2

    dynamixel_obj1.set_goal_pos_callback(int(posd_new))
    dynamixel_obj2.set_goal_pos_callback(int(posd2_new))    

    dynamixel_obj1.set_goal_curr_callback(int(100))
    dynamixel_obj2.set_goal_curr_callback(int(100))

    flag = 1

    flag2 = 1

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            if is_data():
                c = sys.stdin.read(1)
                if c == '\x1b':
                    dynamixel_obj1.shutdown()
                    dynamixel_obj2.shutdown()
                    break
            # pos1h = -90
            # pos2h = -180
            dynamixel_obj1.get_motor_data()
            dynamixel_obj2.get_motor_data()

            pos = dynamixel_obj1.PRESENT_POSITION
            cur = dynamixel_obj1.PRESENT_CURRENT
            vel = dynamixel_obj1.PRESENT_VELOCITY
            pos2 = dynamixel_obj2.PRESENT_POSITION
            cur2 = dynamixel_obj2.PRESENT_CURRENT
            vel2 = dynamixel_obj2.PRESENT_VELOCITY

            ftx = force_listener.filtered_force_x_values[-1]
            ftz = force_listener.filtered_force_z_values[-1]

            q = np.array([pos, pos2])
            q_dot = np.array([vel, vel2])
            qd = np.array([posd, posd2])
            qd_dot = np.array([0, 0])
            ep = qd - q
            ev = qd_dot - q_dot
            m1 = 1
            m2 = 1
            L1 = 1
            L2 = 7
            theta1, theta2 = pos * np.pi / 180, pos2 * np.pi / 180
            theta1_dot, theta2_dot = vel * np.pi / 180, vel2 * np.pi / 180
            g = 9.81
            M = np.array([
                [m1 * L1**2 + m2 * (L1**2 + 2 * L1 * L2 * np.sin(theta2) + L2**2), m2 * (L1 * L2 * np.sin(theta2) + L2**2)],
                [m2 * (L1 * L2 * np.sin(theta2) + L2**2), m2 * L2**2]
            ])
            c_vector = np.array([
                -m2 * L1 * L2 * np.cos(theta2) * (2 * theta1_dot * theta2_dot + theta2_dot**2),
                m2 * L1 * L2 * theta1_dot**2 * np.cos(theta2)
            ])
            g_vector = np.array([
                (m1 + m2) * L1 * g * np.sin(theta1) + m2 * L2 * g * np.sin(theta1 + theta2),
                m2 * L2 * g * np.sin(theta1 + theta2)
            ])
            lam = 1e-2 * np.array([
                [16, 0],
                [0, 8]
            ])
            phip = np.array([
                [4, 0],
                [0, 4]
            ])

            phiv = np.array([
                [0, 0],
                [0, 0]
            ])

            J_f = compute_force_jacobian(theta1, theta2, L1, L2)

            Ftheta = J_f.dot(np.array([ftx, ftz]))

            tau = Ftheta
            tau = np.round(tau)
            tau1 = tau[0]
            tau2 = tau[1]

            print("posd_new1:", posd_new)
            print("posd_new2:", posd2_new)

            # print("Ftx: ", ftx)
            # print("Ftz: ", ftz)

            # print(dynamixel_obj1.DXL_ID, "Position ", " : ", pos, " degrees")
            # print(dynamixel_obj2.DXL_ID, "Position ", " : ", pos2, " degrees")
            print("Ftheta : ", Ftheta)
            # print("Desired Position : ",  posd)
            # print("Desired Position 2 : ",  posd2)

 



            # dynamixel_obj1.set_goal_pos_callback(int(posd_new))
            # dynamixel_obj2.set_goal_pos_callback(int(posd2_new))

        

            # dynamixel_obj1.set_goal_curr_callback(int(tau1))
            # dynamixel_obj2.set_goal_curr_callback(int(tau2))


            dynamixel_obj1.set_goal_curr_callback(int(120))
            dynamixel_obj2.set_goal_curr_callback(int(130))



            if (abs(Ftheta[0]) + abs(Ftheta[1])) > 40 and flag == 1 :
                flag2 = 0

            if flag2 == 0:
                time.sleep(1)
                while posd_new > posd-70 or posd2_new < posd2 + 35:
                    posd_new = posd_new - 1

                    if posd2_new < posd2 + 35:
                        posd2_new = posd2_new + 0.5

                    dynamixel_obj1.set_goal_pos_callback(int(posd_new))
                    dynamixel_obj2.set_goal_pos_callback(int(posd2_new))

                    # dynamixel_obj1.set_goal_curr_callback(int(110 - 1.5*Ftheta[0]))
                    # dynamixel_obj2.set_goal_curr_callback(int(130 - 1.5*Ftheta[1]))

                    
                    # time.sleep(0.001)

                flag = 0 




    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
