#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import numpy as np
import serial
import threading
import time

PI = 3.1415926535897

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


GetData=[0.0]*18
FrameState = 0
Bytenum = 0
CheckSum = 0

Angle = [0.0]*3
angle = [0]*3
acc = [0]*3
gyro = [0]*3
def my_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
def get_s16(val):
    if val < 0x8000:
        return val
    else:
        return (val - 0x10000)

def DueData(inputdata):
    global  FrameState
    global  Bytenum
    global  CheckSum
    global  Angle
    global  angle
    global  acc
    global  gyro
    global  GetData
    for data in inputdata:  #在输入的数据进行遍历
        if FrameState < 2:   #当未确定状态的时候，进入以下判断
            if data==0x5a and (Bytenum==0 or Bytenum==1): #帧为前两个0x5a头，开始读取数据，增大bytenum
                CheckSum=data
                Bytenum += 1
                #print(Bytenum)
                continue
            elif data==0x13 and Bytenum==2:#判断输出什么信息
                CheckSum+=data
                FrameState +=1
                Bytenum += 1
                #print(Bytenum)
                continue
            elif data==0x12 and Bytenum==3:#数据几个字节
                CheckSum+=data
                FrameState +=1
                Bytenum=4
                #print(Bytenum)

        elif FrameState==2: # angle

            if Bytenum<22:
                GetData[Bytenum-4]=data
                CheckSum+=data
                Bytenum+=1
                #print(Bytenum)
            else:
                acc = get_acc(GetData[:6])
                gyro = get_gyro(GetData[6:12])
                Angle = get_angle(GetData[12:22])

                angle[0] = int(Angle[0])
                if(angle[0]<180):
                    angle[0] = -angle[0]
                else:
                    angle[0] = my_map(angle[0],360,180,0,180)
                angle[1] = int(Angle[1])
                if(angle[1]<180):
                    angle[1] = -angle[1]
                else:
                    angle[1] = my_map(angle[1],360,180,0,180)
                angle[2] = my_map(int(Angle[2]),0,360,360,0)+180
                if(angle[2] > 360):
                    angle[2] -= 360
                
                # print('accels:',acc)
                
           
                # print('gyros:',gyro)    
                print('angles:',angle)

                CheckSum=0
                Bytenum=0
                FrameState=0

def get_angle(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    k_angle = 180.0
    angle_x = (rxl<<8|rxh)/100
    angle_y = (ryl<<8|ryh)/100
    angle_z = (rzl<<8|rzh)/100

    if angle_x >= k_angle:
        angle_x -= 300
    if angle_y >= k_angle:
        angle_y -= 295
    if angle_z >=k_angle:
        angle_z-= 300
    return angle_x,angle_y,angle_z

def get_acc(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    _g = 16383.5
    # _g = 1
    acc_x = get_s16(rxl<<8|rxh)/_g
    acc_y = get_s16(ryl<<8|ryh)/_g
    acc_z = get_s16(rzl<<8|rzh)/_g

    return acc_x,acc_y,acc_z

def get_gyro(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    _gyro = 16.3835
    gyro_x = get_s16(rxl<<8|rxh)/_gyro
    gyro_y = get_s16(ryl<<8|ryh)/_gyro
    gyro_z = get_s16(rzl<<8|rzh)/_gyro
    return gyro_x,gyro_y,gyro_z

port = "/dev/ttyAMA0"
baud = 9600
ser = serial.Serial(port, baud, timeout=0.5)

if __name__ == '__main__':
    # global D2R
    rospy.init_node('imu')
    imu_pub = rospy.Publisher("imu/data", Imu, queue_size=2)
    rate = rospy.Rate(50)


    while not rospy.is_shutdown():
        datahex = ser.read(23)
        DueData(datahex)

        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu_link'
        
        quat = Quaternion()
        _ = get_quaternion_from_euler(angle[0], angle[1], angle[2])
        quat.x = _[0]
        quat.y = _[1]
        quat.z = _[2]
        quat.w = _[3]

        imu_msg.orientation = quat
        imu_msg.angular_velocity.x = -(gyro[0] / 360.0) * (PI * 2) 
        imu_msg.angular_velocity.y = -(gyro[1] / 360.0) * (PI * 2)
        imu_msg.angular_velocity.z = -(gyro[2] / 360.0) * (PI * 2)
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = acc[1]
        imu_msg.linear_acceleration.z = acc[2]
        
        imu_pub.publish(imu_msg)
        rate.sleep()


# th = threading.Thread(target=main, args=())
# th.start()
