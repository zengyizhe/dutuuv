#!/usr/bin/env python3
import serial
import threading

AngleData=[0.0]*6
FrameState = 0
Bytenum = 0
CheckSum = 0

Angle = [0.0]*3
angle = [0]*3
def my_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def DueData(inputdata):
    global  FrameState
    global  Bytenum
    global  CheckSum
    global  Angle
    global  angle
    for data in inputdata:  
        if FrameState < 2:   
            if data==0x5a and (Bytenum==0 or Bytenum==1): 
                CheckSum=data
                Bytenum += 1
                #print(Bytenum)
                continue
            elif data==0x10 and Bytenum==2:
                CheckSum+=data
                FrameState +=1
                Bytenum += 1
                #print(Bytenum)
                continue
            elif data==0x06 and Bytenum==3:
                CheckSum+=data
                FrameState +=1
                Bytenum=4
                #print(Bytenum)

        elif FrameState==2: # angle

            if Bytenum<10:
                AngleData[Bytenum-4]=data
                CheckSum+=data
                Bytenum+=1
                #print(Bytenum)
            else:
                Angle = get_angle(AngleData)
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
                """
                if(angle[2]>180):
                    angle[2] - 180
                else:
                    angle[2] + 180"""
                
                print("Angle(deg):%10.3f %10.3f %10.3f"%Angle)
                print("X:%d  Y:%d  Z:%d"%(angle[0],angle[1],angle[2]))
                data = [hex(i) for i in AngleData]
                print(data)
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

port = "/dev/ttyAMA0"
baud = 9600
ser = serial.Serial(port, baud, timeout=0.5)
print(ser)

if __name__ == '__main__':
    # rospy.init_node('imu_mpu6050')
    # imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=1)
    # imu_msg = Imu()

    while True:
        datahex = ser.read(size=11)
        DueData(datahex)
        # imu_msg.orientation =    
        # imu_pub.publish(angle)
        # print(angle)
        # print(acc)
        # print(gyro)

    # rospy.spin()
        
        
        

