#!/usr/bin/python3

import rospy
import time
from guppy_drivers import ms5837
from guppy_msgs.msg import DepthData

if __name__ == '__main__':

    Depth = [0.0]*2
    sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)

    if not sensor.init():
            print ("Sensor could not be initialized")
            exit(1)

    # We have to read values from sensor to update pressure and temperature
    if not sensor.read():
        print ("Sensor read failed!")
        exit(1)

    """print("Pressure: %.2f atm  %.2f Torr  %.2f psi" % (
    sensor.pressure(ms5837.UNITS_atm),
    sensor.pressure(ms5837.UNITS_Torr),
    sensor.pressure(ms5837.UNITS_psi)))

    print("Temperature: %.2f C  %.2f F  %.2f K"% (
    sensor.temperature(ms5837.UNITS_Centigrade),
    sensor.temperature(ms5837.UNITS_Farenheit),
    sensor.temperature(ms5837.UNITS_Kelvin))"""

    ATM=sensor.pressure(ms5837.UNITS_Pa)
    #print("ATM: %.2f Pa"%(ATM))
    freshwaterDepth = sensor.depth(ATM) # default is freshwater
    sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
    saltwaterDepth = sensor.depth(ATM) # No nead to read() again
    sensor.setFluidDensity(1000) # kg/m^3

    time.sleep(5)


    rospy.init_node('depth_sensor_ms5837')
    depth_data_pub = rospy.Publisher('depth/data', DepthData, queue_size=1)

    last_time = last_depth = None

    while not rospy.is_shutdown():

        if sensor.read():

            Depth[0] = sensor.depth(ATM)
            Depth[1] = sensor.temperature(ms5837.UNITS_Centigrade)

            depth_data_msg = DepthData()
            depth_data_msg.depth = Depth[0]
            if last_time is not None and last_depth is not None: 
                depth_data_msg.velocity = (Depth[0] - last_depth) / (time.time() - last_time)
            
            depth_data_pub.publish(depth_data_msg)

            last_time = time.time()
            last_depth = Depth[0]
            
            