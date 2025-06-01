#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Float32
from collections import deque
flag_first_measurement = True
temperature = None
from sensor_msgs.msg import FluidPressure

def fluid_pressure_callback(data):
    # print("fluid_pressure_callback")
    global flag_first_measurement
    barometric_pressure = data.fluid_pressure
    sea_level_pressure = 100775  # Replace with the actual sea-level pressure in millibars
    # 100914
    # 101000

# https://www.xsens.com/hubfs/Downloads/Leaflets/MTi-630.pdf?__hstc=157421285.8694715230c7c1d5669747e22f69a108.1700834250057.1700834250057.1700834250057.1&__hssc=157421285.3.1700834250057&__hsfp=2698739698
# (1 hPa = 100 Pascals = 1 mb.)
# hPa is the abbreviated name for hectopascal (100 x 1 pascal) pressure units which are exactly equal to millibar pressure unit (mb or mbar).

    if temperature is None:
        # Apply moving average filter to pressure measurements
        filtered_pressure = moving_average_filter(barometric_pressure)

        pressure_msg = FluidPressure()
        pressure_msg.header.stamp = data.header.stamp
        pressure_msg.fluid_pressure = filtered_pressure
        filter_pressure_pub.publish(pressure_msg)
        


        print(filtered_pressure)
        # estimated_height = convert_barometer_to_height(barometric_pressure, sea_level_pressure, temperature)
        # estimated_height = convert_barometer_to_height(filtered_pressure, sea_level_pressure)
        estimated_height = convert_barometer_to_height(filtered_pressure,  sea_level_pressure)
        # if flag_first_measurement is True:
        #     calib_value = estimated_height
        #     flag_first_measurement = False
        #     print(calib_value)
        # calib_value = 142.2 - 3
        calib_value = 11
        # calibrated_height = estimated_height - calib_value
        calibrated_height = estimated_height 

        height_msg = FluidPressure()
        height_msg.header.stamp = rospy.Time.now()
        height_msg.fluid_pressure = calibrated_height

        height_pub.publish(height_msg)
        # height_pub.publish(calibrated_height)
        # print("height: ", calibrated_height)


def temperature_callback(data):
    # print("temperature_callback")
    global temperature
    temperature = data.temperature
    print(temperature)


# def convert_barometer_to_height(barometric_pressure, sea_level_pressure, temperature):
#     temperature_lapse_rate = 0.0065  # lapse rate in degrees Celsius per meter
#     gas_constant = 8.31432  # universal gas constant in J/(mol·K)
#     molar_mass_of_air = 0.0289644  # molar mass of dry air in kg/mol
#     acceleration_due_to_gravity = 9.80665  # acceleration due to gravity in m/s^2

#     temperature_kelvin = temperature + 273.15
#     pressure_difference = sea_level_pressure - barometric_pressure

#     height = (temperature_lapse_rate * gas_constant * temperature_kelvin) / (molar_mass_of_air * acceleration_due_to_gravity) * pressure_difference

#     return height


def moving_average_filter(new_measurement):
    # Append the new measurement to the deque
    pressure_measurements.append(new_measurement)

    # If the number of measurements exceeds the window size, remove the oldest measurement
    if len(pressure_measurements) > window_size:
        pressure_measurements.popleft()

    # Calculate the average of the measurements
    average_pressure = sum(pressure_measurements) / len(pressure_measurements)

    return average_pressure


def convert_barometer_to_height1(barometric_pressure, sea_level_pressure):
    # Calculate the pressure difference
    pressure_difference = sea_level_pressure - barometric_pressure

    # Constants for the ISA model
    temperature_lapse_rate = 0.0065  # lapse rate in degrees Celsius per meter
    gas_constant = 8.31432  # universal gas constant in J/(mol·K)
    molar_mass_of_air = 0.0289644  # molar mass of dry air in kg/mol
    acceleration_due_to_gravity = 9.80665  # acceleration due to gravity in m/s^2

    # Calculate the estimated height
    height = (temperature_lapse_rate * gas_constant) / (molar_mass_of_air * acceleration_due_to_gravity) * pressure_difference

    return height

def convert_barometer_to_height(pressure, sea_level_pressure=1013.25):
    # Standard atmospheric pressure at sea level in millibars (1013.25 hPa)
    altitude = 44330.8 * (1 - (pressure / sea_level_pressure) ** 0.1903)
    return altitude





if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('barometer_height_converter')

    # Create a publisher for the estimated height
    # height_pub = rospy.Publisher('estimated_height', Float32, queue_size=10)
    height_pub = rospy.Publisher('estimated_height', FluidPressure, queue_size=10)
    filter_pressure_pub = rospy.Publisher('filter_pressure', FluidPressure, queue_size=10)

    # Subscribe to the fluid pressure and temperature topics
    rospy.Subscriber('/imu_vins/pressure', FluidPressure, fluid_pressure_callback)
    rospy.Subscriber('/imu_vins/temperature', Temperature, temperature_callback)

    print("estimated_height")

    # Define the moving average window size
    window_size = 100

# Create a deque to store pressure measurements
    pressure_measurements = deque(maxlen=window_size)

    # Initialize the temperature variable
    temperature = None

    # Spin the ROS node
    rospy.spin()
