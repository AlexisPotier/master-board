# coding: utf8

import argparse
import math
import os
import sys
from time import clock

import libmaster_board_sdk_pywrap as mbs
import matplotlib.pyplot as plt


def example_script(name_interface):

    N_SLAVES = 6  #  Maximum number of controled drivers
    N_SLAVES_CONTROLED = 1  # Current number of controled drivers

    cpt = 0  # Iteration counter
    dt = 0.001  #  Time step
    t = 0  # Current time
    kp = 5.0  #  Proportional gain
    kd = 0.1  # Derivative gain
    iq_sat = 1.0  # Maximum amperage (A)
    freq = 0.5  # Frequency of the sine wave
    amplitude = math.pi  # Amplitude of the sine wave
    init_pos = [0.0 for i in range(N_SLAVES * 2)]  # List that will store the initial position of motors
    state = 0  # State of the system (ready (1) or not (0))

    cmd_lost_list = []
    sensor_lost_list = []
    cmd_ratio_list = []
    sensor_ratio_list = []
    time_list = []
    histogram_sensor = []
    histogram_cmd = []

    print("-- Start of example script --")

    os.nice(-20)  #  Set the process to highest priority (from -20 highest to +20 lowest)
    robot_if = mbs.MasterBoardInterface(name_interface)
    robot_if.Init()  # Initialization of the interface between the computer and the master board
    try :
        for i in range(N_SLAVES_CONTROLED):  #  We enable each controler driver and its two associated motors
            robot_if.GetDriver(i).motor1.SetCurrentReference(0)
            robot_if.GetDriver(i).motor2.SetCurrentReference(0)
            robot_if.GetDriver(i).motor1.Enable()
            robot_if.GetDriver(i).motor2.Enable()
            robot_if.GetDriver(i).EnablePositionRolloverError()
            robot_if.GetDriver(i).SetTimeout(5)
            robot_if.GetDriver(i).Enable()

        last = clock()

        while (not robot_if.IsTimeout() and not robot_if.IsAckMsgReceived()):
            if ((clock() - last) > dt):
                last = clock()
                robot_if.SendInit()

        if robot_if.IsTimeout():
            print("Timeout while waiting for ack.")

        while ((not robot_if.IsTimeout())
            and (clock() < 20)):  # Stop after 15 seconds (around 5 seconds are used at the start for calibration)

            if ((clock() - last) > dt):
                last = clock()
                cpt += 1
                t += dt
                robot_if.ParseSensorData()  # Read sensor data sent by the masterboard

                if (state == 0):  #  If the system is not ready
                    state = 1
                    for i in range(N_SLAVES_CONTROLED * 2):  # Check if all motors are enabled and ready
                        if not (robot_if.GetMotor(i).IsEnabled() and robot_if.GetMotor(i).IsReady()):
                            state = 0
                        t = 0

                if ((cpt % 100) == 0):  # Display state of the system once every 100 iterations of the main loop
                    print(chr(27) + "[2J")
                    # To read IMU data in Python use robot_if.imu_data_accelerometer(i), robot_if.imu_data_gyroscope(i)
                    # or robot_if.imu_data_attitude(i) with i = 0, 1 or 2
                    robot_if.PrintIMU()
                    robot_if.PrintADC()
                    robot_if.PrintMotors()
                    robot_if.PrintMotorDrivers()
                    robot_if.PrintCmdStats()
                    robot_if.PrintSensorStats()
                    sys.stdout.flush()  # for Python 2, use print( .... , flush=True) for Python 3

                    cmd_lost_list.append(robot_if.GetCmdLost())
                    sensor_lost_list.append(robot_if.GetSensorsLost())
                    cmd_ratio_list.append(100*robot_if.GetCmdLost()/robot_if.GetCmdSent())
                    sensor_ratio_list.append(100*robot_if.GetSensorsLost()/robot_if.GetSensorsSent())
                    time_list.append(clock())

                robot_if.SendCommand()  # Send the reference currents to the master board

        #Stop if clock > 20 : if it's timeout, the interface already calls Stop()
        if(not robot_if.IsTimeout()): 
            robot_if.Stop()  # Shut down the interface between the computer and the master board

        if robot_if.IsTimeout():
            print("Masterboard timeout detected.")
            print("Either the masterboard has been shut down or there has been a connection issue with the cable/wifi.")

    except KeyboardInterrupt:
        print("Keyboard Interrupt")
        robot_if.Stop()  # Shut down the interface between the computer and the master board


    #Plot histograms and graphs
    for i in range(20):
        histogram_sensor.append(robot_if.GetSensorHistogram(i))
        histogram_cmd.append(robot_if.GetCmdHistogram(i))
    
    plt.figure(1)
    plt.subplot(211)
    plt.bar(range(20), histogram_sensor)
    plt.title('Histogram : sensors lost') 
    plt.xticks(range(20), range(1,21))

    plt.subplot(212)
    plt.bar(range(20), histogram_cmd)
    plt.title('Histogram : commands lost') 
    plt.xticks(range(20), range(1,21))
    

    plt.figure(2)
    plt.subplot(211)
    plt.plot(time_list, cmd_lost_list)
    plt.xlabel('time')
    plt.ylabel('commands lost')

    plt.subplot(212)
    plt.plot(time_list, sensor_lost_list)
    plt.xlabel('time')
    plt.ylabel('sensors lost')


    plt.figure(3)
    plt.subplot(211)
    plt.plot(time_list, cmd_ratio_list)
    plt.xlabel('time')
    plt.ylabel('ratio command loss')

    plt.subplot(212)
    plt.plot(time_list, sensor_ratio_list)
    plt.xlabel('time')
    plt.ylabel('ratio sensor loss')
    plt.show()

    print("-- End of example script --")


def main():
    parser = argparse.ArgumentParser(description='Example masterboard use in python.')
    parser.add_argument('-i',
                        '--interface',
                        required=True,
                        help='Name of the interface (use ifconfig in a terminal), for instance "enp1s0"')

    example_script(parser.parse_args().interface)


if __name__ == "__main__":
    main()
