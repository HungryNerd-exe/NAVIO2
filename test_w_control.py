#!/usr/bin/env python
# MAE 5010 Autopilot Design airborne man test file.
# 3 Nov 2019, N. Baker and I. Faruque, i.faruque@okstate.edu

from __future__ import print_function

import math
import multiprocessing
import os
import time
from datetime import datetime
from math import sin, cos, tan

import numpy as np
from pymavlink import mavutil

import air


# from ThePartysEKF import EKF


def current_milli_time(): return int(round(time.time() * 1000))


# define indices of xh for easier access.
#p_n, , vt, alpha, beta, phi, theta, psi, p, q, r = range(12) # Unused -BRANDON

# define indices of y for easier access.
ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z = range(9)
pres_baro = 9
gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d = range(10, 16)

# define indices of servo for easier access.
mode_flag = 0
rcin_0, rcin_1, rcin_2, rcin_3, rcin_4, rcin_5 = range(1, 7)
servo_0, servo_1, servo_2, servo_3, servo_4, servo_5 = range(7, 13)
throttle, aileron, elevator, rudder, none, flaps = range(7, 13)

# define indices of cmd for easier access.
psi_c, h_c = range(2)


def estimator_loop(y, xh, servo):
    # get sensors for read_sensor function call.
    adc, imu, baro, ubl = air.initialize_sensors()

    #Initialize Variables
    ax_rot = ay_rot = az_rot = 0
    v_n_old = v_e_old = v_d_old = 0
    p_n = p_e = p_d = 0
    u = v = w = 0
    v_n = v_e = v_d = 0
    v_n_dot = v_e_dot = v_d_dot = 0

    A = [[-0.028, 0.233, 0, -9.815, 0, 0, 0, 0, 0, 0, 0, 0],
         [-0.978, -8.966, 20.1170, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0.102, 0.022, -6.102, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, -0.45, 0, -0.986, 0.635, 0, 0, 0, 0],
         [0, 0, 0, 0, 57.028, -72.97, 3.279, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 135.737, -0.588, -4.436, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
         [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
         [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    B = [[0, 1, 0, 0],
         [-23.448, 0, 0, 0],
         [-50.313, -0.104, 0, 0],
         [0, 0, 0, 0],
         [0, 0, 0, 0.315],
         [0, 0, 677.27, 18.099],
         [0, 0, -8.875, -99.521],
         [0, 0, 0, 0],
         [0, 0, 0, 0],
         [0, 0, 0, 0],
         [0, 0, 0, 0],
         [0, 0, 0, 0]]

    F_c = [[0.9997, 0.002, 0.0, -0.098, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
           [-0.0092, 0.9142, 0.1865, 0.00046, 0, 0, 0, 0, 0, 0, 0, 0],
           [0.001, 0.0, 0.94, -4.888, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.01, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0, 0.989, 0.0, -0.0096, 0.006, 0.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0, 0.42, 0.482, 0.02, 0.0015, 0.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0, 1.32, -0.004, 0.95, 0.004, 0.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0, 0.002, 0.007, 0.0, 1, 0.0, 0.0, 0.0, 0.0],
           [0.01, 0.0, 0.0, -0.0004, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0, 0.00996, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
           [0.0, 0.00956, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

    G_c = [[-0.0002, 0.01, 0.0, 0.0],
           [-0.272, 0.0, 0.0, 0.0],
           [-0.488, 0.0, 0.0, 0.0],
           [-0.00247, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.00054, 0.00796],
           [0.0, 0.0, 4.806, 0.1172],
           [0.0, 0.0, -0.102, -0.9696],
           [0.0, 0.0, 0.0269, 0.000679],
           [0.0, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0],
           [-0.0013, 0.0, 0.0, 0.0],
           [0.0, 0.0, 0.0, 0.0]]

    H_c = [[0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0]]

    time.sleep(3)  # Allows for initial reading of sensors

    # ==========================================================================
    # BIAS REMOVAL
    # POC: Brandon
    print('WARNING! LEVEL AIRCRAFT UNTIL FURTHER NOTICE!')
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, 'WARNING! LEVEL AIRCRAFT UNTIL FURTHER NOTICE!')
    time.sleep(2)
    # Give 10 seconds of warmup
    t1 = time.time()
    gyro = np.array([[0, 0, 0]])
    accel = np.array([[0, 0, 0]])
    while time.time() - t1 < 5:
        m9a, m9g, m9m = imu.getMotion9()
        accel = np.append(accel, [m9a], axis=0)
        gyro = np.append(gyro, [m9g], axis=0)
        time.sleep(0.05)
    gyro_bias = [np.average(gyro[:, 0]), np.average(gyro[:, 1]), np.average(gyro[:, 2])]
    accel_bias = [np.average(accel[:, 0]), np.average(accel[:, 1]), np.average(accel[:, 2])]

    # >>> ADD IN COVARIANCE
    #sampled_data = np.array([[np.transpose(gyro[:,0])],
    #                         [np.transpose(gyro[:,0])],
    #                         [np.transpose(gyro[:,0])],
    #                         [np.transpose(accel[:,0])],
    #                         [np.transpose(accel[:,0])],
    #                         [np.transpose(accel[:,0])],
    #                         [np.transpose(accel[:,0])]]
    #R = np.cov(sampled_data)

    accel = 0  # Free memory
    gyro = 0  # Free memory
    print('Sensor Bias Calibration Completed')
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, 'Sensor Bias Calibration Completed')
    # ==========================================================================

    # ==========================================================================
    # Logging Initialization
    # POC: Charlie
    now = datetime.now()
    date_time = now.strftime('%y-%m-%d_%H:%M:%S')
    os.chdir('/home/pi/')
    f_logfile = open('log_' + date_time + '.csv', 'w+')
    est_log_string = 'phi_a, theta_a, psi_m, x, y, -h_b, u, v, w, accel_bias, gyro_bias, rcin_0, rcin_1, rcin_2, rcin_3, rcin_4, rcin_5, servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z, pres_baro, gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d\n'
    f_logfile.write(est_log_string)
    # ==========================================================================

    ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z = range(9)
    pres_baro = 9
    gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d, gps_fix = range(10, 17)
    adc_a0, adc_a1, adc_a2, adc_a3, adc_a4, adc_a5, est_curr_consumed, last_curr_time = range(17, 25)

    pres_sl = 1010
    rhoSL = 1.225
    g = 9.8065
    pressure_conversion = 100 / (rhoSL * g)

    # Define Q here
    Q = np.eye(15)

    while True:
        # ==================================================
        # Read Data
        new_gps = air.read_sensor(y, adc, imu, baro, ubl)  # updates values in y

        # ==================================================
        # Create X Hat
        # POCs: Ujjval, Nick, Brandon, Charlie

        # Correct directionalities
        R_imu = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, 1]])

        # >>> UJJVAL CHECK HERE
        [ax_rot, ay_rot, az_rot][0] = np.dot(R_imu, np.transpose([y[ax], y[ay], y[az]]))
        ax_rot = ax_rot - accel_bias[0]
        ay_rot = ay_rot - accel_bias[1]
        az_rot = az_rot - accel_bias[2]

        # Completing angular relations
        phi_a = np.arctan2(ay_rot, math.sqrt(ax_rot ** 2 + az_rot ** 2))  # Reliable
        theta_a = np.arctan2(ax_rot, math.sqrt(ay_rot ** 2 + az_rot ** 2))  # Reliable
        # psi_a = np.arctan2(math.sqrt(ax_rot**2+ay_rot**2), az_rot**2)     # Unreliable

        # Get Psi from Magnetometer
        mag = np.dot(R_imu, np.transpose(np.array([y[mag_x], y[mag_y], y[mag_z]])))
        psi_m = np.arctan2(mag[1], mag[0])

        # Now that we have these values, we make a rotation matrix
        Rhb = np.array([[np.cos(theta_a), np.sin(theta_a) * np.sin(phi_a), np.sin(theta_a) * np.cos(phi_a)],
                        [0, np.cos(phi_a), -np.sin(phi_a)],
                        [-np.sin(theta_a), np.cos(theta_a) * np.sin(phi_a), np.cos(theta_a) * np.cos(phi_a)]])

        # Barometer Altitude
        h_b = (y[pres_baro] - pres_sl) * pressure_conversion

        # Accept the rate gyro values
        # >>> UJJVAL CHECK HERE
        p = y[gyro_p] - gyro_bias[0]
        q = y[gyro_q] - gyro_bias[1]
        r = y[gyro_r] - gyro_bias[2]

        if new_gps:
            [v_n_old, v_e_old, v_d_old] = [v_n, v_e, v_d]
            [p_n, p_e, p_d, v_n, v_e, v_d] = [y[gps_posn_n], y[gps_posn_e], y[gps_posn_d], y[gps_vel_n], y[gps_vel_e],
                                        y[gps_vel_d]]
            delta_t = round(time.time() - t1, 3)
            t1 = time.time()
            try:
                [v_n_dot, v_e_dot, v_d_dot] = ([v_n, v_e, v_d] - [v_n_old, v_e_old, v_d_old]) / delta_t
            except:
                [v_n_dot, v_e_dot, v_d_dot] = [0, 0, 0]

            Rbf = np.array([[cos(theta_a)*cos(psi_m), cos(theta_a)*sin(psi_m), -sin(theta_a)],
                            [sin(phi_a)*sin(theta_a)*cos(psi_m) - cos(phi_a)*sin(psi_m), sin(phi_a)*sin(theta_a)*sin(psi_m) + cos(phi_a)*cos(psi_m), sin(phi_a)*cos(theta_a)]
                            [cos(phi_a)*sin(theta_a)*cos(psi_m) + sin(phi_a)*sin(psi_m), cos(phi_a)*sin(theta_a)*sin(psi_m) - sin(phi_a)*cos(psi_m), cos(phi_a)*cos(theta_a)]])

            [u, v, w][0] = np.dot(Rbf, np.array([[v_n], [v_e], [v_d]]))

        # XTODO: need to define x without GPS. Initialize as zero before loop? -Charlie
            #INITIALIZED BEFORE LOOP - BRANDON
        # TODO: need to define u, v, w. Not sure where those are comping from. -Charlie
            #UPDATED WITH ROTATION MATRIX
        xh = np.array([phi_a, theta_a, psi_m, x, y, -h_b, u, v, w, accel_bias, gyro_bias])

        # ==================================================
        # Kalman Matrices
        # POC: Ujjval

        # Create F Matix
        F = F_Find(xh, [y[ax], y[ay], y[az], y[gyro_p], y[gyro_q], y[gyro_r]])

        # Create H Matix
        # >>> TBD by UJJVAL
        # H = H_Find(xh, [y[ax], y[ay], y[az], y[gyro_p], y[gyro_q], y[gyro_r]])

        # Kalman Filter Algebra
        # >>> TBD

        # ==================================================
        # ALL CODE ABOVE THIS LINE
        # ==================================================
        # DONE: Log X Hat, Servos, RCs, Y to CSV
        f_logfile.write(', '.join(map(str, xh)) + ', '.join(map(str, servo)) + ', '.join(map(str, y)) + '\n')
        # >>> TDB


def F_Find(xh, sn):
    # MONOLITH 1: UJJVAL KNOWS All
    # MONOLITH 2: THIS IS CORRECT

    # Using Matlab's Partial Differentation.
    # x_hat = [ phi, theta, psi, x, y, z, V_n, V_e, V_d, bwx, bwy, bwz, bax, bay, baz]
    # All Angles in RADIANS!

    phi = xh[0]
    theta = xh[1]
    psi = xh[2]
    x = xh[3]
    y = xh[4]
    z = xh[5]
    V_n = xh[6]
    V_e = xh[7]
    V_d = xh[8]
    bwx = xh[9]
    bwy = xh[10]
    bwz = xh[11]
    bax = xh[12]
    bay = xh[13]
    baz = xh[14]

    # Sensor Data sn = [fx,fy,fz,wx,wy,wz]

    fx = sn[0]
    fy = sn[1]
    fz = sn[2]
    wx = sn[3]
    wy = sn[4]
    wz = sn[5]

    wx_cb = -1 * (wx - bwx)
    wy_cb = -1 * (wy - bwy)
    wz_cb = -1 * (wz - bwz)
    ax_cb = -1 * (fx - bax)
    ay_cb = -1 * (fy - bay)
    az_cb = -1 * (fz - baz)

    F = np.array([[sin(phi) * tan(theta) * (wz_cb) - cos(phi) * tan(theta) * wy_cb,
                   - cos(phi) * (wz_cb) * (tan(theta) ** 2 + 1) - sin(phi) * (wy_cb) * (tan(theta) ** 2 + 1), 0, 0, 0,
                   0, 0, 0, 0,
                   -1, -sin(phi) * tan(theta), -cos(phi) * tan(theta), 0, 0, 0],
                  [cos(phi) * (wz_cb) + sin(phi) * (wy_cb), 0, 0, 0, 0, 0, 0, 0, 0, 0, -cos(phi), sin(phi), 0, 0, 0],
                  [(sin(phi) * (wz_cb)) / cos(theta) - (cos(phi) * (wy_cb)) / cos(theta),
                   - (cos(phi) * sin(theta) * (wz_cb)) / cos(theta) ** 2 - (sin(phi) * sin(theta) * (wy_cb)) / cos(
                       theta) ** 2, 0,
                   0, 0, 0, 0, 0, 0, 0, -sin(phi) / cos(theta), -cos(phi) / cos(theta), 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0],
                  [- (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) * (ay_cb) - (
                          cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) * (az_cb),
                   cos(psi) * sin(theta) * (ax_cb) - cos(phi) * cos(psi) * cos(theta) * (az_cb) - cos(psi) * cos(
                       theta) * sin(
                       phi) * (ay_cb), (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) * (ay_cb) - (
                           cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) * (az_cb) + cos(theta) * sin(psi) * (
                       ax_cb), 0, 0, 0, 0, 0, 0, 0, 0, 0, -cos(psi) * cos(theta),
                   cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta),
                   - sin(phi) * sin(psi) - cos(phi) * cos(psi) * sin(theta)],
                  [(cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) * (ay_cb) + (
                          cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) * (az_cb),
                   sin(psi) * sin(theta) * (ax_cb) - cos(phi) * cos(theta) * sin(psi) * (az_cb) - cos(theta) * sin(
                       phi) * sin(
                       psi) * (ay_cb), (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) * (ay_cb) - (
                           sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) * (az_cb) - cos(psi) * cos(theta) * (
                       ax_cb), 0, 0, 0, 0, 0, 0, 0, 0, 0, -cos(theta) * sin(psi),
                   - cos(phi) * cos(psi) - sin(phi) * sin(psi) * sin(theta),
                   cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)],
                  [cos(theta) * sin(phi) * (az_cb) - cos(phi) * cos(theta) * (ay_cb),
                   cos(theta) * (ax_cb) + cos(phi) * sin(theta) * (az_cb) + sin(phi) * sin(theta) * (ay_cb), 0, 0, 0, 0,
                   0, 0, 0,
                   0, 0, 0, sin(theta), -cos(theta) * sin(phi), -cos(phi) * cos(theta)],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    return F


def controller_loop(xh, servo, cmd):
    while True:
        if (servo[mode_flag] == 1):
            pass  # rewrite servo_out values to servo array based on their previous values and xh.
            # if (servo[servo_1]<1.5): servo[servo_1] = 1.55
            # else: servo[servo_1] = 1.45
            # time.sleep(1)
            # Controller should assign values in range 1.25 to 1.75 to outputs;
            # WARNING, servo damage likely if values outside this range are assigned
            # Example: This is a manual passthrough function
            servo[throttle] = servo[rcin_0]
            servo[aileron] = servo[rcin_1]
            servo[elevator] = servo[rcin_2]
            servo[rudder] = servo[rcin_3]
            servo[servo_4] = servo[servo_4]  # no servo; channel used for manual/auto switch
            servo[flaps] = servo[rcin_5]


if __name__ == "__main__":

    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600, source_system=255)

    # initialize arrays for sharing sensor data.
    y = multiprocessing.Array('d', np.zeros(26))  # imu, baro, gps, adc
    xh = multiprocessing.Array('d', np.zeros(12))  # position, orientation, rates
    servo = multiprocessing.Array('d', np.zeros(13))  # mode_flag, rcin, servo_out
    cmd = multiprocessing.Array('d', np.zeros(2))  # psi_c, h_c

    # start processes for interpreting sensor data and setting servo pwm.
    estimator_process = multiprocessing.Process(target=estimator_loop, args=(y, xh, servo))
    estimator_process.daemon = True
    estimator_process.start()
    controller_process = multiprocessing.Process(target=controller_loop, args=(xh, servo, cmd))
    controller_process.daemon = True
    controller_process.start()
    servo_process = multiprocessing.Process(target=air.servo_loop, args=(servo,))
    servo_process.daemon = True
    servo_process.start()
    time.sleep(5)
    # start process for telemetry after other processes have initialized.
    telemetry_process = multiprocessing.Process(
        target=air.telemetry_loop, args=(y, xh, servo, master))
    telemetry_process.daemon = True
    telemetry_process.start()

    print("\nsending heartbeats to {} at 1hz.".format('/dev/ttyAMA0'))
    # loop for sending heartbeats and receiving messages from gcs.
    while True:
        time_sent = 0
        while True:
            # send heartbeat message if one second has passed.
            if ((current_milli_time() - time_sent) >= 980):
                master.mav.heartbeat_send(1, 0, 0, 0, 4, 0)
                # still haven't figured out how to get mode to show up in mission planner.
                # print('heartbeat sent.')
                time_sent = current_milli_time()
            # Simple waypoint tracker
            # DO WAYPOINT TRACKING HERE

            # handle incoming commands over telemetry
            # try:
            #     msg = master.recv_match().to_dict()
            #     if (not (msg['mavpackettype'] == 'RADIO' or msg['mavpackettype'] == 'RADIO_STATUS' or msg['mavpackettype'] == 'HEARTBEAT')):
            #         print(msg)
            #         if (msg['mavpackettype'] == 'COMMAND_LONG'):
            #             master.mav.command_ack_send(msg['command'],4)
            #             print("acknowledge sent.")
            # except:
            #     pass
