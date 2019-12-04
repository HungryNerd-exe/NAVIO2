#!/usr/bin/env python

from __future__ import print_function
from pymavlink import mavutil
import multiprocessing, time
from datetime import datetime
from math import sin, cos, tan
import numpy
import os
import numpy as np
import air

# define indices of xh for easier access.
#x, yy, z, vt, alpha, beta, phi, theta, psi, p, q, r = range(12)
phi, theta, psi, x, yy, z, V_n, V_e, V_d, p, q, r = range(12)

# define indices of y for easier access.
ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z = range(9)
pres_baro = 9
gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d = range(10,16)

# define indices of servo for easier access.
mode_flag = 0
rcin_0, rcin_1, rcin_2, rcin_3, rcin_4, rcin_5 = range(1,7)
servo_0, servo_1, servo_2, servo_3, servo_4, servo_5 = range(7,13)
throttle, aileron, elevator, rudder, none, flaps = range(7,13)

# define indices of cmd for easier access.
psi_c, h_c = range(2)

def estimator_loop(y,xh,servo):
    # get sensors for read_sensor function call.
    adc,imu,baro,ubl = air.initialize_sensors()
    time.sleep(3)
    count=0
    #Sensor installation details
    Rba=np.array([[0,-1,0], [-1,0,0], [0,0,1]]) #acc frame to body (eg, imu to body)
    #Environmental parameters
    declination=+3.233*np.pi/180 #rad, mag declination is +3deg14' (eg, east) in Stillwater 
    pres_sl=1010		#millibars, sea level pressure for the day. Update me! 1mb is 100Pa
    rhoSL=1.225			#kg/m^2, sea level standard density
    g=9.8065 #m/s^2, gravity
    
    #bias calculate
    print('WARNING! LEVEL AIRCRAFT UNTIL FURTHER NOTICE!')
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, 'WARNING! LEVEL AIRCRAFT UNTIL FURTHER NOTICE!')
    # Give 10 seconds of warmup
    t1 = time.time()
    gyro = np.array([[0, 0, 0]])
    accel = np.array([[0, 0, 0]])
    mag = np.array([[0, 0, 0]])
    while time.time() - t1 < 1:
        m9a, m9g, m9m = imu.getMotion9()
        accel = np.append(accel, [m9a], axis=0)
        gyro = np.append(gyro, [m9g], axis=0)
        mag = np.append(mag, [m9m], axis=0)
        time.sleep(0.05)
    gyro_bias = [np.average(gyro[:, 0]), np.average(gyro[:, 1]), np.average(gyro[:, 2])]
    accel_bias = [np.average(accel[:, 0]), np.average(accel[:, 1]), np.average(accel[:, 2])]
    mag_bias = [np.average(mag[:, 0]), np.average(mag[:, 1]), np.average(mag[:, 2])]
    
    # Define Q here
    Q = np.eye(15)
    
    R_INS = np.diag([np.cov(accel[:, 0]),np.cov(accel[:, 1]),np.cov(accel[:, 2]), 1, 1, 1, 1, 1, 1, 1, np.cov(gyro[:, 0]),np.cov(gyro[:, 1]),np.cov(gyro[:, 2])])
    
    R_AHRS = np.diag([np.cov(accel[:, 0]),np.cov(accel[:, 1]),np.cov(accel[:, 2]), 10,np.cov(gyro[:, 0]),np.cov(gyro[:, 1]),np.cov(gyro[:, 2])])
    
    accel = 0
    gyro = 0
    mag = 0
    
    # Logging Initialization
    # POC: Charlie
    now = datetime.now()
    date_time = now.strftime('%y-%m-%d_%H:%M:%S')
    os.chdir('/home/pi/')
    f_logfile = open('log_' + date_time + '.csv', 'w+')
    est_log_string = 'phi_a, theta_a, psi_m, x, y, -h_b, u, v, w, accel_bias, gyro_bias, rcin_0, rcin_1, rcin_2, rcin_3, rcin_4, rcin_5, servo_0, servo_1, servo_2, servo_3, servo_4, servo_5, ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z, pres_baro, gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d\n'
    f_logfile.write(est_log_string)
    # =========================================================================

    ax, ay, az, gyro_p, gyro_q, gyro_r, mag_x, mag_y, mag_z = range(9)
    pres_baro = 9
    gps_posn_n, gps_posn_e, gps_posn_d, gps_vel_n, gps_vel_e, gps_vel_d, gps_fix = range(10, 17)
    adc_a0, adc_a1, adc_a2, adc_a3, adc_a4, adc_a5, est_curr_consumed, last_curr_time = range(17, 25)

   
    while True:
        initialEstTime = time.time()
        new_gps = air.read_sensor(y,adc,imu,baro,ubl) # updates values in y
        #initiate

        if count == 0:
            tp = time.time()
            xh_old = [0,0,0,0,0,0,0,0,0,0,0,0] #todo check and verify
            P_old = numpy.eye(len(xh))
            v_n_old = v_e_old = 0
            
        #First compute sensor-specific estimates for imu/mag sensors
        #Raw sensor data needs to be rotated to stability axes
        acc=Rba.dot( np.array([y[ax]-accel_bias[0],y[ay]-accel_bias[1],y[az]-accel_bias[2]]) )
        mag=Rba.dot( np.array([y[mag_x]-mag_bias[0],y[mag_y]-mag_bias[1],y[mag_z]-mag_bias[2]]))
        
        #Magnetic heading (psi_m)
        Rhb=np.array([[np.cos(xh_old[theta]), np.sin(xh_old[theta])*np.sin(xh_old[phi]), np.sin(xh_old[theta])*np.cos(xh_old[phi])], [0, np.cos(xh_old[phi]), -np.sin(xh_old[phi])], [-np.sin(xh_old[theta]), np.cos(xh_old[theta])*np.sin(xh_old[phi]), np.cos(xh_old[theta])*np.cos(xh_old[phi])]]) #rotation from body to horizontal plane
        #print(Rhb)

        magh=Rhb.dot(mag) #mag vector in horizontal plane components

        psi_m = np.arctan2(magh[1], magh[0]) + declination

        Rbf = np.array([[cos(xh_old[theta]) * cos(psi_m), cos(xh_old[theta]) * sin(psi_m), -sin(xh_old[theta])], [sin(xh_old[phi]) * sin(xh_old[theta]) * cos(psi_m) - cos(xh_old[phi]) * sin(psi_m),sin(xh_old[phi]) * sin(xh_old[theta]) * sin(psi_m) + cos(xh_old[phi]) * cos(psi_m), sin(xh_old[phi]) * cos(xh_old[theta])],[cos(xh_old[phi]) * sin(xh_old[theta]) * cos(psi_m) + sin(xh_old[phi]) * sin(psi_m), cos(xh_old[phi]) * sin(xh_old[theta]) * sin(psi_m) - sin(xh_old[phi]) * cos(psi_m), cos(xh_old[phi]) * cos(xh_old[theta])]]) #rotation from fixed to body frame

        #Pressure altitude  
        h_b= -(y[pres_baro]-pres_sl)*100 /(rhoSL*g)  #*100  from mb to Pa
    
    
        #=====ESTIMATOR CODE STARTS HERE==================================
        xh = xh_old
        sn = np.array([acc[0],acc[1],acc[2],y[gyro_p]-gyro_bias[0],y[gyro_q]-gyro_bias[1],y[gyro_r]-gyro_bias[2]])
        F = F_Find(xh, sn)
        P = P_old
        [xhminus, Phminus] = priori(xh, P, F, Q)
        
        #Handle GPS and then fuse all measurements
        if (new_gps):
            [xh[x], xh[yy], xh[z]] = [y[gps_posn_n], y[gps_posn_e], y[gps_posn_d]]
            [xh[V_n], xh[V_e], xh[V_d]] = np.dot(Rbf,np.array([y[gps_vel_n], y[gps_vel_n], y[gps_vel_n]]))
            zn = np.array([acc[0], acc[1], acc[2], psi_m, y[gps_posn_n], y[gps_posn_e], y[gps_posn_d], y[gps_vel_n], y[gps_vel_n], y[gps_vel_n], y[gyro_p]-gyro_bias[0], y[gyro_q]-gyro_bias[1],y[gyro_r]-gyro_bias[2]]) #todo check and make sure it is correct
            H = H_Find_INS(xh, sn)
            print('NEW GPS')
            [xh, P] = posteriori(xhminus, Phminus, zn, H, R_INS)
      
        else:
            zn = np.array([y[ax], y[ay], y[az], np.arctan2( y[mag_y], y[mag_x]), y[gyro_p]-gyro_bias[0], y[gyro_q]-gyro_bias[1], y[gyro_r]-gyro_bias[2]]) #todo check and make sure it is correct
            H = H_Find_AHRS(xh, sn)
            [xh, P] = posteriori(xhminus, Phminus, zn, H, R_AHRS)
            
            xh[x] = xh[x] + round(time.time() - tp, 3) * v_n_old
            xh[yy] = xh[yy] + round(time.time() - tp, 3) * v_e_old
            tp = time.time()
        #OUTPUT: write estimated values to the xh array--------------------------------------
        xh[z] = -h_b
        xh[psi] = psi_m
        vt = (xh[V_n]**2+xh[V_e]**2+xh[V_d]**2)**(1/2)
        xh_old = xh
        P_old = P
        alpha = numpy.arctan(xh[V_d]/xh[V_n])
        if vt == 0:
            beta = 0
        else:
            beta = numpy.arcsin(xh[V_e]/vt)
        [pe,qe,re]=np.dot(Rbf.T,[y[gyro_p]-gyro_bias[0], y[gyro_q]-gyro_bias[1], y[gyro_r]-gyro_bias[2]])
        
        # xhat for controller
        xc = np.array([xh[x], xh[yy], xh[z], vt, alpha, beta, xh[phi], xh[theta], xh[psi], pe, qe, re])
        
        # ==================================================
        # ALL CODE ABOVE THIS LINE
        # ==================================================
        # DONE: Log X Hat, Servos, RCs, Y to CSV
        f_logfile.write(', '.join(map(str, xc)) + ', '.join(map(str, servo)) + ', '.join(map(str, y)) + '\n')
        # >>> TDB
        
        count = 1
	if (count % 8000)==0:
	    print("Phi=%3.0f, Theta=%3.0f, Psi=%3.0f" %  (xh_old[phi]*180/np.pi, xh_old[theta]*180/np.pi, psi_m*180/np.pi))
	#======ESTIMATOR CODE STOPS HERE===================================
	#if (0.0125- (time.time()-initialEstTime) < 0): print( 1/(time.time()-initialEstTime) )
        time.sleep(max(0.0125-(time.time()-initialEstTime),0) )

def controller_loop(xh,servo,cmd):

    while True:
    	initial_time=time.time()
        # print("total milliseconds between controller_loop iterations: {}".format(initial_time-last_time))
        if (servo[mode_flag] == 1):
	    #======CONTROLLER CODE STARTS HERE===============================================
            # rewrite servo_out values to servo array based on their previous values and xh, cmd
            # if (servo[servo_1]<1.5): servo[servo_1] = 1.55
            # else: servo[servo_1] = 1.45
            # time.sleep(1)
            #Controller should assign values in range 1.25 to 1.75 to outputs;
            #WARNING, servo damage likely if values outside this range are assigned
            #Example: This is a manual passthrough function
            servo[throttle]=servo[rcin_0]
            servo[aileron]=servo[rcin_1]
            servo[elevator]=servo[rcin_2]
            servo[rudder]=servo[rcin_3]
            servo[servo_4]=servo[servo_4] #no servo; channel used for manual/auto switch
            servo[flaps]=servo[rcin_5]
	    #=======CONTROLLER CODE STOPS HERE ======================================
        time.sleep(max(0.0125-(time.time()-initial_time),0) )

def F_Find(xh, sn):
    # Using Matlab's Partial Differentation.
    # x_hat = [ phi, theta, psi, x, y, z, V_n, V_e, V_d, p, q, r]
    # Sensor Data sn = [ax,ay,az,wx,wy,wz]
    # All Angles in RADIANS!
    phi, theta, psi, x, y, z, V_n, V_e, V_d, p, q, r = xh
    ax,ay,az,wx,wy,wz = sn
    g = 9.808

    F = np.array([[sin(phi) * tan(theta) * (-1*wz) - cos(phi) * tan(theta) * -1*wy,
                   - cos(phi) * (-1*wz) * (tan(theta) ** 2 + 1) - sin(phi) * (-1*wy) * (tan(theta) ** 2 + 1), 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0],
                  [cos(phi) * (-1*wz) + sin(phi) * (-1*wy), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [(sin(phi) * (-1*wz)) / cos(theta) - (cos(phi) * (-1*wy)) / cos(theta),
                   - (cos(phi) * sin(theta) * (-1*wz)) / cos(theta) ** 2 - (sin(phi) * sin(theta) * (-1*wy)) / cos(
                       theta) ** 2, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, ],
                  [- (sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) * (-1*ay) - (
                          cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) * (-1*az),
                   cos(psi) * sin(theta) * (-1*ax) - cos(phi) * cos(psi) * cos(theta) * (-1*az) - cos(psi) * cos(
                       theta) * sin(
                       phi) * (-1*ay), (cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) * (-1*ay) - (
                           cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) * (-1*az) + cos(theta) * sin(psi) * (
                       -1*ax), 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [(cos(psi) * sin(phi) - cos(phi) * sin(psi) * sin(theta)) * (-1*ay) + (
                          cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta)) * (-1*az),
                   sin(psi) * sin(theta) * (-1*ax) - cos(phi) * cos(theta) * sin(psi) * (-1*az) - cos(theta) * sin(
                       phi) * sin(
                       psi) * (-1*ay), (cos(phi) * sin(psi) - cos(psi) * sin(phi) * sin(theta)) * (-1*ay) - (
                           sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)) * (-1*az) - cos(psi) * cos(theta) * (
                       -1*ax), 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [cos(theta) * sin(phi) * (-1*az) - cos(phi) * cos(theta) * (-1*ay),
                   cos(theta) * (-1*ax) + cos(phi) * sin(theta) * (-1*az) + sin(phi) * sin(theta) * (-1*ay), 0, 0, 0, 0,
                   0, 0, 0,
                   0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
    return F


def H_Find_INS(xh, sn):
    # Using Matlab's Partial Differentation.
    # z_hat = [ax,ay,az, psi , x, y, z, V_n, V_e, V_d, p, q, r]
    # x_hat = [ phi, theta, psi, x, y, z, V_n, V_e, V_d, p, q, r]
    # Sensor Data sn = [ax,ay,az,wx,wy,wz]
    # All Angles in RADIANS!
    phi, theta, psi, x, y, z, V_n, V_e, V_d, p, q, r = xh
    ax,ay,az,wx,wy,wz = sn
    g = 9.808

    H = np.array([[0, g * cos(theta), 0, 0, 0, 0, 0, 0, 0, 0, V_d, V_e],
                  [-g * cos(phi) * cos(theta), g * sin(phi) * sin(theta), 0, 0, 0, 0, 0, 0, 0, -V_d, 0, -V_n],
                  [g * cos(theta) * sin(phi), g * cos(phi) * sin(theta), 0, 0, 0, 0, 0, 0, 0, V_e, V_n, 0],
                  [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
    return H


def H_Find_AHRS(xh, sn):
    # Using Matlab's Partial Differentation.
    # z_hat = [ax,ay,az,psi,p,q,r]
    # x_hat = [ phi, theta, psi, x, y, z, V_n, V_e, V_d, p, q, r]
    # Sensor Data sn = [ax,ay,az,wx,wy,wz]
    # All Angles in RADIANS!
    phi, theta, psi, x, y, z, V_n, V_e, V_d, p, q, r = xh
    ax,ay,az,wx,wy,wz = sn
    g = 9.808

    H = np.array( [[0, g * cos(theta), 0, 0, 0, 0, 0, 0, 0, 0, V_d, V_e],
                  [-1*g * cos(phi) * cos(theta), g*sin(phi)*sin(theta), 0, 0, 0, 0, 0, 0, 0, -V_d, 0, -V_n],
                  [g*cos(theta)*sin(phi), g*cos(phi)*sin(theta), 0, 0, 0, 0, 0, 0, 0, V_e, V_n, 0],
                  [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
    return H


def priori(xh, P, F, Q):
    # do not forget to initialize xh and P.
    import numpy as np
    FT = np.transpose(F)
    Pminus = np.matmul(np.matmul(F, P), FT)
    xhatminus = np.matmul(F, xh)
    return xhatminus, Pminus


def posteriori(xhatminus, Pminus, zn, H, R):
    ss = len(xhatminus)  # state space size
    HT = numpy.transpose(H)
    # calculate Kalman gain
    Knumerator = numpy.matmul(Pminus, HT)
    Kdenominator = numpy.matmul(numpy.matmul(H, Pminus), HT) + R
    K = numpy.matmul(Knumerator, np.linalg.inv(Kdenominator))  # Kalman gain
    residuals = zn - numpy.matmul(H, xhatminus)
    xhat = xhatminus + numpy.matmul(K, residuals)
    one_minus_KC = numpy.eye(ss) - numpy.matmul(K, H)

    # compute a posteriori estimate of errors
    P = numpy.matmul(one_minus_KC, Pminus)

    return xhat, P


if __name__ == "__main__":

    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600, source_system=255)

    # initialize arrays for sharing sensor data.
    y = multiprocessing.Array('d', np.zeros(26)) # imu, baro, gps, adc
    xh = multiprocessing.Array('d', np.zeros(12)) # position, orientation, rates
    servo = multiprocessing.Array('d', np.zeros(13)) # mode_flag, rcin, servo_out
    cmd = multiprocessing.Array('d', np.zeros(2)) # psi_c, h_c

    # start processes for interpreting sensor data and setting servo pwm.
    estimator_process = multiprocessing.Process(target=estimator_loop, args=(y,xh,servo))
    estimator_process.daemon = True
    estimator_process.start()
    controller_process = multiprocessing.Process(target=controller_loop, args=(xh,servo,cmd))
    controller_process.daemon = True
    controller_process.start()
    servo_process = multiprocessing.Process(target=air.servo_loop, args=(servo,))
    servo_process.daemon = True
    servo_process.start()
    time.sleep(3)
    # start process for telemetry after other processes have initialized.
    telemetry_process = multiprocessing.Process(target=air.telemetry_loop, args=(y,xh,servo,master))
    telemetry_process.daemon = True
    telemetry_process.start()

    print("\nsending heartbeats to {} at 1hz.".format('/dev/ttyAMA0'))
    # loop for sending heartbeats and receiving messages from gcs.
    while True:
        # send heartbeat message periodically
        master.mav.heartbeat_send(1, 0, 0, 0, 4, 0)
        # still haven't figured out how to get mode to show up in mission planner.
        # print('heartbeat sent.')
        time.sleep(0.5)
	#=====WAYPOINT TRACKER STARTS HERE======================
	#Simple waypoint tracker
        #
	#=====WAYPOINT TRACKER STOPS HERE=======================

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
