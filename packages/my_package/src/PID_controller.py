#Node, mis saab jooneanduri data, toidab selle PID kontrollerisse ja väljastab juhised, mille järgi robot sõidab.
from smbus2 import SMBus
import rospy
#!/usr/bin/env python3
    
def apply_controller(t0, t1):
    delta_t = 1
    bus = SMBus(1)
    read = bin(bus.read_byte_data(62, 17))[2:].zfill(8)
    
    #arvutab theta refi keskpunkti välja(otse on 4.5)
    line_values = []
    for i, value in enumerate(read):
        if value =='1':
            line_values.append(i + 1)
    if len(line_values) >= 1:
        theta_hat = sum(line_values)/len(line_values)
    if len(line_values) == 0:
        theta_hat = 4
    print("theta_hat is: ", theta_hat)


    pose_estimation = 4.5
    prev_int = 0

    e = pose_estimation - theta_hat 
    e_int = prev_int + e*delta_t
    prev_int = e_int                                        #integral of the error
    prev_e = e                                              #Tracking
    e_int = max(min(e_int,2),-2)                            # anti-windup - preventing the integral error from growing too much       
    e_der = (e - prev_e)/delta_t                       #derivative of the error
    print("prev_e is: ", prev_e)
    
    Kp = float(rospy.get_param("/p"))#0.065
    Ki = float(rospy.get_param("/i"))#0.02
    Kd = float(rospy.get_param("/d"))#0.6
    
    delta_t = t0-t1
    omega = Kp*e + Ki*e_int + Kd*e_der                 #PID controller for omega
    return omega