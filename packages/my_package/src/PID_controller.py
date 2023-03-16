#Node, mis saab jooneanduri data, toidab selle PID kontrollerisse ja väljastab juhised, mille järgi robot sõidab.
from Movement import Move_Node
#!/usr/bin/env python3

class PID_Controller():
    def __init__(self, Kp, Ki, Kd, I, rospy_rate):
        self.Kp = Kp
        print("Current Kp value is: ", self.Kp)
        self.Ki = Ki
        self.Kd = Kd
        self.I = I
        self.delta_t = rospy_rate
    
    def apply_controller(self, Move_node, err, last_error):
        if err == 0:
            Move_Node.move_forward()
        else:
            P = err
            self.I = (self.I + err) * self.delta_t
            self.I = max(min(self.I, 0.2) -0.2) 
            D = (err - last_error) / self.delta_t
            PID = (self.Kp * P) + (self.Ki * self.I) + (self.Kd * D)
            print("PID value is: ", PID)

            Move_Node.speed_right_wheel = Move_Node.velocity + PID
            Move_Node.speed_left_wheel = Move_Node.velocity - PID
            