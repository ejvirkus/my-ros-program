#Fail, mis määrab ära roboti liikumise

class Move_Node():
    def __init__(self, velocity): #
        self.velocity = velocity
        self.speed_right_wheel = velocity
        self.speed_left_wheel = velocity
        self.fork_in_road = False
        self.fork_in_road_conf = False
        self.fork_go_left = False
        self.fork_go_right = False

    def move_stop(self):
        print("Stopping")
        self.speed_left_wheel = 0
        self.speed_right_wheel = 0
    
    def move_turn_left(self):
        print("Turning left")
        self.speed_left_wheel = 0.1 * self.velocity
        self.speed_right_wheel = self.velocity

    def move_turn_right(self):
        print("Turning right")
        self.speed_left_wheel = self.velocity
        self.speed_right_wheel = 0.1 * self.velocity

    def move_turn_sharp_right(self):
        print("Turning sharp right")
        self.speed_left_wheel = self.velocity * 0.75
        self.speed_right_wheel = 0

    def move_turn_sharp_left(self):
        print("Turning sharp left")
        self.speed_left_wheel = 0
        self.speed_right_wheel = 0.75 * self.velocity

    def move_forward(self):
        print("Driving forward")
        self.speed_left_wheel = self.velocity 
        self.speed_right_wheel = self.velocity