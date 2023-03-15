import re

from bitsandstuff import int_converter
import statistics
import time


def default_movement(error, last_error, current_sensor_position, target_sensor_position, pid_controller, Move_node):
    bits_block, indices = int_converter(current_sensor_position)
    fork_in_road(bits_block, Move_node)

    if len(indices) != 0:
        last_error = error
        error = target_sensor_position - statistics.mean(indices)
        pid_controller.apply_controller(Move_node, error, last_error)

    else:
        Move_node.speed_left_wheel = 0
        Move_node.speed_right_wheel = 0

def fork_in_road(binary, Move_node):
    print("binary value is: ", binary)
    m = re.search('^1+1?0+0?1+1?0+$', binary)
    if m:
        if Move_node.fork_in_road == False:
            Move_node.fork_in_road = True
            print("match", m[0])
        else:
            Move_node.fork_in_road_conf = True
    else:
        print("no can do")