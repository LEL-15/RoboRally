import rospy
import json
import copy
import time
import math;
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


# GLOBALS 
#Pose2D message object, contains x,y,theta members in meters and radians
pose2d_sparki_odometry = None 
#TODO: Track servo angle in radians
servo = None
#DONE: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
value_array = None
ping_distance = None
#ATTEMPTED: Create data structure to hold map representation
#Actually 60x42 centimeters, so here each cell is 3x3 centimeters (20*14)
world_map = [280];

# TODO: Use these variables to hold your publishers and subscribers
publisher_motor = None
publisher_odom = None
publisher_ping = None
publisher_servo = None
subscriber_odometry = None
subscriber_state = None

# CONSTANTS 
IR_THRESHOLD = 300 # IR sensor threshold for detecting black track. Change as necessary.
CYCLE_TIME = 0.1 # In seconds

def main():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global IR_THRESHOLD, CYCLE_TIME
    global pose2d_sparki_odometry
    global servo, value_array, ping_distance
    value_array = [1000, 1000, 0, 1000, 1000]
    #TODO: Init your node to register it with the ROS core

    init()
    publisher_servo.publish(Int16(80))
    publisher_render = rospy.Publisher('/sparki/render_sim', Empty, queue_size=10)
    while not rospy.is_shutdown():
        publisher_servo.publish(Int16(80))
        rate = rospy.Rate(1.0/CYCLE_TIME)
        motor_message = Float32MultiArray()
        #DONE: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        lineLeft = value_array[1]
        lineCenter = value_array[2]
        lineRight = value_array[3]
        #Reset point
        if ((lineCenter < IR_THRESHOLD) and (lineLeft < IR_THRESHOLD) and (lineRight < IR_THRESHOLD) ):
            rospy.loginfo("Loop Closure Triggered")
        #Go forward
        if lineCenter < IR_THRESHOLD:
            motor_message.data = [1.0,1.0]
        #Turn right
        elif( lineLeft < IR_THRESHOLD):
            motor_message.data = [-1.0, 1.0]
        #Turn Left
        elif( lineRight < IR_THRESHOLD):
            motor_message.data = [1.0, -1.0]
        #DONE: Implement CYCLE TIME
        publisher_motor.publish(motor_message)
        rate.sleep()
        #Send ping so that sensor returns
        msg = Empty()
        publisher_ping.publish(msg)
        #Send message to knows to render again
        publisher_render.publish(Empty())
        #TODO: Display map



def init():
    #DONE: Set up your publishers and subscribers
    rospy.init_node("main", anonymous=True)
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    global theta, value_array, ping_distance
    publisher_motor = rospy.Publisher('/sparki/motor_command', Float32MultiArray, queue_size=10)
    publisher_ping = rospy.Publisher('/sparki/ping_command', Empty, queue_size=10)
    publisher_servo = rospy.Publisher('/sparki/set_servo', Int16, queue_size=10)
    publisher_odom = rospy.Publisher('/sparki/set_odometry', Pose2D, queue_size=10)
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', Pose2D, callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', String, callback_update_state)


    #DONE: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    pose2d_sparki_odometry = Pose2D()
    #DONE: Set sparki's servo to an angle pointing inward to the map (e.g., 45)
    msg = Int16(80)
    publisher_servo.publish(msg)
    #ATTEMPTED: Set map values to all be empty
    for i in range(280):
        world_map[i] = False;

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    #DONE: Copy this data into your local odometry variable
    global pose2d_sparki_odometry 
    print(data)
    pose2d_sparki_odometry.x = data.x
    pose2d_sparki_odometry.y = data.y
    pose2d_sparki_odometry.theta = data.theta
    #print(pose2d_sparki_odometry);
    

def callback_update_state(data):
    global servo, value_array, ping_distance
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #DONE: Load data into your program's local state variables
    servo = state_dict["servo"]
    value_array = state_dict["light_sensors"]
    if('ping' in state_dict and state_dict['ping'] != -1):
        ping_distance = state_dict["ping"]

def convert_ultrasonic_to_robot_coords(x_us):
    #ATTEMPTED: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = 0., x_us
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #ATTEMPTED: Using odometry, convert robot-centric coordinates into world coordinates
    x_w  = pose2d_sparki_odometry.x + sin(pose2d_sparki_odometry.theta) * x_r;
    y_w = pose2d_sparki_odometry.x + cos(pose2d_sparki_odometry.theta) * y_r;
    0., 0.

    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #ATTEMPTED: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    cell = ij_to_cell_index(x_ping, y_ping)
    world_map[cell] = True
    return

def display_map():
    #TODO: Display the map
    for i in range(world_map.length()):
        if (world_map[i]):
            print("1")
        else:
            print("0")
        if (i%20 == 19):
            print("\n")

def ij_to_cell_index(i,j):
    #ATTEMPTED: Convert from i,j coordinates to a single integer that identifies a grid cell
    xCell = math.floor(i/3)
    yCell = math.floor(j/3)
    return (xCell + yCell*20)

def cell_index_to_ij(cell_index):
    #ATTEMPTED: Convert from cell_index to (i,j) coordinates
    column = cell_index % 20
    row = floor(cell_index /20)
    return column*3, row*3

def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    return 0

if __name__ == "__main__":
    main()


