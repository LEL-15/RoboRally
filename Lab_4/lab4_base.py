import rospy
import json
import copy
import time
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Empty, String, Int16


# GLOBALS 
pose2d_sparki_odometry = None #Pose2D message object, contains x,y,theta members in meters and radians
#TODO: Track servo angle in radians
#TODO: Track IR sensor readings (there are five readings in the array: we've been using indices 1,2,3 for left/center/right)
#TODO: Create data structure to hold map representation

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
    global theta, value_array, ping_distance

    #TODO: Init your node to register it with the ROS core
    init()

    while not rospy.is_shutdown():
        #TODO: Implement CYCLE TIME
        start_time = int(round(time.time() * 1000))
        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        
        #Reset point
        if ((lineCenter < threshold) && (lineLeft < threshold) && (lineRight < threshold) ):
            rospy.loginfo("Loop Closure Triggered")
        #Spin left
        else if ( lineLeft < threshold ):
            sparki.moveLeft(); // turn left
        #spin right
        elsif( lineRight < threshold ):
            sparki.moveRight(); // turn right
            movement = -1;

        #if the center line sensor is the only one reading a line
        else:
            sparki.moveForward(); // move forward
            movement = 0;

        #TODO: Implement CYCLE TIME
        end_time = int(round(time.time() * 1000));
        if(end_time - start_time < 50):
            rospy.sleep((50 - (start_time - end_time)) / 1000);
        sparki.moveStop();
        msg = Empty()
        publisher_ping.publish(msg)



def init():
    global publisher_motor, publisher_ping, publisher_servo, publisher_odom
    global subscriber_odometry, subscriber_state
    global pose2d_sparki_odometry
    global theta, value_array, ping_distance
    publisher_motor = rospy.Publisher('/sparki/motor_command', 'std_msgs/Float32MultiArray')
    publisher_ping = rospy.Publisher('/sparki/ping_command', 'std_msgs/Empty')
    publisher_servo = rospy.Publisher('/sparki/set_servo', 'std_msgs/Int16')
    publisher_odom = rospy.Publisher('/sparki/set_odometry', 'geometry_msgs/Pose2D')
    subscriber_odometry = rospy.Subscriber('/sparki/odometry', 'geometry_msgs/Pose2D', callback_update_odometry)
    subscriber_state = rospy.Subscriber('/sparki/state', 'std_msgs/String', callback_update_state)

    msg = Int16()
    msg.data = -45
    publisher_servo.publish(msg)

    pose2d_sparki_odometry = Pose2D()
    #TODO: Set up your publishers and subscribers
    #TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    #TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 45)

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry 
    pose2d_sparki_odometry = data.data
    #TODO: Copy this data into your local odometry variable

def callback_update_state(data):
    global theta, value_array, ping_distance
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #TODO: Load data into your program's local state variables
    theta = state_dict["servo"]
    value_array = state_dict["light_sensors"]
    try:
        ping_distance = state_dict["ping_distance"]
    except:
        ping_distance = None

def convert_ultrasonic_to_robot_coords(x_us):
    #TODO: Using US sensor reading and servo angle, return value in robot-centric coordinates
    x_r, y_r = 0., 0.
    return x_r, y_r

def convert_robot_coords_to_world(x_r, y_r):
    #TODO: Using odometry, convert robot-centric coordinates into world coordinates
    x_w, y_w = 0., 0.

    return x_w, y_w

def populate_map_from_ping(x_ping, y_ping):
    #TODO: Given world coordinates of an object detected via ping, fill in the corresponding part of the map
    pass

def display_map():
    #TODO: Display the map
    pass

def ij_to_cell_index(i,j):
    #TODO: Convert from i,j coordinates to a single integer that identifies a grid cell
    return 0

def cell_index_to_ij(cell_index):
    #TODO: Convert from cell_index to (i,j) coordinates
    return 0, 0


def cost(cell_index_from, cell_index_to):
    #TODO: Return cost of traversing from one cell to another
    return 0

if __name__ == "__main__":
    main()


