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

servo = None
value_array = None
ping_distance = None

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
        #TODO: Implement CYCLE TIME
        #start_time = int(round(time.time() * 1000))
        rate = rospy.Rate(1.0/CYCLE_TIME)
        motor_message = Float32MultiArray()
        start_time = time.time()
        #TODO: Implement line following code here
        #      To create a message for changing motor speed, use Float32MultiArray()
        #      (e.g., msg = Float32MultiArray()     msg.data = [1.0,1.0]      publisher.pub(msg))
        
        #Reset point
        lineLeft = value_array[1]
        lineCenter = value_array[2]
        lineRight = value_array[3]
        if ((lineCenter < IR_THRESHOLD) and (lineLeft < IR_THRESHOLD) and (lineRight < IR_THRESHOLD) ):
            rospy.loginfo("Loop Closure Triggered")
        #Spin left
        if lineCenter < IR_THRESHOLD:
            motor_message.data = [1.0,1.0]
        elif( lineLeft < IR_THRESHOLD):
            motor_message.data = [-1.0, 1.0]
        #spin right
        elif( lineRight < IR_THRESHOLD):
            motor_message.data = [1.0, -1.0]

        #if the center line sensor is the only one reading a line
        #else:
            #motor_message.data = [1.0, 1.0]
        publisher_motor.publish(motor_message)
        rate.sleep()
        #TODO: Implement CYCLE TIME
        #end_time = int(round(time.time() * 1000));
        """
        end_time = time.time()
        if(end_time - start_time < 50):
            #rospy.sleep((50 - (end_time - start_time))/1000);
            rospy.sleep(50 - end_time - start_time)
        """
        #sparki.moveStop();
        #motor_message.data = [0.0,0.0]
        #publisher_motor.publish(motor_message)
        msg = Empty()
        publisher_ping.publish(msg)
        publisher_render.publish(Empty())



def init():
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



    msg = Int16(80)
    publisher_servo.publish(msg)

    pose2d_sparki_odometry = Pose2D()
    #TODO: Set up your publishers and subscribers
    #TODO: Set up your initial odometry pose (pose2d_sparki_odometry) as a new Pose2D message object
    #TODO: Set sparki's servo to an angle pointing inward to the map (e.g., 45)

def callback_update_odometry(data):
    # Receives geometry_msgs/Pose2D message
    global pose2d_sparki_odometry 
    pose2d_sparki_odometry.x = data.x
    pose2d_sparki_odometry.y = data.y
    pose2d_sparki_odometry.theta = data.theta
    #TODO: Copy this data into your local odometry variable

def callback_update_state(data):
    global servo, value_array, ping_distance
    state_dict = json.loads(data.data) # Creates a dictionary object from the JSON string received from the state topic
    #TODO: Load data into your program's local state variables
    servo = state_dict["servo"]
    value_array = state_dict["light_sensors"]
    if('ping' in state_dict and state_dict['ping'] != -1):
        ping_distance = state_dict["ping"]

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


