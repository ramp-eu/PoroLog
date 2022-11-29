#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup  # Allow callbacks to be executed in parallel without restriction
from rclpy.executors import MultiThreadedExecutor # Runs callbacks in a pool of threads

from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile
#from basic_mobile_robot.module_to_import import RobotInfo

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path

from commlib.node import Node as nd, TransportType
from commlib.transports.mqtt import ConnectionParameters, Credentials

import time
import math
import base64
import json
import requests
import numpy as np

class RobotInfo(Node):

    def __init__(self, robot_base_frame = "base_link"):
        super().__init__('robot_info')
        rclpy.logging.set_logger_level('robot_info',rclpy.logging.LoggingSeverity.DEBUG)
        self.group = ReentrantCallbackGroup()

        MOSQUITTO_HOST = "83.212.106.23"    # This is okeanos
        MOSQUITTO_PORT = 1893
        MOSQUITTO_USERNAME = "porolog"
        MOSQUITTO_PASSWORD = "fiware"

        API_KEY = "1234abcd"
        ENTITY_TYPE = "Robot"
        ENTITY_ID = "1"
        BASE_TOPIC = f"/{API_KEY}/{ENTITY_TYPE}{ENTITY_ID}/attrs"


        creds = Credentials(
            username=MOSQUITTO_USERNAME,
            password=MOSQUITTO_PASSWORD
        )

        params = ConnectionParameters(
            host=MOSQUITTO_HOST,
            port=MOSQUITTO_PORT,
            creds=creds
        )

        node = nd(
            transport_type=TransportType.MQTT,
            connection_params=params,
            debug=True
        )

        self.battery_level = 100

        self.robot_state = 'IDLE'
        self.robot_base_frame = robot_base_frame
        self.parcel = 0
        self.dist = 0
        timer_period1 = 5
        timer_period2 = 10
        #self.logs = self.create_timer(timer_period1, self.timer_callback_logs, callback_group=self.group) #log
        self.status = self.create_timer(timer_period2, self.timer_callback_status, callback_group=self.group) #status
        self.battery = self.create_timer(timer_period1, self.timer_callback_battery, callback_group=self.group) #battery
        self.state = self.create_timer(timer_period1, self.tele_cb, callback_group=self.group) #state
        self.image_ = self.create_timer(timer_period2, self.image_cb, callback_group=self.group) #image

        self.logs_pub = node.create_publisher(topic=BASE_TOPIC + "/logs")
        self.get_logger().warning("Logs publisher READY!")
    
        self.battery_pub = node.create_publisher(topic=BASE_TOPIC + "/battery")
        self.get_logger().warning("Battery level publisher READY!")
  
        self.status_pub = node.create_publisher(topic=BASE_TOPIC + "/status")
        self.get_logger().warning("Status publisher READY!")

        self.velocities_ros_sub = self.create_subscription(Twist,'/cmd_vel', self.velocities_cb, 10, callback_group=self.group)
        self.velocities_pub = node.create_publisher(topic=BASE_TOPIC + "/velocities")  
        self.get_logger().warning("Velocities publisher READY!")   

        self.tele_sub = node.create_subscriber(topic=BASE_TOPIC + "/tele_mode", on_message=self.tele_cb)
        self.tele_sub.run()
        self.state_pub = node.create_publisher(topic=BASE_TOPIC + "/state")
        self.get_logger().warning("State publisher READY!")

        self.pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability = ReliabilityPolicy.BEST_EFFORT), callback_group=self.group)
        self.pose_publisher = node.create_publisher(topic=BASE_TOPIC + "/pose")
        self.get_logger().warning("Robot pose publisher READY!")

        self.plan_subscriber = self.create_subscription(Path, '/plan', self.plan_cb, QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE, reliability = ReliabilityPolicy.BEST_EFFORT), callback_group=self.group)
        self.plan_publisher = node.create_publisher(topic=BASE_TOPIC + "/path")
        self.get_logger().warning("Robot path publisher READY!")

        self.teleop_sub = node.create_subscriber(topic=BASE_TOPIC + "/teleop/commands", on_message=self.move_cb)
        self.teleop_sub.run()
        self.teleop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().warning("Teleoperation READY!")

        self.go_to_sub = node.create_subscriber(topic=BASE_TOPIC + "/target", on_message=self.goal_cb)
        self.go_to_sub.run()
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().warning("Go to goal READY!")

        self.image_publisher = node.create_publisher(topic=BASE_TOPIC + "/image")
        self.get_logger().warning("Image publisher READY!")


        self.pallet_sub = node.create_subscriber(topic=BASE_TOPIC + "/goal/entity" , on_message = self.pallet_cb)
        self.pallet_sub.run()

        self.parcel_move_publisher = node.create_publisher(topic="/1234abcd/WarehouseKPI1/attrs/parcelsMoved")
    
        self.dist_move_publisher = node.create_publisher(topic="/1234abcd/WarehouseKPI1/attrs/distanceXmassMovedByRobots")

        self.state_pub.publish({"val" : "IDLE"})
        self.logs_pub.publish({"val": "Robot is idle"})

        # self.cancel_goal_sub = node.create_subscriber(topic=BASE_TOPIC + "/target/cancel", on_message=self.cancel_cb)
        # self.cancel_goal_sub.run()

############################################################################
    #logs
    # def timer_callback_logs(self):
    #     self.logs_pub.publish({"val": "I am a log1"})
    
    def euler_from_quaternion(self,quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def pallet_cb(self, msg):
        self.logs_pub.publish({"val": "Robot goes to " + msg["id"] + " of type " + msg["type"]})
        self.parcel += 1
        self.dist += 50
        self.parcel_move_publisher.publish({"val": self.parcel})
        self.dist_move_publisher.publish({"val": self.dist})
        self.state_pub.publish({"val" : "NAV"})

        self.go_to = PoseStamped()
        self.go_to.header.frame_id = "map"
 
        self.x_ = 5.1
        self.y_ = -2.1

        self.go_to.pose.position.x = self.x_
        self.go_to.pose.position.y = self.y_
        self.go_to.pose.orientation.w = 1.0

        self.goal_pub.publish(self.go_to)

    #status
    def timer_callback_status(self):
        self.status_pub.publish({"status" : "Active"})
    
    #velocities
    def velocities_cb(self, msg):
        self.x = round(msg.linear.x,2)
        self.z = round(msg.angular.z,2)
        self.velocities_pub.publish({"linear" : self.x, "angular" : self.z})
        self.battery_level -= 0.1
    
    #battery level
    def timer_callback_battery(self):
        self.battery_pub.publish({"percentage": round(self.battery_level,2)})
    
    # #state
    def tele_cb(self,msg='IDLE'):
        self.robot_state = str(msg['command'])

        if self.robot_state == 'TELE':
            self.state_pub.publish({"val" : "TELE"})
            self.logs_pub.publish({"val": "Robot state is teleoperation"})
        else:
            self.state_pub.publish({"val" : "IDLE"})
            self.logs_pub.publish({"val": "Robot is idle"})
    
    #robot pose
    def pose_cb(self,msg):
        self.robot_pose = []
        self.amcl_pose = msg
        self.robot_pose = round(self.amcl_pose.pose.pose.position.x,2),round(self.amcl_pose.pose.pose.position.y,2), round(self.amcl_pose.pose.pose.orientation.w,2)

        r,p,y = self.euler_from_quaternion(self.amcl_pose.pose.pose.orientation)

        self.pose_publisher.publish({"x": round(self.amcl_pose.pose.pose.position.x,2), "y" : round(self.amcl_pose.pose.pose.position.y,2), "th":round(y,2)})
    
    #robot path
    def plan_cb(self,msg):
        self.robot_path = []
        self.plan_path = msg

        self.variable_i = len(self.plan_path.poses)
        for i in range(self.variable_i):
            self.robot_path.append(((round(self.plan_path.poses[i].pose.position.x,2)),(round(self.plan_path.poses[i].pose.position.y,2))))
        # Nav mode --> Autonomous mode with target
        self.plan_publisher.publish({"points" : self.robot_path})
        self.state_pub.publish({"val" : "NAV"})
    
    #teleoperation commands
    def move_cb(self,msg):
        self.max_time = 1
        # if self.robot_state == 'IDLE':
        #     return
        # else:
        self.move = Twist()
        self.stop = Twist()
        self.stop.linear.x = 0.0
        self.data = str(msg['command'])
        # self.logs_pub.publish({"val": "Teleoperation mode"})
            # run for 1 sec
        if self.data == 'FORWARD':
            self.move.linear.x = 1.0
            self.state_pub.publish({"val" : "TELE"})
            self.logs_pub.publish({"val": "Moving forward"})
        if self.data == 'BACKWARD':
            self.move.linear.x = -1.0
            self.state_pub.publish({"val" : "TELE"})
            self.logs_pub.publish({"val": "Moving backward"})
        if self.data == 'STOP':
            self.move.linear.x = 0.0
            self.state_pub.publish({"val" : "TELE"})
            self.logs_pub.publish({"val": "Stopping"})
        if self.data == 'RIGHT':
            self.move.angular.z = -1.0
            self.state_pub.publish({"val" : "TELE"})
            self.logs_pub.publish({"val": "Rotating right"})
        if self.data == 'LEFT':
            self.move.angular.z = 1.0
            self.state_pub.publish({"val" : "TELE"})
            self.logs_pub.publish({"val": "Rotating left"})
        if self.data == 'LIFT':
            self.state_pub.publish({"val" : "LIFT"})
            self.logs_pub.publish({"val": "Lifting"})
        if self.data == 'RELEASE':
            self.state_pub.publish({"val" : "RELEASE"})
            self.logs_pub.publish({"val": "Releasing"})

        self.delay = 1
        self.close_time = time.time()+self.delay
        while self.close_time >time.time():
            self.teleop_pub.publish(self.move)
        self.teleop_pub.publish(self.stop)
    
    #send goal position
    def goal_cb(self,msg):
        self.go_to = PoseStamped()
        self.go_to.header.frame_id = "map"
    
        # self.x_ = msg['x'][0]
        # self.y_ = msg['y'][1]
        self.x_ = msg['x']
        self.y_ = msg['y']

        self.go_to.pose.position.x = self.x_
        self.go_to.pose.position.y = self.y_
        self.go_to.pose.orientation.w = 1.0

        self.goal_pub.publish(self.go_to)
        self.logs_pub.publish({"val": "Entering navigation mode"})
        self.state_pub.publish({"val" : "NAV"})

    # def cancel_cb(self,msg):
    #     self.data = str(msg['command'])
    #     if self.data == "CANCEL":
    #         self.go_to.pose.position.x = 0.0
    #         self.go_to.pose.position.y = 0.0
    #         self.goal_pub.publish(self.go_to)



    def getFiware(ft, fid):
        response = requests.request(
            "GET", 
            f"{fiware_url}?id={fid}", 
            headers = fiware_headers, params = {"type":ft,"options":"keyValues"})
        return json.loads(response.text)

    # Get parcel or pallet location
    def getParcelID(self):
        self.parcel_x = 0
        self.parcel_y = 0
        self.parcel_z = 0

        self.ftype = "Parcel"
        self.ent_id = "urn:ngsi-ld:Parcel:1"   # message
        self.rr = self.getFiware(self.ftype, self.ent_id)

        self.ftype = "Pallet"               # for palets
        self.ent_id = self.rr[0]["refPallet"]
        self.rr = self.getFiware(self.ftype, self.ent_id)

        # Find the Slot
        self.ftype = "Slot"
        self.ent_id = self.rr[0]["refSlot"]
        self.rr = self.getFiware(self.ftype, self.ent_id)

        # Update parcel coords
        self.parcel_x += self.rr[0]['originx']
        #print("Parcel coordinates thus far: ", parcel_x, parcel_y, parcel_z)

        # Find the Shelf
        self.ftype = "Shelf"
        self.ent_id = self.rr[0]["refShelf"]
        self.rr = self.getFiware(self.ftype, self.ent_id)
        
        # Update parcel coords
        self.parcel_z += self.rr[0]['altitude']
        #print("Parcel coordinates thus far: ", parcel_x, parcel_y, parcel_z)

        # Find the Rack
        self.ftype = "Rack"
        self.ent_id = self.rr[0]["refRack"]
        self.rr = self.getFiware(self.ftype, self.ent_id)
        
        self.rack_orientation = self.rr[0]['dimensions']['orientation']
        self.rack_origin_x = self.rr[0]['origin']['x']
        self.rack_origin_y = self.rr[0]['origin']['y']
        self.parcel_x = math.cos(self.rack_orientation) * self.parcel_x + self.rack_origin_x
        self.parcel_y = math.sin(self.rack_orientation) * self.parcel_x + self.rack_origin_y
        #print("Parcel coordinates thus far: ", parcel_x, parcel_y, parcel_z)

        # Find the Room
        self.ftype = "Room"
        self.ent_id = self.rr[0]["refRoom"]
        self.rr = self.getFiware(self.ftype, self.ent_id)
        
        self.room_origin_x = self.rr[0]['origin']['x']
        self.room_origin_y = self.rr[0]['origin']['y']
        self.parcel_x += self.room_origin_x
        self.parcel_y += self.room_origin_y
        #print("Parcel coordinates thus far: ", parcel_x, parcel_y, parcel_z)
        #print("Final parcel coordinates (in meters): ", parcel_x, parcel_y, parcel_z)
        self.get_logger().info('Parcel coordinates in meters: parcel_x %d parcel_y %d parcel_z %d' %(self.parcel_x, self.parcel_y, self.parcel_z))


        # Find the Warehouse
        self.ftype = "Warehouse"
        self.ent_id = self.rr[0]["refWarehouse"]
        self.rr = self.getFiware(self.ftype, self.ent_id)
        
        self.resolution = self.rr[0]['dimensions']['resolution']
        #print("Final parcel coordinates (in pixels): ", parcel_x/resolution, parcel_y/resolution)
        self.get_logger().info('Parcel coordinates in pixels: parcel_x %d parcel_y %d parcel_z %d' %((self.parcel_x/self.resolution), (self.parcel_y/self.resolution), (self.parcel_z/self.resolution)))

        self.get_parcel = PoseStamped()
        self.go_to.header.frame_id = "map"

        self.get_parcel.pose.position.x = self.parcel_x
        self.get_parcel.pose.position.y = self.parcel_y
        self.get_parcel.pose.position.z = self.parcel_z
        self.get_parcel.pose.orientation.w = 1.0

        self.goal_pub.publish(self.get_parcel)        

    # Send robot's image
    def image_cb(self):
        self.f = open("/home/vas/dev_ws/src/basic_mobile_robot/scripts/warehouse_1.png", "rb")
        self.fileContent = self.f.read()
        self.byteArr = bytearray(self.fileContent)
        self.image = base64.encodebytes(self.fileContent).decode("utf-8")

        self.image_publisher.publish({
            "val":self.image
        })

def main(args=None):
    rclpy.init(args=args)
    try:
        robot_info = RobotInfo()
        executor = MultiThreadedExecutor(num_threads=12)
        executor.add_node(robot_info)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            robot_info.destroy_node()
    finally:
        rclpy.shutdown()
###---------------------------------------------

if __name__ == '__main__':
    main()
