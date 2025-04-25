# Impot Packages
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import Ros Messages
from geometry_msgs.msg import Point, PointStamped
from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import VehicleOdometry
from vicon_receiver.msg import Position

class Translator(Node):

    def __init__(self):
        super().__init__('translator')
        
        # Create QoS policy for PX4 publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Intialize varaibles
        self.timesync = 0
                        
        # Create publishers and subscribers
        
        # Initialize subscriber to the Vicon motion capture topic
        self.mocap_sub = self.create_subscription(Position, 
            '/vicon/Turner_X500/Turner_X500', self.mocap_callback, 10)
        
        # Initilize publisher for motion capture data on /position topic
        self.position_pub = self.create_publisher(PointStamped, 
            '/position', 10)

        # Initialize subscriber to PX4 timesync topic
        self.timesync_sub = self.create_subscription(TimesyncStatus, 
            "/fmu/out/timesync_status", self.timesync_callback, qos_profile)

        # Initialize publisher to PX4 vehicle_visual_odometry topic
        self.vehicle_odometry_pub = self.create_publisher(VehicleOdometry, 
            '/fmu/in/vehicle_visual_odometry', qos_profile)
        
        self.get_logger().info("Publishing motion capture data to PX4.")
       
    # Vicon motion capture callback function. Publishes mocap data to PX4. 
    def mocap_callback(self, msg):
        # Get position data from Vicon message (ENU coordinates)
        x_pos = msg.x_trans/1000.0
        y_pos = msg.y_trans/1000.0
        z_pos = msg.z_trans/1000.0
        
        enu_coordinates = [x_pos, y_pos, z_pos]
        frd_coordinates = [enu_coordinates[1], enu_coordinates[0], -enu_coordinates[2]]
        
        # Create point message from Vicon position data (NED coordinates)
        position = PointStamped()
        position.header.stamp.sec = self.timesync
        position.point.x = frd_coordinates[0]
        position.point.y = frd_coordinates[1]
        position.point.z = frd_coordinates[2]
        self.position_pub.publish(position)

        # Create PX4 message from Vicon position data
        msg_px4 = VehicleOdometry() # Message to be sent to PX4
        msg_px4.timestamp = self.timesync # Set timestamp
        msg_px4.timestamp_sample = self.timesync # Timestamp for mocap sample
        msg_px4.pose_frame = 2 # FRD from px4 message
        msg_px4.position = frd_coordinates
        msg_px4.position_variance = [10**(-6), 10**(-6), 10**(-6)] # Assuming 1mmm standard deviation in world error
        self.vehicle_odometry_pub.publish(msg_px4)
        
    # Callback to keep timestamp for synchronization purposes
    def timesync_callback(self, msg):
        self.timesync = msg.timestamp
        
    # # Callback to translate position data from ROS2 Point message to PX4 
    # def translate_position(self, msg):

    #     msg_px4 = VehicleOdometry() # Message to be sent to PX4

    #     msg_px4.timestamp = self.timesync # Set timestamp
    #     msg_px4.timestamp_sample = self.timesync # Timestamp for mocap sample

    #     # Transfer data from mocap message to PX4 message
    #     # VICON East, North, Up to PX4 Front, Right, Down
    #     # Position/orientation components
    #     msg_px4.pose_frame = 2 # FRD from px4 message
    #     enu_coordinates = [msg.x, msg.y, msg.z]
    #     frd_coordinats = [enu_coordinates[1], enu_coordinates[0], -enu_coordinates[2]] 
    #     msg_px4.position = frd_coordinats
        
    #     # Variances
    #     # Assuming 1mmm standard deviation
    #     msg_px4.position_variance = [10**(-6), 10**(-6), 10**(-6)]
        
    #     self.vehicle_odometry_pub.publish(msg_px4) # Publish to PX4

def main(args=None):
    rclpy.init(args=args)

    translator = Translator()

    rclpy.spin(translator)

    # Destroy the node explicitly
    translator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
