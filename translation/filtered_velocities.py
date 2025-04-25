# Impot Packages
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Import Ros Messages
from geometry_msgs.msg import Point, PointStamped
from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import VehicleOdometry
from vicon_receiver.msg import Position

class FilteredVelocity(Node):

    def __init__(self):
        super().__init__('filtered_velocity_node')
        
        # Create QoS policy for PX4 publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Intialize varaibles
        self.timesync = 0
        self.alpha = 0.075  # Low-pass filter coefficient
        self.prev_position = None
        self.prev_time = None
        self.filtered_position = None
        self.filtered_velocity = None
                        
        # Create publishers and subscribers
        
        # Initialize subscriber to the Vicon motion capture topic
        self.mocap_sub = self.create_subscription(Position, 
            '/vicon/Turner_X500/Turner_X500', self.mocap_callback, 10)
        
        # Initilize publisher for motion capture data on /position topic
        self.position_pub = self.create_publisher(PointStamped, 
            '/position', 10)
        
        # Initialize publisher for filtered velocity data on /velocity topic
        self.velocity_pub = self.create_publisher(PointStamped,
            '/velocity', 10)

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
        
        enu_coordinates = np.float32([x_pos, y_pos, z_pos])
        # frd_pos_coordinates = np.float32([-enu_coordinates[0], enu_coordinates[1], enu_coordinates[2]])
        frd_pos_coordinates = np.float32([enu_coordinates[1], enu_coordinates[0], -enu_coordinates[2]])
        
        # Apply low-pass filter to position
        if not np.any(self.filtered_position):
            self.filtered_position = enu_coordinates
        else:
            self.filtered_position = self.alpha * enu_coordinates + (1 - self.alpha) * self.filtered_position

        # Calculate velocity and filter velocities, if possible
        current_time = self.get_clock().now().nanoseconds / 1e9
        # Check if there is a current/prev position and current/previous tiem
        if np.any(enu_coordinates) and np.any(self.prev_position) and self.prev_time != None:
            dt = current_time - self.prev_time
            raw_velocity = (enu_coordinates - self.prev_position) / dt
            # Ignore velocity measurement if greater than 10 m/s
            if np.linalg.norm(raw_velocity) < 10:
                if self.filtered_velocity is None:
                    self.filtered_velocity = raw_velocity
                else:
                    self.filtered_velocity = self.alpha * raw_velocity + (1 - self.alpha) * self.filtered_velocity
            # frd_vel_coordinates = np.array([-self.filtered_velocity[0], self.filtered_velocity[1], self.filtered_velocity[2]])
            frd_vel_coordinates = np.array([self.filtered_velocity[1], self.filtered_velocity[0], -self.filtered_velocity[2]])
        else:
            frd_vel_coordinates = np.float32([0, 0, 0])
                    
        # Update previous position and time
        self.prev_position = enu_coordinates
        self.prev_time = current_time
        
        # Create point message from Vicon position data (NED coordinates)
        position = PointStamped()
        position.header.stamp.nanosec = 0
        position.point.x = float(frd_pos_coordinates[0]) # Cast numpy.float64 to float
        position.point.y = float(frd_pos_coordinates[1])
        position.point.z = float(frd_pos_coordinates[2])
        self.position_pub.publish(position)
        
        # Create point message from filtered velocity data
        position = PointStamped()
        position.header.stamp.nanosec = 0
        position.point.x = float(frd_vel_coordinates[0]) # Cast numpy.float64 to float
        position.point.y = float(frd_vel_coordinates[1])
        position.point.z = float(frd_vel_coordinates[2])
        self.velocity_pub.publish(position)

        # Create PX4 message from Vicon position data
        msg_px4 = VehicleOdometry() # Message to be sent to PX4
        msg_px4.timestamp = self.timesync # Set timestamp
        msg_px4.timestamp_sample = self.timesync # Timestamp for mocap sample
        msg_px4.pose_frame = 2 # FRD from px4 message
        msg_px4.velocity_frame = 2 # FRD from px4 message
        msg_px4.position = frd_pos_coordinates.tolist() # Convert numpy array to list
        msg_px4.velocity = frd_vel_coordinates.tolist()
        msg_px4.position_variance = [10**(-6), 10**(-6), 10**(-6)] # Assuming 1mmm standard deviation in world error
        msg_px4.velocity_variance = [10**(-6), 10**(-6), 10**(-6)]
        self.vehicle_odometry_pub.publish(msg_px4)
        
    # Callback to keep timestamp for synchronization purposes
    def timesync_callback(self, msg):
        self.timesync = msg.timestamp

def main(args=None):
    rclpy.init(args=args)

    filter = FilteredVelocity()

    rclpy.spin(filter)

    # Destroy the node explicitly
    filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
