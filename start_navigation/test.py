import rclpy
from rclpy.node import Node
import numpy as np
import json
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion

class NavDataProcessorNode(Node):
    def __init__(self):
        super().__init__('nav_data_processor_node')
        
        # Subscriptions
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        # Publisher for web data
        self.web_data_publisher = self.create_publisher(
            String,
            'web_data',
            10
        )

    def publish_to_websocket(self, data_type, data):
        msg = String()
        message = {
            'type': data_type,
            'timestamp': self.get_clock().now().to_msg().sec,
            'data': data
        }
        msg.data = json.dumps(message)
        self.web_data_publisher.publish(msg)
        self.get_logger().debug(f'Published new {data_type} data')

    def pose_callback(self, msg):
        # Extract position
        position = msg.pose.pose.position
        
        # Extract orientation and convert to euler angles
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        # Extract covariance (position and orientation uncertainty)
        covariance = np.array(msg.pose.covariance).reshape(6, 6).tolist()
        
        pose_data = {
            'position': {
                'x': position.x,
                'y': position.y,
                'z': position.z
            },
            'orientation': {
                'yaw': yaw
            },
            'covariance': covariance
        }
        
        self.publish_to_websocket('pose_update', pose_data)

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data).reshape(height, width)
        
        map_info = {
            'data': map_data.tolist(),
            'metadata': {
                'resolution': msg.info.resolution,
                'width': width,
                'height': height,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y
            }
        }
        
        self.publish_to_websocket('map_update', map_info)

def main(args=None):
    rclpy.init(args=args)
    processor = NavDataProcessorNode()
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()