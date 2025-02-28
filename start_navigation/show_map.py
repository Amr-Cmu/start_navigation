import rclpy
from rclpy.node import Node
import websockets
import asyncio
import json
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.msg import Odometry
import threading
from tf_transformations import euler_from_quaternion

class MapOdomWebSocketPublisher(Node):
    def __init__(self):
        super().__init__('map_websocket_publisher')
        
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',  
            self.odom_callback,
            10
        )

        self.path_subscription = self.create_subscription(
            Path,
            'plan',
            self.path_callback,
            10
        )
        
        self.latest_map = None
        self.map_metadata = None
        self.latest_odom = None
        self.latest_path = None
        
        self.ws_loop = None
        self.ws_server = None
        
        self.ws_thread = threading.Thread(target=self.start_ws_server)
        self.ws_thread.daemon = True
        self.ws_thread.start()

    def path_callback(self, msg):
        path_points = []
        for pose in msg.poses:
            path_points.append({
                'x': pose.pose.position.x,
                'y': pose.pose.position.y
            })
            
        self.latest_path = path_points

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        map_data = np.array(msg.data).reshape(height, width)
        
        self.map_metadata = {
            'resolution': msg.info.resolution,
            'width': width,
            'height': height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y
        }
        
        self.latest_map = map_data.tolist()
        self.get_logger().debug('Received new map data')

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        
        self.latest_odom = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': yaw,  
            'linear_velocity': msg.twist.twist.linear.x,
            'angular_velocity': msg.twist.twist.angular.z
        }
        self.get_logger().debug('Received new odom data')

    async def websocket_handler(self, websocket):
        self.get_logger().info('New client connected')
        try:
            while True:
                if self.latest_map is not None and self.map_metadata is not None:
                    data = {
                        'type': 'navigation',
                        'map_data': self.latest_map,
                        'metadata': self.map_metadata
                    }
                    if self.latest_odom is not None:
                        data['odom'] = self.latest_odom

                    if self.latest_path is not None:
                        data['path'] = self.latest_path
                    
                    await websocket.send(json.dumps(data))
                await asyncio.sleep(0.1)  
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("Client disconnected")
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {str(e)}")

    async def setup_ws_server(self):
        try:
            self.ws_server = await websockets.serve(
                self.websocket_handler,
                'localhost',
                9090
            )
            self.get_logger().info('WebSocket server started on ws://localhost:9090')
            await asyncio.Future()
        except Exception as e:
            self.get_logger().error(f"Failed to start WebSocket server: {str(e)}")

    def start_ws_server(self):
        self.ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.ws_loop)
        try:
            self.ws_loop.run_until_complete(self.setup_ws_server())
        except Exception as e:
            self.get_logger().error(f"WebSocket server error: {str(e)}")
        finally:
            self.ws_loop.close()

    def shutdown(self):
        if self.ws_server:
            self.ws_server.close()
        if self.ws_loop and self.ws_loop.is_running():
            self.ws_loop.stop()
        if self.ws_loop and not self.ws_loop.is_closed():
            self.ws_loop.close()

def main(args=None):
    rclpy.init(args=args)
    publisher = MapOdomWebSocketPublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.shutdown()
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()