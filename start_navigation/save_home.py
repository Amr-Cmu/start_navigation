import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import csv
import os
import sys

class SinglePointLogger(Node):
    def __init__(self):
        super().__init__('single_point_logger')
        
        self.point_csv = '/home/thanawat/amr_ws/src/follow_person/csv/default_point.csv'
        
        self.point_saved = False
        
        self.init_csv_file(self.point_csv)
        
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
            
        # Subscriber รับข้อมูลจาก Topic save_file
        self.save_file_subscriber = self.create_subscription(
            String,
            '/save_point',  # เปลี่ยนเป็น save_point เพื่อแยกจาก save_file
            self.save_file_callback,
            10)
        
    def init_csv_file(self, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'X', 'Y', 'Orientation_X', 'Orientation_Y', 'Orientation_Z', 'Orientation_W'])
    
    def pose_callback(self, msg):
        if not self.point_saved:  # บันทึกเฉพาะเมื่อยังไม่ได้บันทึก
            timestamp = self.get_clock().now().to_msg().sec
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            
            self.write_to_csv(self.point_csv, [timestamp, x, y, qx, qy, qz, qw])
            self.point_saved = True
            self.get_logger().info('Point position saved. Waiting for save command...')

    def save_file_callback(self, msg):
        data = msg.data.split(',')
        if len(data) != 2 or data[0].strip().lower() != 'true':
            self.get_logger().warn('Invalid save_point message format!')
            return
            
        if not self.point_saved:
            self.get_logger().warn('No point has been saved yet!')
            return
            
        file_name = data[1].strip()
        
        # บันทึกไฟล์ตามชื่อที่ได้รับ
        os.rename(self.point_csv, f'/home/thanawat/amr_ws/src/follow_person/csv/{file_name}')
        
        self.get_logger().info(f'Saved point as {file_name}')
        
        # หยุดการทำงานของโปรแกรม
        self.get_logger().info('Shutting down node...')
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    def write_to_csv(self, filename, data):
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(data)
        self.get_logger().info(f'Saved data to {filename}')

def main(args=None):
    rclpy.init(args=args)
    node = SinglePointLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()