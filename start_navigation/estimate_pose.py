import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import os
import csv
import time
import sys

class AMRNav(Node):
    def __init__(self, home):
        super().__init__('start_navigation')
        self.navigator = BasicNavigator()
        self.status_publisher = self.create_publisher(String, 'start_status', 10)
        self.go_home_subscriber = self.create_subscription(String, 'go_home', self.home_callback, 10)

        base_path = '/home/thanawat/amr_ws/src/start_navigation/home_point'
        self.csv_filename_B = os.path.join(base_path, home)

        self.publish_status("กำลังเตรียมความพร้อมหุ่นยนต์")

        self.navigator.waitUntilNav2Active()
        
        self.estimate_home = self.load_home_position()
        self.initial_pose()

    def home_callback(self, msg):
        go_home = msg.data
        if go_home == 'go_home':
            self.publish_status("เริ่มเดินทางกลับ Home position")
            
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = "map"
            pose_goal.header.stamp = self.get_clock().now().to_msg()
            pose_goal.pose.position.x = float(self.estimate_home[0])
            pose_goal.pose.position.y = float(self.estimate_home[1])
            pose_goal.pose.orientation.z = float(self.estimate_home[4])
            pose_goal.pose.orientation.w = float(self.estimate_home[5])
            
            self.navigator.goToPose(pose_goal)
            
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    remaining_distance = feedback.distance_remaining
                    self.publish_status(f"กำลังเดินทางกลับ Home เหลือระยะทาง {remaining_distance:.2f} เมตร")
                time.sleep(1.0)
                
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.publish_status("เดินทางถึง Home position แล้ว")
            elif result == TaskResult.CANCELED:
                self.publish_status("การเดินทางถูกยกเลิก")
            else:
                self.publish_status("เกิดข้อผิดพลาดในการเดินทาง")

    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_publisher.publish(msg)
    
    def load_home_position(self):
        if not os.path.exists(self.csv_filename_B):
            self.publish_status(f'ไม่พบไฟล์ {self.csv_filename_B}!')
            return None
        
        with open(self.csv_filename_B, 'r') as csvfile_B:
            reader = csv.reader(csvfile_B)
            next(reader) 
            for row in reader:
                try:
                    x, y, ox, oy, oz, ow = map(float, row[1:7])
                    return (x, y, ox, oy, oz, ow)
                except ValueError:
                    self.publish_status(f'ข้อมูลในไฟล์ไม่ถูกต้อง: {row}')
        return None
    
    def initial_pose(self):
        if self.estimate_home is None:
            self.publish_status('ไม่พบข้อมูลตำแหน่งเริ่มต้น!')
            return

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.position.x = self.estimate_home[0]
        pose_msg.pose.position.y = self.estimate_home[1]
        pose_msg.pose.orientation.x = self.estimate_home[2]
        pose_msg.pose.orientation.y = self.estimate_home[3]
        pose_msg.pose.orientation.z = self.estimate_home[4]
        pose_msg.pose.orientation.w = self.estimate_home[5]
        
        self.navigator.setInitialPose(pose_msg)
        self.publish_status('หุ่นยนต์พร้อมทำงานแล้ว')
        time.sleep(3)

def main():
    rclpy.init()
    home = sys.argv[1]
    node = AMRNav(home)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
