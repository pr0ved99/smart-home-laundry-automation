import rclpy
import math
import time
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# [설정] 좌표 및 방향 오프셋
CORRECTION_OFFSET = 14
WAIT_TIME_AT_WAYPOINT = 1

# ★ 컨베이어 좌표 (사용자 최신 테스트 값 반영)
CONVEYOR_APPROACH = [3.3, -0.555, 180.0]
CONVEYOR_DROP = [2.955, -0.555, 180.0]

class FinalIntegratedMissionNode(Node):
    def __init__(self):
        super().__init__('final_integrated_mission_node')
        self.bridge = CvBridge()
        
        # 실제 Pi Camera 압축 이미지 토픽 구독
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.image_callback, 
            1)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.gripper_pub = self.create_publisher(String, '/gripper_cmd', 10)
        
        # 상태 제어 변수
        self.detected = False
        self.is_in_roi = False
        self.is_full_inside = False
        self.error_x = 0
        self.command_sent = False
        self.detected_color_name = "NONE"
        self.kernel = np.ones((5, 5), np.uint8)

    def image_callback(self, msg):
        # 1. 압축 이미지 디코딩 및 160x160 리사이즈
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return
        frame = cv2.resize(frame, (160, 160))
        display_img = frame.copy()
        h, w = 160, 160

        # 2. 가이드라인 설정 (하단 10% ROI)
        roi_w_rate, roi_h_rate = 0.5, 0.1 
        x1, y1 = int((w - int(w*roi_w_rate))/2), h - int(h*roi_h_rate)
        x2, y2 = x1 + int(w*roi_w_rate), h
        sq_size = y2 - y1
        sq_x1, sq_x2 = (w // 2) - (sq_size // 2), (w // 2) + (sq_size // 2)

        # 3. 비전 처리 (사용자가 성능을 검증한 HSV 범위 적용)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_masks = self.get_individual_masks(hsv)
        full_mask = np.zeros((h, w), dtype=np.uint8)
        for m in color_masks: full_mask |= m['mask']
        full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, self.kernel)

        # 4. 물체 감지 및 상태 업데이트
        coords = cv2.findNonZero(full_mask)
        status_text = "SEARCHING..."
        status_color = (255, 255, 255)

        if coords is not None:
            rx, ry, rw, rh = cv2.boundingRect(coords)
            obj_x1, obj_x2, obj_y1, obj_y2 = rx, rx + rw, ry, ry + rh
            self.detected_color_name = self.identify_color(color_masks, rx, ry, rw, rh)
            
            # 정밀 판정 로직
            self.is_in_roi = (obj_x1 >= x1 and obj_x2 <= x2 and obj_y1 >= y1 and obj_y2 <= y2)
            self.detected = True

            if self.is_in_roi:
                self.is_full_inside = (obj_x1 >= sq_x1 and obj_x2 <= sq_x2)
                self.error_x = ((obj_x1 + obj_x2) / 2) - (w / 2)
                status_text = "LOCKED!" if self.is_full_inside else "ALIGNING..."
                status_color = (0, 255, 0) if self.is_full_inside else (0, 255, 255)
            
            cv2.rectangle(display_img, (obj_x1, obj_y1), (obj_x2, obj_y2), (0, 0, 255), 1)
        else:
            self.detected = self.is_in_roi = self.is_full_inside = False

        # 5. 시각화 (GUI 출력)
        cv2.rectangle(display_img, (x1, y1), (x2, y2-1), (0, 255, 0), 1)
        cv2.rectangle(display_img, (sq_x1, y1), (sq_x2, y2-1), (255, 255, 0), 1)
        cv2.putText(display_img, f"{self.detected_color_name}: {status_text}", (5, 15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)

        cv2.imshow("Pi Camera Mission (160x160)", display_img)
        cv2.waitKey(1)

    def identify_color(self, color_masks, x, y, w, h):
        max_px, name = 0, "UNKNOWN"
        for item in color_masks:
            count = cv2.countNonZero(item['mask'][y:y+h, x:x+w])
            if count > max_px: max_px, name = count, item['name']
        return name

    def get_individual_masks(self, hsv):
        """사용자가 성능을 인정한 HSV 범위 세팅"""
        return [
            {'name': 'RED', 'mask': cv2.inRange(hsv, np.array([0, 150, 150]), np.array([10, 255, 255])) | 
                                    cv2.inRange(hsv, np.array([170, 150, 150]), np.array([180, 255, 255]))},
            {'name': 'YELLOW', 'mask': cv2.inRange(hsv, np.array([20, 150, 150]), np.array([35, 255, 255]))},
            {'name': 'GREEN', 'mask': cv2.inRange(hsv, np.array([35, 120, 120]), np.array([85, 255, 255]))},
            {'name': 'BLUE', 'mask': cv2.inRange(hsv, np.array([100, 150, 120]), np.array([130, 255, 255]))}
        ]

def degrees_to_quaternion(degrees):
    radians = math.radians(degrees + CORRECTION_OFFSET)
    return 0.0, 0.0, math.sin(radians / 2), math.cos(radians / 2)

def make_pose(x, y, angle, navigator):
    goal = PoseStamped()
    goal.header.frame_id, goal.header.stamp = 'map', navigator.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = x, y
    _, _, goal.pose.orientation.z, goal.pose.orientation.w = degrees_to_quaternion(angle)
    return goal

def main():
    rclpy.init()
    navigator = BasicNavigator()
    tracker = FinalIntegratedMissionNode()
    navigator.waitUntilNav2Active()

    PATROL_WAYPOINTS = [
        [3.3, 0.0, 0.0], [3.8, 0.0, 0.0], [4.5, 0.0, 270.0],
        [4.5, -0.35, 270.0], [4.5, -0.7, 180.0], 
        [3.9, -0.7, 180.0], [3.3, -0.7, 90.0], [3.3, -0.35, 0.0]
    ]
    goals = [make_pose(pt[0], pt[1], pt[2], navigator) for pt in PATROL_WAYPOINTS]

    try:
        while rclpy.ok():
            for i, goal in enumerate(goals):
                tracker.get_logger().info(f"📍 Waypoint {i+1} 이동 중...")
                navigator.goToPose(goal)

                while not navigator.isTaskComplete():
                    rclpy.spin_once(tracker, timeout_sec=0.01)
                    
                    if tracker.is_in_roi and not tracker.command_sent:
                        tracker.get_logger().warn(f"📦 [{tracker.detected_color_name}] 감지!")
                        navigator.cancelTask()
                        tracker.gripper_pub.publish(String(data='2')) # 준비 명령
                        
                        # 1. 정밀 조향 (사용자 피드백 수치 -0.004 적용)
                        twist = Twist()
                        while rclpy.ok() and not tracker.is_full_inside:
                            rclpy.spin_once(tracker, timeout_sec=0.01)
                            if not tracker.detected: break
                            twist.angular.z = -0.004 * tracker.error_x
                            tracker.cmd_vel_pub.publish(twist)
                            
                        # 루프 종료 직후 반드시 정지 명령 발행
                        stop_twist = Twist()
                        tracker.cmd_vel_pub.publish(stop_twist) 
                        tracker.get_logger().info("🛑 정렬 완료, 즉시 정지")

                        # 2. 정중앙 안착 시 집기
                        if tracker.is_full_inside:
                            tracker.cmd_vel_pub.publish(Twist())
                            tracker.gripper_pub.publish(String(data='1'))
                            tracker.command_sent = True
                            time.sleep(3.5)

                            # 3. 컨베이어 이동
                            approach_pose = make_pose(CONVEYOR_APPROACH[0], CONVEYOR_APPROACH[1], CONVEYOR_APPROACH[2], navigator)
                            navigator.goToPose(approach_pose)
                            while not navigator.isTaskComplete(): rclpy.spin_once(tracker, timeout_sec=0.01)

                            time.sleep(3.5)
                            navigator.goToPose(make_pose(CONVEYOR_DROP[0], CONVEYOR_DROP[1], CONVEYOR_DROP[2], navigator))
                            while not navigator.isTaskComplete(): rclpy.spin_once(tracker, timeout_sec=0.01)

                            # 4. 투하 및 후진 (사용자 테스트 수치 적용)
                            tracker.gripper_pub.publish(String(data='q'))
                            time.sleep(3.5)
                            
                            tracker.get_logger().info("🏎️ 고속 후진 실시")
                            rev_twist = Twist()
                            rev_twist.linear.x = -0.3
                            for _ in range(20):
                                tracker.cmd_vel_pub.publish(rev_twist)
                                time.sleep(0.05)
                            tracker.cmd_vel_pub.publish(Twist())
                            
                            tracker.command_sent = False
                            navigator.goToPose(goal)
                        else:
                            navigator.goToPose(goal)

                if navigator.getResult() == TaskResult.SUCCEEDED:
                    time.sleep(WAIT_TIME_AT_WAYPOINT)
                
    except KeyboardInterrupt: pass
    finally:
        navigator.cancelTask()
        rclpy.shutdown()

if __name__ == '__main__':
    main()