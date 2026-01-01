import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import os

class GripperBridge(Node):
    def __init__(self):
        super().__init__('gripper_bridge')
        
        # 구독 설정 (노트북에서 보내는 'gripper_cmd' 토픽을 듣습니다)
        self.subscription = self.create_subscription(
            String,
            'gripper_cmd',
            self.listener_callback,
            10)
        
        self.ser = None
        self.port_name = '/dev/ttyACM0' # STM32 연결 포트
        self.baud_rate = 115200         # 통신 속도 (STM32와 일치해야 함)
        
        # 초기 연결 시도
        self.connect_serial()

    def connect_serial(self):
        """시리얼 연결을 시도하는 함수 (재연결 로직 포함)"""
        if self.ser and self.ser.is_open:
            self.ser.close()

        try:
            # 포트가 실제로 있는지 확인
            if os.path.exists(self.port_name):
                self.ser = serial.Serial(self.port_name, self.baud_rate, timeout=1)
                self.get_logger().info(f'✅ STM32 Connected on {self.port_name}')
            else:
                self.get_logger().warn(f'Waiting for {self.port_name}...')
        except Exception as e:
            self.get_logger().error(f'Connection Failed: {e}')

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Laptop sent: "{command}"')
        
        # 연결이 없으면 재시도
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('⚠ Serial disconnected. Trying to reconnect...')
            self.connect_serial()
            return

        try:
            # STM32로 전송 (엔터키 \n 추가)
            send_data = f"{command}\n"
            self.ser.write(send_data.encode('utf-8'))
        
        except serial.SerialException as e:
            # ★ 에러 발생 시 프로그램 죽지 않고 재연결 시도 ★
            self.get_logger().error(f'❌ USB Error (Power drop?): {e}')
            self.get_logger().warn('Hardware reset detected. Reconnecting...')
            
            # 연결 객체 초기화 후 재시도
            if self.ser:
                self.ser.close()
            self.ser = None
            time.sleep(1) # STM32 재부팅 시간 벌어주기
            self.connect_serial()

def main(args=None):
    rclpy.init(args=args)
    node = GripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()