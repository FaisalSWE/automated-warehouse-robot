#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class QRScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner_node')
        self.publisher = self.create_publisher(String, '/qr_data', 10)
        self.camera = cv2.VideoCapture(0)  # Raspberry Pi Camera or USB cam
        self.timer = self.create_timer(0.1, self.scan_qr)

    def scan_qr(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        detector = cv2.QRCodeDetector()
        data, bbox, _ = detector.detectAndDecode(frame)
        if data:
            self.get_logger().info(f'QR Code detected: {data}')
            msg = String()
            msg.data = data
            self.publisher.publish(msg)

    def destroy_node(self):
        self.camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = QRScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
