import os
import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from pestibot_messages.msg import Detection
from rclpy.node import Node
from ultralytics import YOLO


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_ai_node')
        self.get_logger().info('Camera AI node started.')

        self.cap = None
        self.yolo_ai = None

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('confidence_threshold', 0.5)

        self.conf_threshold = float(
            self.get_parameter('confidence_threshold').value
        )

        self.setup_cam()
        self.setup_ai()

        self.detection_publisher = self.create_publisher(Detection, 'detect', 10)
        self.timer = self.create_timer(0.1, self.process_ai)

    def setup_cam(self):
        self.get_logger().info('Initializing camera...')
        camera_index = int(self.get_parameter('camera_index').value)
        self.cap = cv2.VideoCapture(camera_index)

        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error('Failed to initialize camera.')
            self.cap = None
            return

        self.get_logger().info('Camera successfully started.')

    def setup_ai(self):
        self.get_logger().info('Initializing AI model...')
        try:
            package_share_dir = get_package_share_directory('camera_pkg')
            model_path = os.path.join(package_share_dir, 'config', 'best.pt')
            self.get_logger().info(f'Loading YOLO model from: {model_path}')
            self.yolo_ai = YOLO(model_path)
            self.get_logger().info('YOLO model loaded.')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.yolo_ai = None

    def process_ai(self):
        if self.cap is None or self.yolo_ai is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame.')
            return

        results = self.yolo_ai(frame, verbose=False)
        annotated = frame.copy()

        msg = Detection()
        msg.stamp = self.get_clock().now().to_msg()
        msg.image_width = frame.shape[1]
        msg.image_height = frame.shape[0]

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                conf = float(box.conf[0].item())
                if conf < self.conf_threshold:
                    continue

                cls_id = int(box.cls[0].item())
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                if isinstance(self.yolo_ai.names, dict):
                    cls_name = self.yolo_ai.names.get(cls_id, str(cls_id))
                else:
                    cls_name = self.yolo_ai.names[cls_id]

                msg.class_ids.append(cls_id)
                msg.class_names.append(str(cls_name))
                msg.confidences.append(conf)
                msg.x_min.append(x1)
                msg.y_min.append(y1)
                msg.x_max.append(x2)
                msg.y_max.append(y2)

                label = f'{cls_name} {conf:.2f}'
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    annotated,
                    label,
                    (x1, max(0, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        # Publish only when at least one detection exists.
        if len(msg.class_ids) > 0:
            self.detection_publisher.publish(msg)

        cv2.imshow('AI Detection', annotated)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    finally:
        if camera_node.cap is not None:
            camera_node.cap.release()
        cv2.destroyAllWindows()
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()