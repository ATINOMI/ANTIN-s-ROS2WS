import rclpy
from rclpy.node import Node
from ultralytics import YOLO # type: ignore
import cv2
from yolo_msgs.msg import Detection,DetectionArray


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('model_path','yolo11n.engine')
        model_path=self.get_parameter('model_path').value
        self.declare_parameter('input_path','0')
        input_path=self.get_parameter('input_path').value       
        self.declare_parameter('conf_threshold',0.5)
        self.conf_threshold=self.get_parameter('conf_threshold').value  
        self.declare_parameter('publish_rate',30.0)
        publish_rate=self.get_parameter('publish_rate').value
        self.model =YOLO(model_path)
        if input_path.isdigit():
            source=int(input_path)
        else :
            source =input_path
        self.cap=cv2.VideoCapture(source)
        self.pub =self.create_publisher(DetectionArray, '/yolo/detections' ,10)
        self.timer = self.create_timer(1.0/publish_rate,self.timer_callback)  
    def timer_callback(self):
        ret,frame=self.cap.read()
        if not ret:
            return
        results=self.model(frame,conf=self.conf_threshold, verbose=False)
        msg=DetectionArray()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id="camera"
        for result in results:
            for box in result.boxes:
                cls_id =int(box.cls[0])
                class_name=result.names[cls_id]
                confidence =float(box.conf[0])
                xywh=box.xywh[0]
                det=Detection()
                det.class_name=class_name
                det.confidence=confidence
                det.x=float(xywh[0])
                det.y=float(xywh[1])
                det.width=float(xywh[2])
                det.height=float(xywh[3])
                msg.detections.append(det)
        for det in msg.detections:
            cx,cy,w,h=int(det.x),int(det.y),int(det.width),int(det.height)
            x1,y1=cx-w//2,cy-h//2
            x2,y2=cx+w//2,cy+h//2
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.putText(frame,f"{det.class_name}{det.confidence:.2f}",(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
        cv2.namedWindow('YOLO Detection',cv2.WINDOW_NORMAL)
        cv2.imshow('YOLO Detection',frame)
        cv2.waitKey(1)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()