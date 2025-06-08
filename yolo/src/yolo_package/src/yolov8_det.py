#!/usr/bin/env python3

import rospy, torch, cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO 


class Yolov8Detector:
    def __init__(self):
        self.bridge   = CvBridge()
        #self.pub      = rospy.Publisher("/camera/overlay", Image, queue_size=10)
        self.object_pub = rospy.Publisher("/motion/yolo_motion_command", String, queue_size=1)
        self.device   = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

        self.model = YOLO('/home/ubuntu/EVC/workshops/workshop4_motion_1654411/best.pt').to(self.device)

        self.model.eval()
        rospy.loginfo("YOLO Model loaded on %s", self.device)

        rospy.Subscriber("/camera/image_undistorted", Image, self.cb, queue_size=None)

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.model(frame)[0]
        boxes   = results.boxes.xyxy.cpu().numpy() 
        scores  = results.boxes.conf.cpu().numpy()  
        classes = results.boxes.cls.cpu().numpy()    

        for (x1, y1, x2, y2), cls_id, conf in zip(boxes, classes, scores):
            label_txt = f"{self.model.names[int(cls_id)]} {conf*100:.0f}%"

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 255, 0), 2)
            cv2.putText(frame, label_txt, (int(x1), int(y1)-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            # publish just the class name (without %) on /detected_object
            label_msg = String()
            label_msg.data = self.model.names[int(cls_id)]
            self.object_pub.publish(label_msg)

        # out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # out.header = msg.header                
        # self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("yolov8_detector", xmlrpc_port=49100, tcpros_port=49101)
    Yolov8Detector()
    rospy.spin()
