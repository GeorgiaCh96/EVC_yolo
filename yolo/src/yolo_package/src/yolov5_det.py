#!/usr/bin/env python3

import os
import rospy, torch, cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO 


class Yolov8Detector:
    def __init__(self):
        self.initialized=False
        rospy.loginfo("Initializing YOLO detector node...")
        rospy.init_node("yolov8_detector", anonymous=True, xmlrpc_port=49100, tcpros_port=49101)  # anonymous=True?

        self.bridge   = CvBridge()
        self.first_image_received = False

        # Subscriber for undistorted images (sent from Calibration_node)
        self.sub_image = rospy.Subscriber(
            "/camera/image_undistorted", 
            CompressedImage,
            self.image_cb,
            buff_size=2**24,
            queue_size=1)  # 1 or None?
        
        # publish motion command to /motion/yolo_motion_command topic
        self.command_pub = rospy.Publisher(
            "/motion/yolo_motion_command", 
            String, 
            queue_size=1)
        
        # publish annotated image (for inspection)
        self.annotated_pub = rospy.Publisher(
            "/camera/yolo_processed_image",
            CompressedImage,
            queue_size=1
        )

        #self.last_pub_time   = rospy.Time(0) 
        self.last_cmd        = None            
        # self.updated = False
        # self.pub_interval    = rospy.Duration(5.0)
        # self.timer = rospy.Timer(self.pub_interval, self.timer_cb)
        
        self.initialized=True
        rospy.loginfo("YOLO detector node initialized!")

        self.device   = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

        current_dir = os.path.dirname(os.path.abspath(__file__))
        try:
            self.model = YOLO("/home/ubuntu/EVC/workshops/workshop4_motion_1654411/best.pt").to(self.device)
            rospy.loginfo("YOLO Model loaded on %s", self.device)
        except Exception as e:
            rospy.logerr("Could not load YOLO model: {}".format(e))
            self.model=None


    def image_cb(self, msg):
        """
            Callback fuction to process incomming image messages
        """
        #rospy.loginfo("entered into image callback")

        class_names = {
            0: "Green_light",
            1: "Red_light",
            2: "speed_limit_10",
            3: "speed_limit_100",
            4: "speed_limit_110" ,
            5: "speed_limit_120",
            6: "speed_limit_20",
            7: "speed_limit_30",
            8: "speed_limit_40",
            9: "speed_limit_50" ,
            10: "speed_limit_60",
            11: "speed_limit_70",
            12: "speed_limit_80",
            13: "speed_limit_90",
            14: "stop" 
        }

        if not self.initialized:
            return

        if not self.first_image_received:
            self.first_image_received = True
            rospy.loginfo("YOLO subscriber captured first image from publisher")


        try:
            img_rgb = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            #img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            #rospy.loginfo("PubSub delay: {}".format((rospy.Time.now() - msg.header.stamp).to_sec()))

            #results = self.model(frame)[0]
            results = self.model(img_rgb)[0]
            #results = results[0]
            boxes   = results.boxes.xyxy.cpu().numpy() 
            scores  = results.boxes.conf.cpu().numpy()  
            classes = results.boxes.cls.cpu().numpy()
            #rospy.loginfo("Predicted object: {}".format(class_names[int(classes[0])])) 

            for (x1, y1, x2, y2), cls_id, conf in zip(boxes, classes, scores):
                label_txt = f"{self.model.names[int(cls_id)]} {conf*100:.0f}%"

                cv2.rectangle(img_rgb, (int(x1), int(y1)), (int(x2), int(y2)),
                            (0, 255, 0), 2)
                cv2.putText(img_rgb, label_txt, (int(x1), int(y1)-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

                # publish just the class name (without %) on /detected_object
                #label_msg = String()
                #label_msg.data = self.model.names[int(cls_id)]
                #self.object_pub.publish(label_msg)

                # Compress and publish as CompressedImage
                # out = CompressedImage()
                # #out = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                # out.header.stamp = rospy.Time.now()
                # out.format = "jpeg"
                # out.data = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                
                # self.annotated_pub.publish(out)
                
                # print the motion command
                # if int(cls_id) in range(2, 14):
                #     rospy.loginfo("Send slow down motion command")
                # elif int(cls_id) in [1, 14]:
                #     rospy.loginfo("Send stop command")
                
                # publish motion command
                if classes.size != 0:
                    if int(cls_id) in range(2,14):
                        new_cmd = "G0 100 0.3"
                    elif int(cls_id) == 14:
                        new_cmd = "STOP"

                    if new_cmd != self.last_cmd:
                        self.command_pub.publish(new_cmd)

                    self.last_cmd = new_cmd
                            
                    
            
            success, encoded_image = cv2.imencode(".jpg", img_rgb)
            if success:  
                out = CompressedImage()
                out.header.stamp = rospy.Time.now()
                out.format = "jpeg"
                out.data = encoded_image.tobytes()
                self.annotated_pub.publish(out)



        except Exception as e:
            rospy.logerr("Error processing image: {}".format(e))

    # def timer_cb(self, event):

    #     if self.updated is False:
    #         return            

    #     now = rospy.Time.now()
    #     if now - self.last_pub_time >= self.pub_interval:
    #         self.command_pub.publish(self.last_cmd)
    #         self.last_pub_time = now
    #         self.updated = False
    #         rospy.loginfo("Published motion command: %s", self.last_cmd)




if __name__ == "__main__":
    try:
        yolo_node = Yolov8Detector()
        rate=rospy.Rate(10)
        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        yolo_node.cleanup()
