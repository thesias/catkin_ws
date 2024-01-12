import rospy
import math
import time
import cv2
# import pyzed.sl as sl

# from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from my_controller.msg import IMU
from ultralytics import YOLO



model=YOLO('/home/euclid/catkin_ws/src/my_controller/src/best.pt')
# ARUCO_DICT = {
# 	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
# 	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
# 	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
# 	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
# 	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
# 	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
# 	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
# 	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
# 	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
# 	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
# 	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
# 	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000, 
# 	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
# 	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
# 	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
# 	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
# 	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
# 	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
# 	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
# 	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
# 	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
# }
initial = None
target=None
yaw=0
# zed = sl.Camera()
cap = cv2.VideoCapture(2)
# qx = data.pose.orientation.x
# def once(data):
#     global initial
#     qx = data.pose.orientation.x
#     qy = data.pose.orientation.y
#     qz = data.pose.orientation.z
#     qw = data.pose.orientation.w
#     initial = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy*2 + qz*2))
#     print("Updated initial value inside callback: ", initial)  # Print the updated value here

# def print_initial():
#     print("Base angl e:", initial)
markerID=-2
def callback(data):
    global yaw,diff,initial,markerID,target
    qx = data.pose.orientation.x
    qy = data.pose.orientation.y
    qz = data.pose.orientation.z
    qw = data.pose.orientation.w
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))*180/math.pi
    if(initial==None):
        initial=yaw
    diff = initial-yaw
    if(target!=None):
        diff=target-yaw
    pub.publish(float(diff),int(markerID))
    print("diff is:",diff)

if __name__ =='__main__':
    rospy.init_node('imu_data',anonymous=True)
    rospy.Subscriber('/zed2i/zed_node/pose',PoseStamped,callback)
    pub = rospy.Publisher('servo',IMU,queue_size=10)
    print(1)
    r = rospy.Rate(10)
    r.sleep()
#     def aruco_display(corners, ids, rejected, image):
#         global markerID
#         if len(corners) > 0:
#             ids = ids.flatten()
#             for (markerCorner, markerID) in zip(corners, ids):
#                 corners = markerCorner.reshape((4, 2))
#                 (topLeft, topRight, bottomRight, bottomLeft) = corners
#                 topRight = (int(topRight[0]), int(topRight[1]))
#                 bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
#                 bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
#                 topLeft = (int(topLeft[0]), int(topLeft[1]))
#                 cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
#                 cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
#                 cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
#                 cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
#                 cX = int((topLeft[0] + bottomRight[0]) / 2.0)
#                 cY = int((topLeft[1] + bottomRight[1]) / 2.0)
#                 cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
#                 cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
#                     0.5, (0, 255, 0), 2)
#                 print("[Inference] ArUco marker ID: {}".format(markerID))
#         return image
#     aruco_type = "DICT_4X4_50"
#     arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
#     arucoParams = cv2.aruco.DetectorParameters_create()
#     cap = cv2.VideoCapture(0)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#     while cap.isOpened():
#         global target
#         ret, img = cap.read()
#         h, w, _ = img.shape 
#         width = 1000
#         height = int(width*(h/w))
#         img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)
#         corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
#         detected_markers = aruco_display(corners, ids, rejected, img)
#         cv2.imshow("Image", detected_markers)
#         key = cv2.waitKey(1) & 0xFF
#         if key == ord("q"):
#             break  
#         if  (markerID != None):
#             if (markerID == 1):
#                 target = yaw + 90
#             elif (markerID == 0):
#                 target = yaw - 90    
#             time.sleep(5)

    # cv2.destroyAllWindows() 
    # cap.release()
    check=0
    while True:
        
        ret, frame = cap.read()

        results=model(frame,show=True)  
        for result in results:
            if result.boxes.cls.numel() > 0:
                markerID = (result.boxes.cls.item())  # Convert PyTorch tensor to Python integer
                print(markerID)
                # pub.publish(Float64(markerID))
                if((markerID==0 or markerID==1) and check==0):
                    if(markerID==0):
                        target=yaw-90
                    elif(markerID==1):
                        target=yaw+90
                    check=1
                # elif(check==1 and markerID==None):
                    check=0
            else:
                continue    
        # time.sleep(1)
            # print(result.boxes.cls)
            # print(type(result.boxes.cls))
            
        cv2.imshow("Frame",frame)
    rospy.spin()