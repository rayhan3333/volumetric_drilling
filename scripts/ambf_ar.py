#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from scipy.spatial.transform import Rotation as R, Slerp

from ambf_client import Client
import time
_client = Client()
_client.connect()
print(_client.get_obj_names())
drill = _client.get_obj_handle('/ambf/env/mastoidectomy_drill')

camera = _client.get_obj_handle('/ambf/env/cameras/main_camera')

ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)

ARUCO_PARAMS = aruco.DetectorParameters()

time.sleep(0.2)

camera.set_pos(0, 0, 0)
camera.set_rpy(0, 1.57, 0)
MARKER_SIZE = 0.02  # 5 cm



camera_matrix = np.array([[475.77081298828125, 0.0, 315.8103332519531],
 			   [0.0,475.77081298828125, 183.71730041503906],
 			   [0.0, 0.0, 1.0]], dtype=np.float32)
dist_coeffs = np.zeros((5, 1))
detector = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)
first = True
prevz = np.array([0, 0, 0])
prevquat = None

def slerp_quat(prev_quat, curr_quat, alpha=0.05):
    print(prev_quat)
    print(curr_quat)
    r_interp = Slerp([0, 1], R.from_quat([prev_quat, curr_quat]))
    print(r_interp)
    rot = r_interp([alpha])[0]
    return rot.as_quat()
class ArucoDetector:
    def __init__(self):
        #rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/zed2i/zed_node/left/image_rect_color', Image, self.image_callback)

    def image_callback(self, msg):
        global first
        global prevz
        global prevquat
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("cv_bridge exception: %s", e)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            #rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, camera_matrix, dist_coeffs)

            for corner in corners:
                objp = np.array([[-0.5, 0.5, 0], 
                                [0.5, 0.5, 0],
                                [0.5, -0.5, 0],
                                [-0.5, -0.5, 0]]) * MARKER_SIZE
                imgp = corner[0].reshape((4, 2))
            recieved, rvec, tvec = cv2.solvePnP(objp, imgp, camera_matrix, dist_coeffs)

            if (recieved):

            
                #rospy.loginfo(f"Marker ID: {id[0]} | Position: {tvec} | Rotation (rvec): {rvec}")
            
            
                R_matrix, _ = cv2.Rodrigues(rvec)
                rot_curr = R.from_matrix(R_matrix)
                quat_curr = rot_curr.as_quat()
                if prevquat is None:
                    prevquat = quat_curr
                else:
                    prevquat = slerp_quat(prevquat, quat_curr)
                    R_matrix = R.from_quat(prevquat).as_matrix()

                # x= R_matrix[:, 0]
                # y = R_matrix[:, 1]
                # z = np.cross(x, y)
                # z /= np.linalg.norm(z)
                # z = np.array([0, 0, -1])

                # if (first):
                #     prevz = z
                #     first = False
               
                print("new r matrix")
                sy = np.sqrt(R_matrix[0, 0]**2 + R_matrix[1, 0]**2)
                # dot = np.clip(np.dot(z, prevz), -1, 1)
                # angle = np.arccos(dot)
                # if (angle > 0.5):
                #     print("Large change")
                #     #z = prevz
                #     #return

                singular = sy < 1e-6
                # print("first matrx: " + str(R_matrix))
                # R_matrix= np.column_stack((x, y, z))
                # print("second matrix: " + str(R_matrix))

                if not singular:
                    roll  = np.arctan2(R_matrix[2,1], R_matrix[2,2])
                    pitch = np.arctan2(-R_matrix[2,0], sy)
                    yaw   = np.arctan2(R_matrix[1,0], R_matrix[0,0])
                else:
                    roll  = np.arctan2(-R_matrix[1,2], R_matrix[1,1])
                    pitch = np.arctan2(-R_matrix[2,0], sy)
                    yaw   = 0
                #print(str(tvec[0]) +"   " + str(tvec[1]) + "    " + str(tvec[2]))
                #print("Old position:")
                #tvec = tvec + R_matrix @ np.array([0.0, -0.175, -0.02]).reshape((3,1))
                #print(tvec)
                drill.set_pos(1.35*(-tvec[1]-0.027), tvec[0]*1.15, tvec[2]+0.03)
                drill.set_rpy(3.14-pitch, -roll, yaw+3.14)

                #camera.set_pos(0, 0, -30)
                rotation = camera.get_rpy()

                # print("Drill Position: " + str(drill.get_pos()))
                # print("Drill Rotation: " + str(drill.get_rpy()))

                # print("Camera Position: " + str(camera.get_pos()))
                # print("Camera Rotation: " + str(rotation))
                #camera.set_rpy(0, 0, 0)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec.reshape((3, 1)), 0.03)


                roll_deg = np.degrees(roll)
                pitch_deg = np.degrees(pitch)
                yaw_deg = np.degrees(yaw)
                # prevz = z
                #prevquat = quat_curr
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ArucoDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()



import cv2
import numpy as np






