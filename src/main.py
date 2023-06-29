#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from pymycobot import MyCobotSocket
import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.optimize import fsolve
import time

    
class MyNode:
    def __init__(self):
        rospy.init_node('my_node')
        #Create publisher for controlling the AGV
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist , queue_size=10)

        #Connecting to myCobot
        self.robot = MyCobotSocket('172.25.128.1',9000)
        self.robot.connect()
        self.robot.power_on()
        self.robot.sync_send_angles([0,0,0,0,0,0], 50)
        self.robot.sync_send_coords([200,-63.6,250,-90,0,-90], 50,0)

        #init camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)                
        self.profile = self.pipeline.start(self.config)
        

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = self.profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        # Create the detector parameters
        parameters = aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
      
        self.rate = rospy.Rate(1)  # 30hz


    #Helper functions to get rotation matrices
    def get_rx(self,ang):
        return np.array([[1,0,0],
                        [0,np.cos(ang),-np.sin(ang)],
                        [0,np.sin(ang),np.cos(ang)]])

    def get_ry(self,ang):
        return np.array([[np.cos(ang),0,np.sin(ang)],
                        [0,1,0],
                        [-np.sin(ang),0,np.cos(ang)]])

    def get_rz(self,ang):
        return np.array([[np.cos(ang),-np.sin(ang),0],
                        [np.sin(ang),np.cos(ang),0],
                        [0,0,1]])
    
    # Define the function that represents the system of equations
    def equations(self,variables):
        #TODO clean up the equations
        rx, ry, rz = variables
        rx_offset = -7*np.pi/180

        R_x = np.array([[1, 0, 0], [0, np.cos(rx+rx_offset), -np.sin(rx+rx_offset)], [0, np.sin(rx+rx_offset), np.cos(rx+rx_offset)]])
        R_y = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
        R_z = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])

        rr = np.dot(np.dot(R_z, R_y), R_x)       
        eqs = np.array(self.robot_cur_pos[0:3]) + np.dot(rr, np.array([0, 0, self.distance])) - np.array(self.target_pos[0:3])
        
        return eqs
    def moveAGV(self):
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_pub.publish(twist)     
        print("moving AGV")
        time.sleep(2)
        twist.linear.x = 0
        self.cmd_pub.publish(twist)
        print("stop AGV")        
        

    def run(self):
        while not rospy.is_shutdown():
            counter = 0
            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()
            

            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Get the intrinsics of the color camera
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

            # Get the camera matrix and distortion coefficients
            cameraMatrix = np.array([[color_intrinsics.fx, 0, color_intrinsics.ppx],
                                    [0, color_intrinsics.fy, color_intrinsics.ppy],
                                    [0, 0, 1]])

            distCoeffs = np.array(color_intrinsics.coeffs)

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
                    
            #get depth at the center
            depth = aligned_depth_frame.get_distance(320, 240)
            #print the depth
            

            # Detect the markers
            corners, ids, rejected = self.detector.detectMarkers(color_image)

            marker_size = 0.087
            marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

            #draw a cicle at center
            cv2.circle(color_image, (color_frame.width // 2, color_frame.height // 2), 5, (255, 0, 0), -1)
            # # Draw the detected markers and IDs on the image        
            #define the cache
            markers = []
            if(ids is not None):
                markers = []
                for i in range(len(ids)):
                    center = np.mean(corners[i][0], axis=0)
                    cv2.circle(color_image, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)         
                    res, rvecs, tvecs = cv2.solvePnP(marker_points, corners[i], cameraMatrix, distCoeffs,False,cv2.SOLVEPNP_IPPE_SQUARE)            
                    cv2.drawFrameAxes(color_image, cameraMatrix, distCoeffs, rvecs, tvecs, 0.1)
                    d = aligned_depth_frame.get_distance(int(center[0]), int(center[1]))
                    markers.append([ids[i], rvecs, tvecs, d])
                    # print("ids: ", ids[i], "rvecs: ", rvecs, "tvecs: ", tvecs,"depth: ", d)
                cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
                cv2.imshow('Align Example', color_image)
                # key = cv2.waitKey(0)
                # # Press esc or 'q' to close the image window
                # if key & 0xFF == ord('q') or key == 27:
                #     cv2.destroyAllWindows()
                #     break
            
            #move the robot facing to the marker; only support the first marker for now 
            # TODO 
            if(len(markers) > 0):
                self.robot_cur_pos = self.robot.get_coords()
                marker_t = np.array(markers[0][2]).reshape((3,)) #TODO clean up a bit
                marker_r = np.array(markers[0][1]).reshape((3,))
                
                #compute the marker pos with respect to the camera                
                R,_ = cv2.Rodrigues(marker_r)
                marker_pos_camera = np.dot(R.T,marker_t) 

                #rotate the marker pos to the camera frame
                R = self.get_rx(np.pi/2)
                marker_pos_camera = np.matmul(R,marker_pos_camera)
                print("Camera frame:",marker_pos_camera*100)
                
                #Apply the rotation so we get the marker pos wrt to the camera in world frame
                marker_pos_origin = np.matmul(self.get_rz(-np.pi/2),np.matmul(self.get_rx(-np.pi/2),marker_pos_camera)) 
                print("marker pos wrt origin:",marker_pos_origin*100)

                #distance between camera and marker
                marker_offset = marker_pos_origin*1000
                                
                print("fixed marker offset:",marker_offset)
                self.distance = np.linalg.norm(marker_offset)
                print("distance:",self.distance)    

                
                            

                #target_pos                 
                self.target_pos = marker_offset + self.robot_cur_pos[0:3]
                print("target pos:",self.target_pos)

                # #Assume we have a REALLY LONG pointer, compute there the current position is in the world frame(+z in the direction of the pointer)
                
                print("robot cur pos:",self.robot_cur_pos)

                pointer_offset =  np.matmul(self.get_rz(self.robot_cur_pos[5]*np.pi/180),np.matmul(self.get_ry(self.robot_cur_pos[4]*np.pi/180),np.matmul(self.get_rx(self.robot_cur_pos[3]* np.pi/180), np.array([0,0,self.distance]))))
                print("pointer offset:",pointer_offset)

                robot_cur_pos_plus_pointer = self.robot_cur_pos[0:3] + pointer_offset
                print("robot cur pos plus pointer:",robot_cur_pos_plus_pointer)

                # Solve the system of equations numerically using fsolve
                initial_guess = np.array([0, 0, 0])  # Initial guess for rx, ry, rz
                sol = fsolve(self.equations, initial_guess)
                print(sol)
                #rad to deg
                
                new_pointer_pos =  np.matmul(self.get_rz(sol[2]),np.matmul(self.get_ry(sol[1]),np.matmul(self.get_rx(sol[0]), np.array([0,0,self.distance]))))
                print("===")
                print("robot cur pos plus pointer:",pointer_offset)
                print("robot new pos with pointer:",new_pointer_pos)
                #Only check 2D distance
                new_dist = np.linalg.norm(pointer_offset[0:2] - new_pointer_pos[0:2])
                print("new dist:",new_dist)
                print("===")

                #only move the robot if the new predicted position is too far
                if(new_dist>30):
                    if(new_dist>300):
                        print("Error: the computed position is too far away from the current position")
                    else:
                        
                        sol = np.array(sol)                        
                        if(np.any(sol>=np.pi) or np.any(sol<=-np.pi)):
                            #TODO wrap this                            
                            print("invalid sol, skip")
                            continue
                        
                        #rad to deg
                        sol = np.array(sol) * 180 / np.pi
                        print("new sol:",sol)
                        
                        print("moving robot")
                        #move the robot
                        new_pos = [200,-63.6,250,sol[0],sol[1],sol[2]]
                        self.robot.sync_send_coords(new_pos, 20, 0)               
                        print("done moving robot")
                        
                else:
                    print("No need to move the robot, ready for the next step")
                    if(self.distance < 350):
                        #TODO clean up a bit
                        print("**ARRIVED :)**")
                        cv2.imshow('ARRIVED!!', color_image)
                        key = cv2.waitKey(0)
                        # Press esc or 'q' to close the image window
                        if key & 0xFF == ord('q') or key == 27:
                            cv2.destroyAllWindows()                    
                            break
                        break                                        
                    self.moveAGV()
            else:
                print("No marker detected, ready for the next step")
                counter+=1
                if(counter>100):
                    print("Error: no marker detected, abort")
                    cv2.imshow('Align Example', color_image)
                    key = cv2.waitKey(0)
                    # Press esc or 'q' to close the image window
                    if key & 0xFF == ord('q') or key == 27:
                        cv2.destroyAllWindows()                    
                        break
            
            
                
            self.rate.sleep()
        
if __name__ == '__main__':
    my_node = MyNode()
    my_node.run()
    
    
