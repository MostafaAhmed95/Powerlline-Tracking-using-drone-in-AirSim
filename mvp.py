import airsim
import os
import math
import time
from numpy import radians
import cv2
import numpy as np
from numpy import degrees
from scipy.spatial.transform import Rotation as R


def draw_lines(img, houghLines, color=[0, 255, 0], thickness=10):
    # just for debug purposes
    global img_counter
    rho, theta = houghLines[0][0][0], houghLines[0][0][1]
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a * rho
    y0 = b * rho
    x1 = int(x0 + 1000 * (-b))
    y1 = int(y0 + 1000 * (a))
    x2 = int(x0 - 1000 * (-b))
    y2 = int(y0 - 1000 * (a))
    #cv2.line(img, (x1, y1), (x2, y2), color, thickness)
    cv2.imwrite("img"+str(img_counter)+".png",img)
    cv2.line(img, (x1, y1), (x2, y2), color, thickness)
    img_counter +=1

def capture_image(client):
    #function to capture an image
    png_image = client.simGetImage("front_center_custom", airsim.ImageType.Scene)
    image = cv2.imdecode(airsim.string_to_uint8_array(png_image), cv2.IMREAD_UNCHANGED)
    return image

def new2_image_angle(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (9, 9), 0)
    #changing type to unit8: https://stackoverflow.com/questions/19103933/depth-error-in-2d-image-with-opencv-python
    blurred_image2 = (blurred_image * 255).astype(np.uint8)
    edges_image = cv2.Canny(blurred_image2, 50, 120)
    hough_lines = cv2.HoughLines(edges_image, rho_resolution, theta_resolution, threshold)
    draw_lines(image, hough_lines)
    #print(type(hough_lines))
    if hough_lines.size == 0:
        print("empty")
    try:
        # TODO solve the not recognized line problem
        print("image: ", degrees(hough_lines[0][0][1]))
        new_angle = degrees(hough_lines[0][0][1])
        x0 = client.simGetObjectPose("drone_1").orientation.x_val
        y0 = client.simGetObjectPose("drone_1").orientation.y_val
        z0 = client.simGetObjectPose("drone_1").orientation.z_val
        w0 = client.simGetObjectPose("drone_1").orientation.w_val
        l = R.from_quat([x0, y0, z0, w0])
        current_angle = int(l.as_euler('xyz', degrees=True)[2])
        print("drone current_angle: ", current_angle)
        if new_angle > 90:
            new_angle = new_angle - 180
            new_angle = current_angle + new_angle

        else:
            new_angle = current_angle + new_angle
        print("applied to the drone:", new_angle)
        return radians(new_angle)
    except:
        return current_angle

def set_new_yaw(yaw):
    #this function takes the angle and change the drone yaw
    global vx, vy
    #print("the angle: ",yaw)
    vx = math.cos(yaw)
    vy = math.sin(yaw)
    #it should be
    #vx = speed * math.cos(yaw)
    #vy = speed * math.sin(yaw)

img_counter = 0
duration = 5
speed = 1
vx = 0
vy = 0
z = 0
rho_resolution = 1
theta_resolution = np.pi/180
threshold = 155

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()

#reach to the power line
client.moveToPositionAsync(52, -52, -60, 5).join()
time.sleep(5)

#our loop
while True:
    img = capture_image(client)
    yaw = new2_image_angle(img)
    set_new_yaw(yaw)
    z = client.simGetObjectPose("drone_1").position.z_val
    client.moveByVelocityZAsync(vx, vy, z, 3, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
    print("3 seconds passed")