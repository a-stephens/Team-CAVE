import cv2
import numpy as np
import matplotlib.pyplot as plt
import timeit
import time
from Checker_Images.CameraCalibration import get_params_from_file
import rospy
from std_msgs.msg import Float64MultiArray

class AV:
    def __init__(self, img, arr, ippx, ippy, shape, id):
        self.angle_val = 1      # Stores if angle is positive or negative
        self.img = img
        self.id = id
        self.array = [[np.round((((arr[0][0]*2)-shape[1])*ippx)/2, 2), np.round(((shape[0]-(arr[0][1]*2))*ippy)/2, 2)],
                      [np.round((((arr[1][0]*2)-shape[1])*ippx)/2, 2), np.round(((shape[0]-(arr[1][1]*2))*ippy)/2, 2)],
                      [np.round((((arr[2][0]*2)-shape[1])*ippx)/2, 2), np.round(((shape[0]-(arr[2][1]*2))*ippy)/2, 2)],
                      [np.round((((arr[3][0]*2)-shape[1])*ippx)/2, 2), np.round(((shape[0]-(arr[3][1]*2))*ippy)/2, 2)]]
        self.pix_center = [int((arr[0][0] + arr[1][0] + arr[2][0] + arr[3][0]) / 4),
                           int((arr[0][1] + arr[1][1] + arr[2][1] + arr[3][1]) / 4)]
        self.actual = []
        self.__av_center()

    def __av_center(self):
        self.actual = np.array([np.round((self.array[0][0] + self.array[1][0]
                                + self.array[2][0] + self.array[3][0]) / 4, 2),
                                np.round((self.array[0][1] + self.array[1][1]
                                + self.array[2][1] + self.array[3][1]) / 4, 2)])

    def __get_av_vect(self):

        mid12 = np.array([np.round((self.array[1][0] + self.array[2][0]) / 2, 2),
                          np.round((self.array[1][1] + self.array[2][1]) / 2, 2)])
        av_vect = mid12 - self.actual

        if av_vect[1] > 0:
            self.angle_val = 1
        else:
            self.angle_val = -1

        return av_vect

    def __get_angle(self):
        u_vect_1 = [1, 0] / np.linalg.norm([1, 0])
        u_vect_2 = self.__get_av_vect() / np.linalg.norm(self.__get_av_vect())
        dot_product = np.dot(u_vect_1, u_vect_2)
        angle = self.angle_val * round((180 * np.arccos(dot_product)) / np.pi, 2)
        return angle

    def get_id(self):
        return self.id

    def get_pos(self):
        av_wrld = np.array([self.actual[0] - np.round(4.65*np.cos(np.pi*self.__get_angle()/180), 2),
                            self.actual[1] - np.round(4.65*np.sin(np.pi*self.__get_angle()/180), 2),
                            self.__get_angle()], dtype=float)
        return av_wrld

    def plot(self):
        cv2.putText(self.img, str(self.get_pos()), (self.pix_center[0] - 150,
                    self.pix_center[1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0),
                    2, cv2.LINE_AA)

def field_cv_pub():

    # initialize ros node
    rospy.init_node("field_cv")

    # Defining ARUCO libraries
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    arucoParams = cv2.aruco.DetectorParameters_create()

    # Starting system timer
    start = timeit.default_timer()

    # Setting up camera capture parameters
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    check, img = capture.read()

    # img = cv2.imread('Field2.jpg')          # Use if no camera

    # Grabs defishing variables and resizes the image
    K, D, w, h = get_params_from_file("Checker_Images/camParams.txt")
    img = cv2.resize(img, (w, h))
    newcammtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0, (w, h))
    img_new = cv2.undistort(img, K, D, None, newcammtx)
    x, y, w, h = [265, 0, 1420, h]
    img_new = img_new[y:y+h, x:x+w]

    # get shape of image
    shape = img_new.shape

    # calculate inches per pixel
    ippx = 144/shape[1]
    ippy = 94/shape[0]

    # convert to grayscale
    gray = cv2.cvtColor(img_new, cv2.COLOR_BGR2GRAY)

    # apply gaussian blur
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    # cv2.imwrite('blur.jpg', blur)

    # apply canny edge detection and save
    edges = cv2.Canny(blur, 80, 240, apertureSize=3)
    # cv2.imwrite('edges.jpg', edges)

    # set parameters for hough lines and generate world lines
    minLL = 200
    maxLG = 5
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 15, minLL, maxLG)
    wrld_lines = np.empty([len(lines), 4], dtype=float)
    for i in range(len(lines)-1):
        for x1, y1, x2, y2 in lines[i]:
            cv2.line(img_new, (x1, y1), (x2, y2), (0, 255, 0), 2)
            wrld_lines[i] = np.array([(((x1*2)-shape[1])*ippx)/2,
                                    ((shape[0] - y1*2)*ippy)/2,
                                    ((x2*2-shape[1])*ippx)/2,
                                    ((shape[0] - y2*2)*ippy)/2], dtype=float)

    # Store world lines to text file
    f = open('world_lines.txt', 'w')
    for i in range(len(wrld_lines)-1):
        for j in range(4):
            f.write(str(wrld_lines[i][j]) + ' ')
        f.write('\n')

    # Detect ARUCO markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)

    publishers = []
    avs = []
    # Plot avs if they are found
    if ids is not None:
        av1 = AV(img_new, corners[0][0], ippx, ippy, shape, ids[0])
        print(av1.get_pos())                                            # Prints the coords and heading angle for av1
        av1.plot()                                                      # Plots the coords and heading angle for av1
        pub = rospy.Publisher("pose{}".format(ids[0]))
        publishers.append(pub)
        avs.append(av1)
        if len(corners) > 1:
            av2 = AV(img_new, corners[1][0], ippx, ippy, shape, ids[1])
            pub = rospy.Publisher("pose{}".format(ids[0]))
            print(av2.get_pos())                                        # Prints the coords and heading angle for av2
            av2.plot()                                                  # Plots the coords and heading angle for av2
            publishers.append(pub)
            avs.append(av2)

    for i, pub in enumerate(publishers):
        pose = avs[i].get_pos()
        msg = Float64MultiArray()
        msg.data = [pose[0], pose[1], pose[2]]
        pub.publish(msg.data)

    # Plots figure with dimensions of the field
    plt.figure()
    plt.title('Field plot')
    ext = [-72.0, 72.0, -47.00, 47.0]
    plt.imshow(img_new, zorder=0, extent=ext)
    plt.show()

    # Stops initiation timer
    stop = timeit.default_timer()
    print('Time: ', stop - start)

    if ids is not None:
        while True:
            start = timeit.default_timer()
            ret, frame = capture.read()
            frame_dst = cv2.undistort(frame, K, D, None, newcammtx)

            x, y, w, h = [265, 0, 1420, 1080]
            frame_dst = frame_dst[y:y + h, x:x + w]

            grey = cv2.cvtColor(frame_dst, cv2.COLOR_BGR2GRAY)
            (corners, ids, rejected) = cv2.aruco.detectMarkers(grey, arucoDict, parameters=arucoParams)

            if ids is not None:
                frame_test = cv2.aruco.drawDetectedMarkers(frame_dst, corners, ids)
                car1 = AV(frame_dst, corners[0][0], ippx, ippy, shape, ids[0])
                pose = car1.get_pos()
                msg = Float64MultiArray()
                msg.data = [pose[0], pose[1], pose[2]]
                publishers[0].publish(msg)
                print(car1.get_pos())
                car1.plot()
                if len(corners) > 1:
                    car2 = AV(frame_dst, corners[1][0], ippx, ippy, shape, ids[1])
                    print(car2.get_pos())
                    pose = car2.get_pos()
                    msg = Float64MultiArray()
                    msg.data = [pose[0], pose[1], pose[2]]
                    publishers[1].publish(msg)
                    car2.plot()
                del ids, corners

            cv2.imshow('pls wrk', frame_dst)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(.04)
            stop = timeit.default_timer()
            print('Time: ', stop - start)


    capture.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    field_cv_pub()