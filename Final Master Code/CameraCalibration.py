import cv2
import numpy as np
import os
import glob

################################################################################

def calibrate(path_to_calibration_imgs, board_size):
    """
    Description:\n
    Determines the camera intrinsics, distortion vector, image width, and image 
    height, to be used later for undistorting an image and mapping
    image coordinates to world coordinates.\n
    Inputs:\n
    path_to_calibration_imgs: path w/ wildcard character to all images 
                            to be used by calibration.\n
    board_size: size of chessboard/checkerboard used during
                process of taking calibration images.\n
    Outputs:\n
    K: camera intrinsics matrix\n
    D: camera distortion vector\n
    img_w: image width used during calibration\n
    img_h: image height used during calibration\n
    """
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((board_size[0]*board_size[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:int(board_size[0]), 0:int(board_size[1])].T.reshape(-1,2)

    obj_points = []
    img_points = []

    images = glob.glob(path_to_calibration_imgs)

    img_w = None
    img_h = None

    for fname in images:
        img = cv2.imread(fname)

        img_h, img_w = img.shape[:2]

        gray = cv2.cvtColor(img ,cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, board_size, None)

        if ret == True:
            obj_points.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11,11),
                                        (-1,-1), criteria)

            img_points.append(corners)

            cv2.drawChessboardCorners(img, board_size, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)
    cv2.destroyAllWindows()

    ret, K, D, _, _ = cv2.calibrateCamera(obj_points, img_points,
                                          gray.shape[::-1], None, None)

    return K, D, img_w, img_h

def calibrate_to_file(path_to_calibration_imgs, board_size, file_name):
    """
    Description:\n
    the same as calibrate(), but writes outputs to a file.\n
    Inputs:\n
    path_to_calibration_imgs:   path w/ wildcard character to all images to be used by calibration.\n
    board_size: size of chessboard/checkerboard used during process of taking calibration images.\n
    Outputs:\n
    K: camera intrinsics matrix\n
    D: camera distortion vector\n
    img_w: image width used during calibration\n
    img_h: image height used during calibration
    """
    K, D, img_w, img_h = calibrate(path_to_calibration_imgs, board_size)

    f = open(file_name, 'w')
    for row in K:
        for col in row:
            f.write(str(col) + " ")
        f.write("\n")
    for item in D[0]:
        f.write(str(item) + " ")
    f.write("\n")
    f.write(str(img_w) + " " + str(img_h))
    f.close()

def get_params_from_file(file_name):
    """
    get_params_from_file(file_name)\n
    Retrieves camera parameters from provided file name.\n
    Parameters:\n
    file_name:\tstring representing name of file to read from\n
    Outputs:\n
    K:\tCamera intrinsics matrix\n
    D:\tCamera distortion vector\n
    img_w:\timage width from calibration images\n
    img_h:\timage height from calibration images
    """

    data = []
    with open(file_name) as f:
        line = f.readline()
        while line:
            data.append(line.strip())
            line = f.readline()
    for i in range(len(data)):
        data[i] = data[i].split()
    K = np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            K[i, j] = np.float64(data[i][j])
    D = np.zeros((4,1), np.float64)
    for i in range(4):
        D[i] = np.float64(data[3][i])
    img_w = int(data[4][0])
    img_h = int(data[4][1])

    return K, D, img_w, img_h

################################################################################

def main():
    BASE_PATH = os.path.join(os.getcwd())
    IMGS_PATH = os.path.join(BASE_PATH, "TopDownImages")
    IMG_PATH = os.path.join(IMGS_PATH, "Field1.jpg")

    CAMERA_VERIFY_BASE_PATH = os.path.join(os.getcwd())
    CAMERA_VERIFY_PATH = os.path.join(CAMERA_VERIFY_BASE_PATH, "Field_Checker")
    CAMERA_VERIFY_IMG_PATH = os.path.join(CAMERA_VERIFY_PATH, "*.jpg")

    CHECKERBOARD_SIZE = (4, 7)
    
    CAM_PARAMS_FNAME = "camParams.txt"

    # mtx, dist, w, h = calibrate(CAMERA_VERIFY_IMG_PATH, CHECKERBOARD_SIZE)
    calibrate_to_file(CAMERA_VERIFY_IMG_PATH, CHECKERBOARD_SIZE, CAM_PARAMS_FNAME)

    mtx, dist, w, h = get_params_from_file(CAM_PARAMS_FNAME)

    img = cv2.imread(IMG_PATH)
    img = cv2.resize(img, (w,h))

    newcammtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0, (w,h))

    dst = cv2.undistort(img, mtx, dist, None, newcammtx)

    print(mtx)
    print(dist)

    cv2.imshow('Source.jpg',img)
    cv2.imwrite('Und.jpg',dst)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()