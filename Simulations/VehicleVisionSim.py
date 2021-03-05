import cv2
import numpy as np

def Select_Pts(event, x, y, flags, param):
    mod_img = param[1].copy()
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(param[0]) < 4:
            param[0].append([x, y])
        print(param[0])
        
    if event == cv2.EVENT_RBUTTONDOWN:
        if len(param[0]) != 0:
            param[0].pop()
        print(param[0])
    for pt in param[0]:
        mod_img = cv2.circle(mod_img, (pt[0], pt[1]), 5, (255, 0, 0), -1)
        cv2.imshow("dash", mod_img)

img = cv2.imread("dashcam.jpg")
#img = cv2.resize(img, (int(0.25*img.shape[0]), int(0.25*img.shape[1])))

width = img.shape[1]
height = img.shape[0]

dst_pts = [
    [0,0],
    [width,0],
    [width, height],
    [0, height]
]
src_pts = []

cv2.imshow("dash", img)

print("Left click to add point. Right click to remove point.")
print("select points going from top left, then proceeding clockwise")
print("Press any key to finalize points")
cv2.setMouseCallback("dash", Select_Pts, [src_pts, img])

cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imshow("dash_original", img)

M = cv2.getPerspectiveTransform(np.array(src_pts, dtype=np.float32),
                                np.array(dst_pts, dtype=np.float32))
warped_img = cv2.warpPerspective(img, M, (width, height),
                                flags=cv2.INTER_LINEAR, 
                                borderMode=cv2.BORDER_CONSTANT)

cv2.imshow("warped", warped_img)

print("Press any key to exit.")
cv2.waitKey(0)
cv2.destroyAllWindows()
