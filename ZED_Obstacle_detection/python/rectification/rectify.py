import cv2
import numpy as np

# 左相机参数
camera_matrix0 = np.array([[7.595583645934358e+02, 0, 0],
                            [0, 7.634387607529503e+02, 0],
                            [6.414913244968774e+02, 3.975647449415878e+02, 1]]).reshape((3, 3))

distortion0 = np.array([0.059383074533725, -0.176556695042484, 0, 0, 0])

# 右相机参数
camera_matrix1 = np.array([[7.365598651612320e+02, 0, 0],
                            [0, 7.342040028623361e+02, 0],
                            [5.785294166901433e+02, 3.740621057429653e+02, 1]]).reshape((3, 3))

distortion1 = np.array([0.002972925238633, -0.089351898203001, 0, 0, 0])

# 双目相机外参
R = np.array([[0.989866340395966,-9.683780391837979e-05,-0.142002178763347],
                [-0.002011851716252,0.999889836174897,-0.014706052047000],
                [0.141987959361946,0.014842713248493,0.989757098110265]]).reshape((3, 3))

T = np.array([-1.254917614044857e+02, -0.888258249584530, -5.373751386652202])



def cat2images(limg, rimg):
    HEIGHT = limg.shape[0]
    WIDTH = limg.shape[1]
    imgcat = np.zeros((HEIGHT, WIDTH*2+20,3))
    imgcat[:,:WIDTH,:] = limg
    imgcat[:,-WIDTH:,:] = rimg
    for i in range(int(HEIGHT / 32)):
        imgcat[i*32,:,:] = 255 
    return imgcat

def main():
    left_image = cv2.imread("./img/img_l.jpg")
    right_image = cv2.imread("./img/img_r.jpg")

    imgcat_source = cat2images(left_image,right_image)

    HEIGHT = left_image.shape[0]
    WIDTH = left_image.shape[1]

    cv2.imwrite('./res/src.jpg', imgcat_source )

    (R_l, R_r, P_l, P_r, Q, validPixROI1, validPixROI2) = \
        cv2.stereoRectify(camera_matrix0, distortion0, camera_matrix1, distortion1, np.array([WIDTH,HEIGHT]), R, T) # 计算旋转矩阵和投影矩阵

    (map1, map2) = \
        cv2.initUndistortRectifyMap(camera_matrix0, distortion0, R_l, P_l, np.array([WIDTH,HEIGHT]), cv2.CV_32FC1) # 计算校正查找映射表

    rect_left_image = cv2.remap(left_image, map1, map2, cv2.INTER_CUBIC) # 重映射

    (map1, map2) = \
        cv2.initUndistortRectifyMap(camera_matrix1, distortion1, R_r, P_r, np.array([WIDTH,HEIGHT]), cv2.CV_32FC1)

    rect_right_image = cv2.remap(right_image, map1, map2, cv2.INTER_CUBIC)

    imgcat_out = cat2images(rect_left_image,rect_right_image)
    cv2.imwrite('./res/imgcat_out.jpg', imgcat_out)

if __name__ == "__main__":
    main()
