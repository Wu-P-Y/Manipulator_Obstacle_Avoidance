from asyncio.windows_events import NULL
from email.mime import image
from re import X, template
import sys
from turtle import color
import numpy as np
import pyzed.sl as sl
import cv2


def matchTemplate(image):
    # 模板匹配寻找机器人工作区域
    src = image
    if src is None:
        print('Could not open or find the image')
        exit(0)
    print(src.shape)

    template = cv2.imread('D:/bishe/code/scripts/edge_detect/template.jpg', 1)
    if template is None:
        print('Could not open or find the template')
        exit(0)
    print(template.shape)

    h, w = template.shape[:2]

    # 模板匹配
    # methods = ['cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED', 'cv2.TM_CCORR', 'cv2.TM_CCORR_NORMED',
    #           'cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED']
    res = cv2.matchTemplate(src, template, cv2.TM_CCOEFF_NORMED)
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(res)
    
    # 图像左上和右下的坐标
    topLeft = maxLoc
    bottomRight = (topLeft[0] + w, topLeft[1] + h)

    output = src[topLeft[1]:bottomRight[1], topLeft[0]:bottomRight[0]]

    # cv2.imshow(output)

    return output, topLeft[0], topLeft[1]

def getObstacleCenter(img:np.array, threshold = 50, max_threshold = 255, area = 50):
    # 提取障碍物重心坐标
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    # 彩色图转灰度图

    edge = cv2.Canny(gray, threshold, max_threshold, apertureSize = 3)  # 提取边缘
    cv2.imshow("edges", edge)

    contours, hierarchy = cv2.findContours(edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)    # 寻找轮廓


    # 计算凸包
    hull = []
    res = []
    for c in range(len(contours)):
        hull.append(cv2.convexHull(contours[c], clockwise = False))

    # 绘制凸包
    for c in range(len(contours)):
        if cv2.contourArea(contours[c]) < area:
            continue

        res.append(hull[c])

        drawing = cv2.drawContours(img, contours, c, (0, 255, 0), 3)
        drawing = cv2.drawContours(img, hull, c, (0, 255, 0), 3)

    cv2.imshow("drawing", img)

    # 计算物体位置信息， 返回障碍物重心像素坐标
    centers = []
    for c in range(len(contours)):
        if cv2.contourArea(contours[c]) < area:
            continue
        
        m = cv2.moments(contours[c], False)
        center = [m['m10'] / m['m00'], m['m01'] / m['m00']]
        centers.append(center)
    
    return centers

        

def main():

    def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
        global x0, y0
        if event == cv2.EVENT_LBUTTONDBLCLK:
            x0 = x
            y0 = y
            print(x, y)

    zed = sl.Camera()

    # 设置初始化参数
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    # 打开相机
    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)

    # 设置运行参数
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    # 准备一个新的图片格式来保存原图片一半分辨率的图片
    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width / 2
    image_size.height = image_size.height / 2

    # 声明 sl.mat 矩阵
    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    depth_image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)
    point_cloud = sl.Mat()


    cv2.namedWindow('src')
    cv2.setMouseCallback('src', on_EVENT_LBUTTONDOWN)

    key = ' '
    while key != 113:
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS :
            # 抓取左相机图片和深度图
            zed.retrieve_image(image_zed, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            zed.retrieve_image(depth_image_zed, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
            # 抓取 RGBA 点云
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, image_size)

            # 将 sl.mat 格式图片转化为 opencv 的 numpy array 格式
            image_ocv = image_zed.get_data()
            depth_image_ocv = depth_image_zed.get_data()

            cv2.imshow("ORG", image_ocv)

            
            # 键盘输入's'，保存图片并获取障碍物坐标
            while (key == 115):
                cv2.imwrite("D:/bishe/code/scripts/edge_detect/img.jpg", image_ocv)

                img = cv2.imread("D:/bishe/code/scripts/edge_detect/img.jpg")

                cv2.imshow('image', img)
                
                src, topleft_x, topleft_y = matchTemplate(img)
                cv2.imshow("src", src)

                centers = getObstacleCenter(src)
                for c in range(len(centers)):
                    err, point_cloud_value = point_cloud.get_value(centers[c][0] + topleft_x, centers[c][1] + topleft_y)
                    # if point_cloud_value[0] != "nan" and point_cloud_value[0] != 0 and point_cloud_value[2] < 2000:
                    print(point_cloud_value[0], " ", point_cloud_value[1], " ", point_cloud_value[2], "\n")

                print("-----------------------------------------\n")
                
                key = ' '

            key = cv2.waitKey(10)

    # 若手工取点请取消该部分的注释
    # if x0 != 0:
    #     err, point_cloud_value = point_cloud.get_value(x0 + topleft_x, y0 + topleft_y)
    #     print(point_cloud_value[0], " ", point_cloud_value[1], " ", point_cloud_value[2], "\n")

    # 关闭相机
    cv2.destroyAllWindows()
    zed.close()

if __name__ == "__main__":
    main()
                    


