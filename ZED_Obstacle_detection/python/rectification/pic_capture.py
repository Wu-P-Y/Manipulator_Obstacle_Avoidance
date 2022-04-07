import numpy as np
import pyzed.sl as sl
import cv2

def pic_capture():

    zed = sl.Camera()

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE

    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS :
        print(repr(err))
        zed.close()
        exit(1)

    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD

    img_l = sl.Mat()
    img_r = sl.Mat()
    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width / 2
    image_size.height = image_size.height / 2

    key = ' '

    while key != 113:
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(img_l, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            zed.retrieve_image(img_r, sl.VIEW.RIGHT, sl.MEM.CPU, image_size)
            image_ocv_l = img_l.get_data()
            image_ocv_r = img_r.get_data()
            
            cv2.imshow("img", image_ocv_l)

            while key == 115:
                cv2.imwrite(".\img\img_l.jpg", image_ocv_l)
                cv2.imwrite(".\img\img_r.jpg", image_ocv_r)

                key = ' '

        key = cv2.waitKey(10)

    zed.close()

def main():
    pic_capture()

if __name__ == "__main__":
    main()

