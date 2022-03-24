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

    img = sl.Mat()
    image_size = zed.get_camera_information().camera_resolution
    image_size.width = image_size.width / 2
    image_size.height = image_size.height / 2

    key = ' '

    while key != 113:
        err = zed.grab(runtime)
        if err == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(img, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            image_ocv = img.get_data()
            
            cv2.imshow("img", image_ocv)

            while key == 115:
                cv2.imwrite(".\images\img.jpg", image_ocv)

                key = ' '

        key = cv2.waitKey(10)

    zed.close()

def main():
    pic_capture()

if __name__ == "__main__":
    main()

