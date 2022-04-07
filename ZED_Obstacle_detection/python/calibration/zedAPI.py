import numpy as np
import pyzed.sl as sl
import cv2

def main():

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

    calibration_params = zed.get_camera_information().camera_configuration.calibration_parameters

    print(calibration_params.T.z)

    key = ' '
    while key != 113:
        key = cv2.waitKey(10)

    zed.close()

if __name__ == "__main__":
    main()