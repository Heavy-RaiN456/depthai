# -*- coding: utf-8 -*-
"""
Created on Fri Dec 17 17:05:03 2021

@author: bezti
"""


import depthai as dai 
import contextlib
import cv2

def manip_cam_config(key):
    cfg = dai.ImageManipConfig()
    if key== ord("c"):
        print("set cfg manip: warp with four points")
        pt0 = dai.Point2f()
        pt0.x = 0 
        pt0.y = 0
        pt1 = dai.Point2f()
        pt1.x = 1 
        pt1.y = 0
        pt2 = dai.Point2f()
        pt2.x = 1 
        pt2.y = 1
        pt3 = dai.Point2f()
        pt3.x = 0 
        pt3.y = 1
        
        points2List = [pt2, pt3, pt0, pt1] #3. rotate 180
        
        cfg.setWarpTransformFourPoints(points2List,True)
    return cfg
    
def fix_focus(lensPos=150):
    ctrl = dai.CameraControl()
    ctrl.setManualFocus(lensPos)
    return ctrl
    
def make_pipeline():
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    manip_cam = pipeline.create(dai.node.ImageManip)
    xout_cam = pipeline.createXLinkOut()
    manip_camOut = pipeline.create(dai.node.XLinkOut)
    manip_camCfg = pipeline.create(dai.node.XLinkIn)

#'''properties'''
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setInterleaved(False)
    cam.setPreviewSize(640,480) #320,240
    
    #cam.initialControl.setManualFocus(100) # value from 0 - 255
    #cam.initialControl.setAutoFocusMode(dai.RawCameraControl.AutoFocusMode.OFF)
    manip_cam.setMaxOutputFrameSize(2000 * 1500 * 3)

#'''link'''
    cam.preview.link(manip_cam.inputImage)
    cam.preview.link(xout_cam.input)

    manip_camCfg.out.link(manip_cam.inputConfig)
    manip_cam.out.link(manip_camOut.input)
    
    
#'''message transform'''
    xout_cam.setStreamName('cam_out')
    
    manip_camCfg.setStreamName("manip_camCfg_in")
    manip_camOut.setStreamName("manip_camOut")
    
    return pipeline

fourcc = cv2.VideoWriter_fourcc(*'MP4V') #(*'avc1')
out1 = cv2.VideoWriter('output1.mp4', fourcc, 20.0, (640, 480))
out2 = cv2.VideoWriter('output2.mp4', fourcc, 20.0, (640, 480))
q_rgb_list = []
with contextlib.ExitStack() as stack:
    
    device_infos = dai.Device.getAllAvailableDevices()
    if len(device_infos) == 0:
        raise RuntimeError("No devices found!")
    else:
        print("Found", len(device_infos), "devices")
    device_infos_shelf = []  
    _,device_info1 = dai.Device.getDeviceByMxId("172.16.4.142")
    _,device_info2 = dai.Device.getDeviceByMxId("172.16.4.140")
    device_infos_shelf.append(device_info1)
    device_infos_shelf.append(device_info2)
    for device_info in device_infos_shelf:
        # Note: the pipeline isn't set here, as we don't know yet what device it is.
        # The extra arguments passed are required by the existing overload variants
        openvino_version = dai.OpenVINO.Version.VERSION_2021_4
        usb2_mode = False
        
        device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))

        # Note: currently on POE, DeviceInfo.getMxId() and Device.getMxId() are different!
        print("=== Connected to " + device_info.getMxId())
        mxid = device.getMxId()
        cameras = device.getConnectedCameras()
        usb_speed = device.getUsbSpeed()
        print("   >>> MXID:", mxid)
        print("   >>> Cameras:", *[c.name for c in cameras])
        print("   >>> USB speed:", usb_speed.name)

        device_type = "unknown"
        if   len(cameras) == 1: device_type = "OAK-1"
        elif len(cameras) == 3: device_type = "OAK-D"
        # If USB speed is UNKNOWN, assume it's a POE device
        if usb_speed == dai.UsbSpeed.UNKNOWN: device_type += "-POE"

        # Get a customized pipeline based on identified device type
        pipeline = make_pipeline()
        print("   >>> Loading pipeline for:", device_type)
        device.startPipeline(pipeline)

        # Output queue will be used to get the rgb frames from the output defined above
        q_rgb = device.getOutputQueue(name="cam_out", maxSize=1, blocking=True)
        q_rgb_manip = device.getOutputQueue(name="manip_camOut", maxSize=1, blocking=True)
        qManipCfg = device.getInputQueue(name="manip_camCfg_in")
        cfg = manip_cam_config(ord("c"))
        qManipCfg.send(cfg)
        stream_name = "rgb-" + mxid + "-" + device_type
        q_rgb_list.append((q_rgb, stream_name,q_rgb_manip,qManipCfg))

    while True:
        for q_rgb, stream_name,q_rgb_manip,qManipCfg in q_rgb_list:
            
            in_rgb = q_rgb.tryGet()
            in_manipRbg = q_rgb_manip.tryGet()
            if in_rgb is not None:
                cam_frame = in_rgb.getCvFrame()
                cv2.imshow(stream_name, cam_frame)
            if in_manipRbg is not None:
                cam_frame_manip = in_manipRbg.getCvFrame()
                cv2.imshow(stream_name+"-manip", cam_frame_manip)
                if stream_name.find("600") == -1:
                    out1.write(cam_frame_manip)
                else:
                    out2.write(cam_frame_manip)
        key = cv2.waitKey(1)
        if key==ord('q'):
            break
'''
If this work fine -> To set a static IPv4 address 
https://docs.luxonis.com/projects/api/en/latest/samples/bootloader/poe_set_ip/
https://docs.luxonis.com/en/latest/pages/tutorials/getting-started-with-poe/

'''
            