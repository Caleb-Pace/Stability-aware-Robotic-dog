#!/usr/bin/env python3
import cv2
import sys, time
from cv2.typing import MatLike
import numpy as np
# from unitree_sdk2py.core.channel import ChannelFactoryInitialize
# from unitree_sdk2py.go2.video.video_client import VideoClient
# TODO: Remove, Placeholder
class VideoClient():
    def GetImageSample(self):
        return -1, []


fps = 30.0
using_test_camera = True


def _unitree_camera_client_init() -> VideoClient:
    # # TODO: See if required
    # channel_id = 0
    # ChannelFactoryInitialize(channel_id)

    # Create video client
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()

    return client

def _get_frame_unitree(client:VideoClient) -> MatLike|None:
    code, data = client.GetImageSample()
    if code != 0:
        print(f"[Error] Unitree Image sampler error! (Code: {code})")
        return None
    
    # Convert to numpy image
    image_data = np.frombuffer(bytes(data), dtype=np.uint8)
    frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

    return frame

def _test_camera_init() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("[Error] Could not open video device.")
        exit()

    return cap

def _get_frame_test(cap:cv2.VideoCapture) -> MatLike|None:
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame. Exiting...")
        return None
    
    return  frame

def main():
    delay = 1.0 / fps
    
    client = None
    cap = None
    _cam_width = 1920
    _cam_height = 1080
    if not using_test_camera:
        client = _unitree_camera_client_init()

        sample_frame = _get_frame_unitree(client)
        if sample_frame is not None:
            _cam_height, _cam_width, _ = sample_frame.shape
        else:
            print("  [Error] Could not get frame!")

        print("[Recorder] Unitree camera, initialised!")
    else:
        cap = _test_camera_init()

        sample_frame = _get_frame_test(cap)
        if sample_frame is not None:
            _cam_height, _cam_width, _ = sample_frame.shape
        else:
            print("  [Error] Could not get frame!")

        print("[Recorder] Local camera, initialised!")

    # Video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, fps, (_cam_width, _cam_height))

    print("[Recorder] Recording loop, started!")
    try:
        frame_count = 0
        while True:
            frame = None
            if client is not None:
                frame = _get_frame_unitree(client)
            if cap is not None:
                frame = _get_frame_test(cap)

            if frame is None:
                break

            # Write frame to file
            out.write(frame)
            frame_count += 1

            # Time/End check
            if frame_count == (4 * fps):
                break
            
            # Frame delay
            time.sleep(delay)
        print("[Recorder] Recording loop, finished!")
    finally:
        out.release()
        if cap is not None:
            cap.release()
        print("[Recorder] Resources have been released!")

if __name__ == "__main__":
    main()
