#!/usr/bin/env python3
import cv2
import sys, time
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

fps = 30.0

_cam_width = 1920
_cam_height = 1080

def main():
    print("Hello, World!")

    delay = 1.0 / fps

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    # Video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, fps, (_cam_width, _cam_height))

    # Create a video client
    client = VideoClient()
    client.SetTimeout(3.0)
    client.Init()

    print("Start")  # TODO: Remove, for debugging

    frame_count = 0
    # Request normal when code==0
    while True:
        # Get Image data from Go2 robot
        code, data = client.GetImageSample()

        if code != 0:
            print("Get image sample error. code:", code)
            break

        # Convert to numpy image
        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        frame = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

        out.write(frame)
        frame_count += 1

        if frame_count == (4 * fps):
            break

        # Delay
        time.sleep(delay)

    print("Done")  # TODO: Remove, for debugging

if __name__ == "__main__":
    main()
