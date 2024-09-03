#!/usr/bin/env python3
import asyncio
import cv2
import numpy as np
from av import VideoFrame
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.signaling import BYE, TcpSocketSignaling

class VideoStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)  # Adjust for Pi camera

    async def recv(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = np.flip(frame, axis=-1)  # Flip frame horizontally
        pts, time_base = await self.next_timestamp()
        return VideoFrame.from_ndarray(frame, format="rgb24", pts=pts, time_base=time_base)

async def run(pc, signaling):
    await signaling.connect()
    await signaling.send(pc.localDescription)

    while True:
        obj = await signaling.receive()
        if isinstance(obj, RTCSessionDescription):
            await pc.setRemoteDescription(obj)

            if obj.type == "offer":
                await pc.setLocalDescription(await pc.createAnswer())
                await signaling.send(pc.localDescription)
        elif obj is BYE:
            break

if __name__ == "__main__":
    pc = RTCPeerConnection()
    pc.addTrack(VideoStreamTrack())
    signaling = TcpSocketSignaling("localhost", 8080)
    
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run(pc, signaling))
    except KeyboardInterrupt:
        pass
    finally:
        loop.run_until_complete(pc.close())
        loop.run_until_complete(signaling.close())
