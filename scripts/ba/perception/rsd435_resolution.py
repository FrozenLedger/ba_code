from enum import Enum

class RealSenseD435DepthResolution(Enum):
    rs256x144 = (256,144)
    rs480x270 = (480,270)
    rs640x360 = (640,360)
    rs640x480 = (640,480)
    rs848x480 = (848,480)
    rs1280x720 = (1280,720)

class RealSenseD435ColorResolution(Enum):
    rs424x240 = (424,240)
    rs640x480 = (640,480)
    rs1280x720 = (1280,720)
    rs1920x1080 = (1920,1080)