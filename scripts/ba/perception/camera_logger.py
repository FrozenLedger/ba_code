import logging
import sys
from ba.features.logging.formatter import FORMATTER

LOGPATH = "/tmp/camera.log"
CAMERALOGGER = logging.getLogger(__name__)
CAMERALOGGER.setLevel(logging.INFO)
FILEHANDLER = logging.FileHandler(LOGPATH)
FILEHANDLER.setLevel(logging.INFO)
FILEHANDLER.setFormatter(FORMATTER)
CONSOLEHANDLER = logging.StreamHandler(sys.stdout)
CONSOLEHANDLER.setFormatter(FORMATTER)
CAMERALOGGER.addHandler(CONSOLEHANDLER)
CAMERALOGGER.addHandler(FILEHANDLER)