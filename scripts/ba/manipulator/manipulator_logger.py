import logging
import sys
from ba.features.logging.formatter import FORMATTER

LOGPATH = "/tmp/manipulator.log"
MANIPULATORLOGGER = logging.getLogger(__name__)
MANIPULATORLOGGER.setLevel(logging.INFO)
FILEHANDLER = logging.FileHandler(LOGPATH)
FILEHANDLER.setLevel(logging.INFO)
FILEHANDLER.setFormatter(FORMATTER)
CONSOLEHANDLER = logging.StreamHandler(sys.stdout)
CONSOLEHANDLER.setFormatter(FORMATTER)
MANIPULATORLOGGER.addHandler(CONSOLEHANDLER)
MANIPULATORLOGGER.addHandler(FILEHANDLER)