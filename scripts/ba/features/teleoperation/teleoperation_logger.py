import logging
import sys
from ba.features.logging.formatter import FORMATTER

LOGPATH = "/tmp/teleoperation.log"
TELEOPLOGGER = logging.getLogger(__name__)
TELEOPLOGGER.setLevel(logging.INFO)
FILEHANDLER = logging.FileHandler(LOGPATH)
FILEHANDLER.setLevel(logging.INFO)
FILEHANDLER.setFormatter(FORMATTER)
CONSOLEHANDLER = logging.StreamHandler(sys.stdout)
CONSOLEHANDLER.setFormatter(FORMATTER)
TELEOPLOGGER.addHandler(CONSOLEHANDLER)
TELEOPLOGGER.addHandler(FILEHANDLER)