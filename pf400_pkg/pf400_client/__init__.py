"""
Python library to control an PF400 robot through its TCP/IP interface
"""
from pf400_client.pf400_client import PF400  # MAIN CLASS
__version__ = "0.0.1"

try:
    from pf400_client.rpl_pf400 import RPL_PF400
except ImportError as ex:
    print("Exception while importing RPL base robot, disabling use of RPL robot functions")
    Robot = PF400