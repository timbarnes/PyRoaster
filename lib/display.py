""" display.py

    Setup and update the display.
    Supports a 4x20 I2C LCD display.
    
    This may not be necessary if a suitable library can be found.
"""

from machine import I2C
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

class Display(object):
    """
    Administer LCD display(s) on the I2C bus.
    """    
    
    def __init__(self, I2C, address, rows, cols):
        """
        Create a display instance. The I2C connection is created outside and passed in.
        """
        self._I2c = I2C
        self._screen = I2cLcd(self._I2c, address, rows, cols)
        self._screen.clear()
        
    

