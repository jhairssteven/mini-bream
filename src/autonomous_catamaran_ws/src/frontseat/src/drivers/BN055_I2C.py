#! /usr/bin/python3
import adafruit_bno055
import board

class IMUDriver_BNO055_I2C:
    """ Device uses relative heading. Heading values would be given
      relative to the initial heading direction. Device should be then pointing north
      upon initialization for correct values."""
    
    def __init__(self) -> None:
        self._IMUDriver = adafruit_bno055.BNO055_I2C(board.I2C())
        self.previousHeading = 0

    def getHeading(self):
        """ Returns the current bearing in NED frame (0° North, 90° East) 
        or  previous reading if valid value was not read. """

        compass_bearing_deg = self._IMUDriver.euler[0]
        """ if (compass_bearing_deg >= 180):
        compass_bearing_deg -= 180
        else:
        compass_bearing_deg += 180 """

        if isinstance(compass_bearing_deg, float) == True:
            self.previousHeading = compass_bearing_deg
        
        return self.previousHeading
