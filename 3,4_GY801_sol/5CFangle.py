import smbus
import time
from math import *

bus = smbus.SMBus(1); # 0 for R-Pi Rev. 1, 1 for Rev. 2

EARTH_GRAVITY_MS2    = 9.80665 # m/s2

# the following address is defined by datasheet
ADXL345_ADDRESS         =    0x53 # I2C address

ADXL345_BW_RATE         =    0x2C # Data rate and power mode control 
ADXL345_POWER_CTL       =    0x2D # Power-saving features control 
ADXL345_DATA_FORMAT     =    0x31 # Data format control 
ADXL345_DATAX0          =    0x32
ADXL345_DATAX1          =    0x33
ADXL345_DATAY0          =    0x34
ADXL345_DATAY1          =    0x35
ADXL345_DATAZ0          =    0x36
ADXL345_DATAZ1          =    0x37
# ---------------------------------

# set value
ADXL345_SCALE_MULTIPLIER= 0.00390625    # G/LSP. 1/256 = 0.00390625
ADXL345_BW_RATE_100HZ   = 0x0A          # 0A = 0000 1111
ADXL345_MEASURE         = 0x08          # 08 = 0000 1000


L3G4200D_ADDRESS        =    0x69
L3G4200D_CTRL_REG1      =    0x20 # CTRL_REG1 (20h)
L3G4200D_CTRL_REG4      =    0x23 # CTRL_REG4 (23h)
L3G4200D_OUT_X_L        =    0x28 # X-axis angular rate data. 
L3G4200D_OUT_X_H        =    0x29 # The value is expressed as two's complement.
L3G4200D_OUT_Y_L        =    0x2A
L3G4200D_OUT_Y_H        =    0x2B
L3G4200D_OUT_Z_L        =    0x2C
L3G4200D_OUT_Z_H        =    0x2D



class IMU(object):

    def write_byte(self,adr, value):
        bus.write_byte_data(self.ADDRESS, adr, value)
    
    def read_byte(self,adr):
        # read_byte_data: read one byte from "self.ADDRESS", offset "adr"
        return bus.read_byte_data(self.ADDRESS, adr)

    def read_word(self,adr,rf=1):
        # rf=1 Little Endian Format, rf=0 Big Endian Format
        if (rf == 1):
            # read two byte data. ex: addr 50 and 51
            low = self.read_byte(adr)
            high = self.read_byte(adr+1)
        else:
            high = self.read_byte(adr)
            low = self.read_byte(adr+1)

        # combine "high" and "low" byte together.
        # shift "high" to left by 8 bits, then put "low"
        # like this: HHHH HHHH  LLLL LLLL
        val = (high << 8) + low

        return val

    def read_word_2c(self,adr,rf=1):
        # adr = register address. ex: set "0x32", then "adr" is equal to 50
        val = self.read_word(adr,rf)
        
        # 1 << 16: shift "1" to left by 16 bis
        # ex: 1 << 2 is equal to 0001 -> 0100
        if(val & (1 << 16 - 1)):
            return val - (1<<16)
        else:
            return val

class gy801(object):
    def __init__(self) :
        self.accel = ADXL345()
        self.gyro = L3G4200D()

class ADXL345(IMU):
    
    ADDRESS = ADXL345_ADDRESS

    def __init__(self) :
        #Class Properties
        self.Xoffset = 0  # unit: G, modify by yourself
        self.Yoffset = 0  # unit: G, modify by yourself
        self.Zoffset = 0    # unit: G, modify by yourself
        self.Xraw = 0.0
        self.Yraw = 0.0
        self.Zraw = 0.0
        self.Xg = 0.0
        self.Yg = 0.0
        self.Zg = 0.0
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.t0x = None
        self.t0y = None
        self.t0z = None

        self.df_value = 0b00001000 
        
        self.Xcalibr = ADXL345_SCALE_MULTIPLIER
        self.Ycalibr = ADXL345_SCALE_MULTIPLIER
        self.Zcalibr = ADXL345_SCALE_MULTIPLIER

        # Register 0x2C: BW_RATE
        self.write_byte(ADXL345_BW_RATE, ADXL345_BW_RATE_100HZ)    
        # write value= 0x0A = 00001111
        # D3-D0: The default value is 0x0A, 
        # which translates to a 100 Hz output data rate.


        # Register 0x2D: POWER_CTL 
        self.write_byte(ADXL345_POWER_CTL, ADXL345_MEASURE)    
        # write value: 0x08 = 00001000
        # D3=1: set 1 for measurement mode.


        # Register 0x31: DATA_FORMAT 
        self.write_byte(ADXL345_DATA_FORMAT, self.df_value)
        # write value=00001000
        # D3 = 1: the device is in full resolution mode, 
        # where the output resolution increases with the g range 
        # set by the range bits to maintain a 4 mg/LSB scale factor. 
        # D1 D0 = range. 00 = +-2g 

    
    # RAW readings in LPS
    # Register 0x32 to Register 0x37:
    # DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1 (Read Only)
    def getRawX(self) :
        self.Xraw = self.read_word_2c(ADXL345_DATAX0)
        return self.Xraw

    def getRawY(self) :
        self.Yraw = self.read_word_2c(ADXL345_DATAY0)
        return self.Yraw
    
    def getRawZ(self) :
        self.Zraw = self.read_word_2c(ADXL345_DATAZ0)
        return self.Zraw

    # G related readings in g
    # similar to filter. combine current value with previous one.
    # plf = 1 means it only uses "current reading"
    def getXg(self,plf = 1.0) :
        self.Xg = (self.getRawX() * self.Xcalibr - self.Xoffset) * plf + (1.0 - plf) * self.Xg
        return self.Xg

    def getYg(self,plf = 1.0) :
        self.Yg = (self.getRawY() * self.Ycalibr - self.Yoffset) * plf + (1.0 - plf) * self.Yg
        return self.Yg

    def getZg(self,plf = 1.0) :
        self.Zg = (self.getRawZ() * self.Zcalibr - self.Zoffset) * plf + (1.0 - plf) * self.Zg
        return self.Zg
    
    # Absolute reading in m/s2
    def getX(self,plf = 1.0) :
        self.X = self.getXg(plf) * EARTH_GRAVITY_MS2
        return self.X
    
    def getY(self,plf = 1.0) :
        self.Y = self.getYg(plf) * EARTH_GRAVITY_MS2
        return self.Y
    
    def getZ(self,plf = 1.0) :
        self.Z = self.getZg(plf) * EARTH_GRAVITY_MS2
        return self.Z


    # acc * LP * LP *100 = dist (cm)
    def getDistX(self,plf = 1.0) :
        if self.t0x is None : self.t0x = time.time()
        t1x = time.time()
        LP = t1x - self.t0x
        self.t0x = t1x
        self.Xdist = self.getX(plf) * LP * LP * 100
        return self.Xdist
        
    def getDistY(self,plf = 1.0) :
        if self.t0y is None : self.t0y = time.time()
        t1y = time.time()
        LP = t1y - self.t0y
        self.t0y = t1y
        self.Ydist = self.getY(plf) * LP * LP * 100
        return self.Ydist
        
    def getDistZ(self,plf = 1.0) :
        if self.t0z is None : self.t0z = time.time()
        t1z = time.time()
        LP = t1z - self.t0z
        self.t0z = t1z
        self.Zdist = self.getZ(plf) * LP * LP * 100
        return self.Zdist

class L3G4200D(IMU):
    
    ADDRESS = L3G4200D_ADDRESS

    def __init__(self) :
        #Class Properties
        self.Xraw = 0.0
        self.Yraw = 0.0
        self.Zraw = 0.0
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self.Xangle = 0.0
        self.Yangle = 0.0
        self.Zangle = 0.0
        self.t0x = None
        self.t0y = None
        self.t0z = None
        
        # FS=250dps, Sensitivity = 8.75 mdps/digit
        self.gain_std = 0.00875 

        # 0x20: CTRL_REG1, write 0x0F = 0000 1111
        self.write_byte(L3G4200D_CTRL_REG1, 0x0F)
        # DR1-DR0 = 00; BW1-BW0 = 00 -> 100Hz, Cut-off=12.5
        # PD, Zen, Yen, Xen = 1111

        # 0x23: CTRL_REG4, write 0x80 = 0000 1000
        self.write_byte(L3G4200D_CTRL_REG4, 0x80)
        # BDU=continous update; BLE=Data LSB
        # FS1-FS0=00 -> 250 dps

        self.setCalibration()

    def setCalibration(self) :
        gyr_r = self.read_byte(L3G4200D_CTRL_REG4)
        
        self.gain = 2 ** ( gyr_r & 48 >> 4) * self.gain_std

    # read raw data
    def getRawX(self):
        self.Xraw = self.read_word_2c(L3G4200D_OUT_X_L)
        return self.Xraw

    def getRawY(self):
        self.Yraw = self.read_word_2c(L3G4200D_OUT_Y_L)
        return self.Yraw

    def getRawZ(self):
        self.Zraw = self.read_word_2c(L3G4200D_OUT_Z_L)
        return self.Zraw

    # similar to filter. combine current value with previous one.
    # plf = 1 means it only uses "current reading"
    def getX(self,plf = 1.0):
        self.X = ( self.getRawX() * self.gain ) * plf + (1.0 - plf) * self.X
        return self.X

    def getY(self,plf = 1.0):
        self.Y = ( self.getRawY() * self.gain ) * plf + (1.0 - plf) * self.Y
        return self.Y

    def getZ(self,plf = 1.0):
        self.Z = ( self.getRawZ() * self.gain ) * plf + (1.0 - plf) * self.Z
        return self.Z

    # convert dps to angle. LP = loop period.
    # Degree per second * second = degree
    def getXangle(self,plf = 1.0) :
        if self.t0x is None : self.t0x = time.time()
        t1x = time.time()
        LP = t1x - self.t0x
        self.t0x = t1x
        self.Xangle = self.getX(plf) * LP
        return self.Xangle
    
    def getYangle(self,plf = 1.0) :
        if self.t0y is None : self.t0y = time.time()
        t1y = time.time()
        LP = t1y - self.t0y
        self.t0y = t1y
        self.Yangle = self.getY(plf) * LP
        return self.Yangle
    
    def getZangle(self,plf = 1.0) :
        if self.t0z is None : self.t0z = time.time()
        t1z = time.time()
        LP = t1z - self.t0z
        self.t0z = t1z
        self.Zangle = self.getZ(plf) * LP
        return self.Zangle

try:
    sensors = gy801()
    adxl345 = sensors.accel
    gyro = sensors.gyro    
    
    total_distX = 0.0
    total_distY = 0.0
    total_distZ = 0.0
    current_distX = 0.0
    current_distY = 0.0
    current_distZ = 0.0
    
    CF_X = 0
    CF_Y = 0
    
    while True:
        current_distX = adxl345.getDistX()
        current_distY = adxl345.getDistY()
        current_distZ = adxl345.getDistZ()
        
        # ====================================================
        # calculate pitch, roll, tilt
        aX = adxl345.X
        aY = adxl345.Y
        aZ = adxl345.Z
        
        # based on https://www.nxp.com/docs/en/application-note/AN3461.pdf
        roll = atan2(aY,aZ) * 180.0/pi
        pitch = -1 * atan2(-aX,sqrt(aY*aY+aZ*aZ)) * 180.0/pi
       
        print ("roll = %.3f, " % ( roll )),
#        print ("pitch = %.3f, " % ( pitch )),

        # ====================================================
        gyroX = gyro.getXangle()
        gyroY = gyro.getYangle()
        gyroZ = gyro.getZangle()
        
        print ("(Gyro) X = %.3f, " % ( gyroX )),
#        print ("Y = %.3f ." % ( gyroY ))

        # ====================================================
        CF_X = ( CF_X + gyroX) * 0.98 + roll * 0.02
        CF_Y = ( CF_Y + gyroY) * 0.98 + pitch * 0.02

        print ("CF X = %.3f, " % ( CF_X )),
        print ("CF X2 = %.3f ." % ( CF_X2 ))


#        time.sleep(1)
        
except KeyboardInterrupt:
    print("Cleanup")
