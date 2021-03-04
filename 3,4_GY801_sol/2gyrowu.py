import smbus
import time
from math import *

bus = smbus.SMBus(1);            # 0 for R-Pi Rev. 1, 1 for Rev. 2

L3G4200D_ADDRESS        =    0x69
L3G4200D_CTRL_REG1      =    0x20 # CTRL_REG1 (20h)
L3G4200D_CTRL_REG4      =    0x23 # CTRL_REG4 (23h)
L3G4200D_OUT_X_L        =    0x28 # X-axis angular rate data. 
L3G4200D_OUT_X_H        =    0x29 # The value is expressed as two’s complement.
L3G4200D_OUT_Y_L        =    0x2A
L3G4200D_OUT_Y_H        =    0x2B
L3G4200D_OUT_Z_L        =    0x2C
L3G4200D_OUT_Z_H        =    0x2D

class IMU(object):

    def write_byte(self,adr, value):
        bus.write_byte_data(self.ADDRESS, adr, value)
    
    def read_byte(self,adr):
        return bus.read_byte_data(self.ADDRESS, adr)

    def read_word(self,adr,rf=1):
        # rf=1 Little Endian Format, rf=0 Big Endian Format
        if (rf == 1):
            # read two byte data.
            low = self.read_byte(adr)
            high = self.read_byte(adr+1)
        else:
            high = self.read_byte(adr)
            low = self.read_byte(adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self,adr,rf=1):
        val = self.read_word(adr,rf)
        if(val & (1 << 16 - 1)):
            return val - (1<<16)
        else:
            return val

class gy801(object):
    def __init__(self) :
        self.gyro = L3G4200D()


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
    total_gyroXangle = 0.0
    sensors = gy801()
    gyro = sensors.gyro    
    
    while True:    
        
        gyroX = gyro.getXangle()
        gyroY = gyro.getYangle()
        gyroZ = gyro.getZangle()
        
        total_gyroXangle+=gyroX

#        print ("Gyro: ")
        print ("(raw) X = %.3f dps, " % ( gyro.X )),
        print ("Y = %.3f dps, " % ( gyro.Y )),
        print ("Z = %.3f dps" % ( gyro.Z ))

        print ("(angle) X = %.3f deg, " % ( gyroX )),
        print ("Y = %.3f deg, " % ( gyroY )),
        print ("Z = %.3f deg" % ( gyroZ ))
#        print ("total gyro X = %.3f deg" % ( total_gyroXangle ))

        
except KeyboardInterrupt:
    print("Cleanup")
