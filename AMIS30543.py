"""
AMIS 30543 has an SPI interface. This python file is inspired from the 
documentation of POLOLU AMIS 30543:
https://pololu.github.io/amis-30543-arduino/
and
https://pololu.github.io/amis-30543-arduino/class_a_m_i_s30543.html

For information on the control and status registers, visit
https://www.pololu.com/file/0J869/AMIS-30543-D.pdf [AMIS30543-datasheet]

Transferring a packet follows:
|       BYTE1       |   BYTE2   |
|CMD[2:0] ADDR[4:0] |   D[7:0]  |

CMD operation is performed on register address specified in ADDR
and BYTE2 data is read/written.
"""

import spidev
import RPi.GPIO as GPIO


class AMIS30543:
    def __init__(self, bus, device):
        self.wr = 0
        self.cr0 = 0 
        self.cr1 = 0 
        self.cr2 = 0 
        self.cr3 = 0
        self.bus = bus
        self.device = device
        self.spi = spidev.SpiDev()								# Enable SPI
        # bus, device = 0, 1    							    # Device is the chip select pin. Set to 0 or 1, depending on the connections
        self.spi.open(self.bus, self.bus)                       # Open a connection to a specific bus and device (chip select pin)
        self.spi.max_speed_hz = 500000    					    # Setting SPI speed, mode 0 and MSBfirst
        self.spi.mode = 0
        self.spi.lsbfirst = False                               # Serial data transfer is assumed to follow MSB first rule.

        # Decalre the addresses of control and status registers. Table 11 AMIS30543-datasheet
        self.REG = {'WR':  0x00,
                    'CR0': 0x01,
                    'CR1': 0x02,
                    'CR2': 0x03,
                    'CR3': 0x09,
                    'SR0': 0x04,
                    'SR1': 0x05,
                    'SR2': 0x06,
                    'SR3': 0x07,
                    'SR4': 0x0A}
        
        self.CMD = {'READ': 0x00, 
                    'WRITE':0x80}
    
    def reset_settings(self):
        self.wr = 0
        self.cr0 = 0 
        self.cr1 = 0 
        self.cr2 = 0 
        self.cr3 = 0
        self.applysettings()
    
    def initSS(self, ssPin):
        self.ssPin = ssPin
        GPIO.output(self.ssPin, GPIO.HIGH)
        
    def enableDriver(self):
        self.cr2 = self.cr2 | 0b10000000
        self.applysettings()
    
    def applysettings(self):
        self.writeReg(self.REG['CR2'], self.cr2)
        self.writeReg(self.REG['WR'], self.wr)
        self.writeReg(self.REG['CR0'], self.cr0)
        self.writeReg(self.REG['CR1'], self.cr1)
        self.writeReg(self.REG['CR3'], self.cr3)
    
    def writeReg(self, address, value):
        self.selectChip()
        byte1 = self.CMD['WRITE'] | (address & 0b11111)
        byte2 = value
        self.transfer([byte1, byte2])
        self.deselctChip()
        
    def selectChip(self):
        GPIO.output(self.ssPin, GPIO.LOW)

    def deselctChip(self):
        GPIO.output(self.ssPin, GPIO.HIGH)
        # NEED TO ADD A MICROSECOND DELAY BECAUSE 
        # CS high time is specified as 2.5 us in the AMIS-30543 datasheet.
    
    def close_port(self):
        self.spi.close()
    
    def transfer(self, byteVal):
        receivedVal = self.spi.xfer2(byteVal)  # data from slave (if any) in response to transferring byteVal saved in receivedVal

    def setCurrentMilliamps(self, current):
        CUR_reg = 0
        # From Table 13 of the AMIS-30543 datasheet,
        # More values should be added for more current settings
        if current >= 2070:
            CUR_reg = 0b10100
        elif current >= 1850:
            CUR_reg = 0b10011
        elif current >= 1695:
            CUR_reg = 0b10010
        elif current >= 1520:
            CUR_reg = 0b10001   
        self.cr0 = (self.cr0 & 0b11100000) | CUR_reg
        self.writeReg(self.REG['CR0'], self.cr0)
    
    def bit_not(self, n, numbits=8):
        return (1 << numbits) - 1 - n
    
    def setStepMode(self, mode):
        # By default, the mode is 32. 
        # See Table 12 of the AMIS-30543 datasheet
        esm = 0b000
        sm = 0b000
        if mode == 64:
            esm = 0b010
        elif mode == 128:
            esm = 0b001
        elif mode == 16:
            sm = 0b001
        else:
            pass
            # print("Default mode set to 1/32 microsteps")
            
        self.cr0 = (self.cr0 & self.bit_not(0b11100000, 8)) | (sm << 5)
        self.cr3 = (self.cr3 & self.bit_not(0b111, 3)) | esm
        self.writeReg(self.REG['CR0'], self.cr0)
        self.writeReg(self.REG['CR3'], self.cr3)

if __name__ == "__main__":
    print("This file only contains the main class for interfacing and using the AMIS-30543 stepper driver.")
