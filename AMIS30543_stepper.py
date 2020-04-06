from time import sleep
import spidev
import RPi.GPIO as GPIO


# these are random values chsnge the pin numbers once they are decided.
SlaveSelect = 33
DIR = 31
STEP = 29

GPIO.setmode(GPIO.BOARD)
GPIO.setup(SlaveSelect, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setwarnings(False)

class AMIS30543:
    def __init__(self):
        self.wr = 0
        self.cr0 = 0 
        self.cr1 = 0 
        self.cr2 = 0 
        self.cr3 = 0
        self.spi = spidev.SpiDev()    # Enable SPI
#         bus, device = 0, 1    #Device is the chip select pin. Set to 0 or 1, depending on the connections
        self.spi.open(0, 1)    # Open a connection to a specific bus and device (chip select pin)
        self.spi.max_speed_hz = 500000    # Set SPI speed and mode
        self.spi.mode = 0
        self.spi.lsbfirst = False

        
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
#         Uncommment for poor man's debugging
#         print("Byte1[CMD,ADDR]:", byte1)
#         print("Byte2[DATA]:", byte2)
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
        receivedVal = self.spi.xfer2(byteVal)
#         Uncommment for poor man's debugging
#         print("Value received after transfer:\n", 
#               receivedVal, type(receivedVal), len(receivedVal))

    def setCurrentMilliamps(self, current):
        CUR_reg = 0
        # From Table 13 of the AMIS-30543 datasheet,
        # More values should be added for more current possibilities
        if current >= 2070:
            CUR_reg = 0b10100
        elif current >= 1850:
            CUR_reg = 0b10011
        elif current >= 1695:
            CUR_reg = 0b10010
        elif current >= 1520:
            CUR_reg = 0b10001   
#        print("Value of cr0 before setting CUR_reg:", bin(self.cr0))
        self.cr0 = (self.cr0 & 0b11100000) | CUR_reg;
#        print("Value of cr0 after setting CUR_reg:", bin(self.cr0))
        self.writeReg(self.REG['CR0'], self.cr0)
    
    def bit_not(self, n, numbits=8):
        return (1 << numbits) - 1 - n
    
    def setStepMode(self, mode):
        # By default mode is 32. 
        # See Table 12 fo AMIS-30543 datasheet
        esm = 0b000
        sm = 0b000
        if mode == 64:
            esm = 0b010
        elif mode == 128:
            esm = 0b001
        elif mode == 16:
            sm = 0b001
        else:
            print("Default mode set to 1/32 microsteps")
            
#        print("Value of cr0 before setting stepping mode:", bin(self.cr0))
        self.cr0 = (self.cr0 & self.bit_not(0b11100000, 8)) | (sm << 5)
#        print("Value of cr0 after setting CUR_reg:", bin(self.cr0))
        self.cr3 = (self.cr3 & self.bit_not(0b111, 3)) | esm
        self.writeReg(self.REG['CR0'], self.cr0)
        self.writeReg(self.REG['CR3'], self.cr3)
            
        
        

    
stepper1 = AMIS30543()


stepper1.initSS(SlaveSelect)
GPIO.output(STEP, GPIO.LOW)
GPIO.output(DIR, GPIO.LOW)
sleep(1);     #Give the driver some time to power up.


stepper1.reset_settings()

maxCurrent = 1800
stepper1.setCurrentMilliamps(1800);
stepper1.setStepMode(16)
stepper1.enableDriver()



total_steps = 200
u_steps = 16
rev_per_sec = 2
step_count = total_steps * u_steps
delay = (1 / (step_count*rev_per_sec)) / 2

    
GPIO.output(DIR, GPIO.HIGH)
for i in range(step_count*5):
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)

#GPIO.output(DIR, GPIO.LOW)
#for i in range(step_count):
#    GPIO.output(STEP, GPIO.HIGH)
#    sleep(delay)
#    GPIO.output(STEP, GPIO.LOW)
#    sleep(delay)

stepper1.close_port()

GPIO.cleanup()