# copyright 2016 psionprime as registered user of pololu.com support forums

import sys

if sys.version_info < (3, 5):
    raise "must use python 3.5 or greater"
else:
    print("- python interpreter >= 3.5")

import gpiozero
from spidev import SpiDev
import time
import random


class AMIS30543_Controller():

    def __init__(self, DO_STEP_PIN, DO_DIRECTION_PIN, DO_RESET_PIN, DI_FAULT_PIN):
        """
        AMIS-30543 Stepper Controller Class
        :param DO_STEP_PIN:
        :param DO_DIRECTION_PIN:
        :param DO_RESET_PIN:
        :param DI_FAULT_PIN:
        """

        # AMIS−30543 Registers & Command constants
        self.REG = {
            'WR': 0x00,
            'CR0': 0x01,
            'CR1': 0x02,
            'CR2': 0x03,
            'CR3': 0x09,
            'SR0': 0x04,
            'SR1': 0x05,
            'SR2': 0x06,
            'SR3': 0x07,
            'SR4': 0x0A
        }

        self.CMD = {
            'READ': 0x00,
            'WRITE': 0x80
        }

        # InitGPIO
        self.DO_STEP = gpiozero.LED(DO_STEP_PIN)
        self.DO_RESET = gpiozero.DigitalOutputDevice(DO_RESET_PIN)
        self.DO_DIRECTION = gpiozero.DigitalOutputDevice(DO_DIRECTION_PIN)
        self.DI_NO_FAULT = gpiozero.DigitalInputDevice(DI_FAULT_PIN)

        self.spi = SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1000000

        self.ResetStepper()

    def __del__(self):
        self.spi.close()

    def ResetStepper(self):
        self.DO_RESET.off()  # must be off for AMIS to see reset
        time.sleep(0.11)
        self.DO_RESET.on()
        time.sleep(0.11)
        self.DO_RESET.off()

    def StepIt(self):
        # future note maybe use PWM for hopefully much more steady hardware pulses
        if not self.DI_NO_FAULT:
            return
        self.DO_STEP.on()
        time.sleep(0.0001)
        self.DO_STEP.off()

    def RegisterDump(self):
        print("\nAMIS-30543 Registers\n")

        # check stepper status
        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['WR'], 0])
        print(" WR = ", bin(resp[1]), " ", str(resp[1]))

        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['CR0'], 0])
        print("CR0 = ", bin(resp[1]), " ", str(resp[1]))

        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['CR1'], 0])
        print("CR1 = ", bin(resp[1]), " ", str(resp[1]))

        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['CR2'], 0])
        print("CR2 = ", bin(resp[1]), " ", str(resp[1]))

        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['CR3'], 0])
        print("CR3 = ", bin(resp[1]), " ", str(resp[1]))

        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['SR0'], 0])
        print("SR0 = ", bin(resp[1]), " ", str(resp[1]))

        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['SR1'], 0])
        print("SR1 = ", bin(resp[1]), " ", str(resp[1]))

        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['SR2'], 0])
        print("SR2 = ", bin(resp[1]), " ", str(resp[1]))

        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['SR3'], 0])
        print("SR3 = ", bin(resp[1]), " ", str(resp[1]))

        resp = self.spi.xfer2([self.CMD['READ'] | self.REG['SR4'], 0])
        print("SR4 = ", bin(resp[1]), " ", str(resp[1]))

        print("")

    def ReverseBits(self, byte):
        byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4)
        byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2)
        byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1)
        return byte

    def test_RegisterRW(self):
        self.ResetStepper()

        self.spi.writebytes([self.CMD['WRITE'] | self.REG['WR'], 0b01111000])
        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR0'], 0b11110011])
        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR1'], 0b00100000])
        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR2'], 0b00001000])
        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR3'], 0b01000000])

        if self.spi.xfer2([self.CMD['READ'] | self.REG['WR'], 0])[1] != 0b01111000:
            print("Writing or reading self.REG['WR'] failed; driver power might be off.")
            return False

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR0'], 0])[1] != 0b11110011:
            print("Writing or reading self.REG['CR0'] failed; driver power might be off.")
            return False

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR1'], 0])[1] != 0b00100000:
            print("Writing or reading self.REG['CR1'] failed; driver power might be off.")
            return False

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR2'], 0])[1] != 0b00001000:
            print("Writing or reading self.REG['CR2'] failed; driver power might be off.")
            return False

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR3'], 0])[1] != 0b01000000:
            print("Writing or reading self.REG['CR3'] failed; driver power might be off.")
            return False

        self.RegisterDump()
        print("test_RegisterRW Passed\n")

        self.ResetStepper()
        return True

    def SetRegDefaults(self):
        """
        Sets default values:
         WR: no watchdog
        CR0: 1/32 step & current limit to 1850mA
        CR1: CW motion (depending on wiring) & step on rising edge
        CR2: motor off & no sleep & SLA no transparent & SLA gain @ 0.5
        CR3: no extended step mode
        :return:
        """

        self.ResetStepper()

        self.spi.writebytes([self.CMD['WRITE'] | self.REG['WR'], 0b00000000])
        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR0'], 0b00010011])
        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR1'], 0b00000000])
        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR2'], 0b00000000])
        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR3'], 0b00000000])

        if self.spi.xfer2([self.CMD['READ'] | self.REG['WR'], 0])[1] != 0b00000000:
            print("Writing or reading self.REG['WR'] failed; driver power might be off.")
            return False

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR0'], 0])[1] != 0b00010011:
            print("Writing or reading self.REG['CR0'] failed; driver power might be off.")
            return False

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR1'], 0])[1] != 0b00000000:
            print("Writing or reading self.REG['CR1'] failed; driver power might be off.")
            return False

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR2'], 0])[1] != 0b00000000:
            print("Writing or reading self.REG['CR2'] failed; driver power might be off.")
            return False

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR3'], 0])[1] != 0b00000000:
            print("Writing or reading self.REG['CR3'] failed; driver power might be off.")
            return False

        self.RegisterDump()
        print("Register Defaults Enabled\n")

        return True

    def SetMotorEnable(self):
        """
        CR2: motor on & no sleep & SLA no transparent & SLA gain @ 0.5
        :return:
        """

        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR2'], 0b10000000])

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR2'], 0])[1] != 0b10000000:
            print("Writing or reading self.REG['CR2'] failed; driver power might be off.")
            return False

    def SetMotorDisable(self):
        """
        CR2: motor on & no sleep & SLA no transparent & SLA gain @ 0.5
        :return:
        """

        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR2'], 0b00000000])

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR2'], 0])[1] != 0b00000000:
            print("Writing or reading self.REG['CR2'] failed; driver power might be off.")
            return False

    def SetDirPos(self):
        """
        CR1: CW motion (depending on wiring) & step on rising edge
        :return:
        """

        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR1'], 0b00000000])

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR1'], 0])[1] != 0b00000000:
            print("Writing or reading self.REG['CR1'] failed; driver power might be off.")
            return False

    def SetDirNeg(self):
        """
        CR1: CCW motion (depending on wiring) & step on rising edge
        :return:
        """

        self.spi.writebytes([self.CMD['WRITE'] | self.REG['CR1'], 0b10000000])

        if self.spi.xfer2([self.CMD['READ'] | self.REG['CR1'], 0])[1] != 0b10000000:
            print("Writing or reading self.REG['CR1'] failed; driver power might be off.")
            return False


axis_X1 = AMIS30543_Controller(19, 26, 20, 21)
axis_X1.RegisterDump()
axis_X1.test_RegisterRW()

if not axis_X1.SetRegDefaults():
    sys.exit("Failed to set register defaults")

axis_X1.SetMotorEnable()

# steps for 250mm = .3mm/step with GT2 2mm belt and 30 tooth pully and using 1/32 micro steps
steps = int(250 * ((200 * 32) / (2 * 30)))

print("\nStep Test for 250mm requires ", steps, " steps")

tStart = time.monotonic()

# long test to see if still running in morning
# add a little randomness to not land on the same spot the whole test and potentialy shorten the life of th motor
random.seed()
bToggle = True
while (True):

    for step in range(int(steps + random.getrandbits(8))):
        axis_X1.StepIt()
        if not axis_X1.DI_NO_FAULT:
            print("FAULT")
            axis_X1.RegisterDump()
            break

    if bToggle:
        axis_X1.SetDirNeg()
        bToggle = False
    else:
        axis_X1.SetDirPos()
        bToggle = True

# axis_X1.DO_STEP.blink(.000005, .000005, steps, False)
tEnd = time.monotonic()

print("\nStep Test took ", tEnd - tStart, " secs")
print("\nSpeed: ", 250 / (tEnd - tStart), " mm/sec\n")

axis_X1.SetMotorDisable()
