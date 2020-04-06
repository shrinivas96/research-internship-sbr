CMD = {'READ': 0x00, 'WRITE':0x80}

REG = {'WR':  0x00,
        'CR0': 0x01,
        'CR1': 0x02,
        'CR2': 0x03,
        'CR3': 0x09,
        'SR0': 0x04,
        'SR1': 0x05,
        'SR2': 0x06,
        'SR3': 0x07,
        'SR4': 0x0A }

a = [CMD['READ'] | REG['SR4'], 0]

b = [CMD['WRITE'] | REG['WR'],  0b01111000]

print(bin(CMD['WRITE']))
print(CMD['WRITE'])
print(bin(REG['WR']))
print(REG['WR'])
print(bin(CMD['WRITE'] | REG['WR']))
print(CMD['WRITE'] | REG['CR0'])


#import spidev
#spi = spidev.SpiDev()
#spi.open(0, 1)
#spi.max_speed_hz = 1000000
#spi.xfer([value_8bit])
#    
#spi.close() # … close the port before exit
##end try
