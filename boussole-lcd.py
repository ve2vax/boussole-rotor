#!/usr/bin/python
# Normand Labossiere VE2VAX /VA2NQ bousole antenne system
# avec  sonde de temperature
# Calibrate GY-26 I2C Compass
# Default address 0x70
#!/usr/bin/python
# Including  LCD Def.

from time import sleep


class CharLCD(object):

    # commands
    LCD_CLEARDISPLAY        = 0x01
    LCD_RETURNHOME          = 0x02
    LCD_ENTRYMODESET        = 0x04
    LCD_DISPLAYCONTROL      = 0x08
    LCD_CURSORSHIFT         = 0x10
    LCD_FUNCTIONSET         = 0x20
    LCD_SETCGRAMADDR        = 0x40
    LCD_SETDDRAMADDR        = 0x80

    # flags for display entry mode
    LCD_ENTRYRIGHT          = 0x00
    LCD_ENTRYLEFT           = 0x02
    LCD_ENTRYSHIFTINCREMENT = 0x01
    LCD_ENTRYSHIFTDECREMENT = 0x00

    # flags for display on/off control
    LCD_DISPLAYON           = 0x04
    LCD_DISPLAYOFF          = 0x00
    LCD_CURSORON            = 0x02
    LCD_CURSOROFF           = 0x00
    LCD_BLINKON             = 0x01
    LCD_BLINKOFF            = 0x00

    # flags for display/cursor shift
    LCD_DISPLAYMOVE         = 0x08
    LCD_CURSORMOVE          = 0x00

    # flags for display/cursor shift
    LCD_DISPLAYMOVE         = 0x08
    LCD_CURSORMOVE          = 0x00
    LCD_MOVERIGHT           = 0x04
    LCD_MOVELEFT            = 0x00

    # flags for function set
    LCD_8BITMODE            = 0x10
    LCD_4BITMODE            = 0x00
    LCD_2LINE               = 0x08
    LCD_1LINE               = 0x00
    LCD_5x10DOTS            = 0x04
    LCD_5x8DOTS             = 0x00

    def __init__(self, pin_rs=25, pin_e=24, pins_db=[23, 17, 21, 22], GPIO=None):
        # Emulate the old behavior of using RPi.GPIO if we haven't been given
        # an explicit GPIO interface to use
        if not GPIO:
            import RPi.GPIO as GPIO
            GPIO.setwarnings(False)
        self.GPIO = GPIO
        self.pin_rs = pin_rs
        self.pin_e = pin_e
        self.pins_db = pins_db

        self.GPIO.setmode(GPIO.BCM)
        self.GPIO.setup(self.pin_e, GPIO.OUT)
        self.GPIO.setup(self.pin_rs, GPIO.OUT)

        for pin in self.pins_db:
            self.GPIO.setup(pin, GPIO.OUT)

        self.write4bits(0x33)  # initialization
        self.write4bits(0x32)  # initialization
        self.write4bits(0x28)  # 2 line 5x7 matrix
        self.write4bits(0x0C)  # turn cursor off 0x0E to enable cursor
        self.write4bits(0x06)  # shift cursor right

        self.displaycontrol = self.LCD_DISPLAYON | self.LCD_CURSOROFF | self.LCD_BLINKOFF

        self.displayfunction = self.LCD_4BITMODE | self.LCD_1LINE | self.LCD_5x8DOTS
        self.displayfunction |= self.LCD_2LINE

        # Initialize to default text direction (for romance languages)
        self.displaymode = self.LCD_ENTRYLEFT | self.LCD_ENTRYSHIFTDECREMENT
        self.write4bits(self.LCD_ENTRYMODESET | self.displaymode)  # set the entry mode

        self.clear()

    def begin(self, cols, lines):
        if (lines > 1):
            self.numlines = lines
            self.displayfunction |= self.LCD_2LINE

    def home(self):
        self.write4bits(self.LCD_RETURNHOME)  # set cursor position to zero
        self.delayMicroseconds(3000)  # this command takes a long time!

    def clear(self):
        self.write4bits(self.LCD_CLEARDISPLAY)  # command to clear display
        self.delayMicroseconds(3000)  # 3000 microsecond sleep, clearing the display takes a long time

    def setCursor(self, col, row):
        self.row_offsets = [0x00, 0x40, 0x14, 0x54]
        if row > self.numlines:
            row = self.numlines - 1  # we count rows starting w/0
        self.write4bits(self.LCD_SETDDRAMADDR | (col + self.row_offsets[row]))

    def noDisplay(self):
        """ Turn the display off (quickly) """
        self.displaycontrol &= ~self.LCD_DISPLAYON
        self.write4bits(self.LCD_DISPLAYCONTROL | self.displaycontrol)

    def display(self):
        """ Turn the display on (quickly) """
        self.displaycontrol |= self.LCD_DISPLAYON
        self.write4bits(self.LCD_DISPLAYCONTROL | self.displaycontrol)

    def noCursor(self):
        """ Turns the underline cursor off """
        self.displaycontrol &= ~self.LCD_CURSORON
        self.write4bits(self.LCD_DISPLAYCONTROL | self.displaycontrol)

    def cursor(self):
        """ Turns the underline cursor on """
        self.displaycontrol |= self.LCD_CURSORON
        self.write4bits(self.LCD_DISPLAYCONTROL | self.displaycontrol)

    def noBlink(self):
        """ Turn the blinking cursor off """
        self.displaycontrol &= ~self.LCD_BLINKON
        self.write4bits(self.LCD_DISPLAYCONTROL | self.displaycontrol)

    def blink(self):
        """ Turn the blinking cursor on """
        self.displaycontrol |= self.LCD_BLINKON
        self.write4bits(self.LCD_DISPLAYCONTROL | self.displaycontrol)

    def DisplayLeft(self):
        """ These commands scroll the display without changing the RAM """
        self.write4bits(self.LCD_CURSORSHIFT | self.LCD_DISPLAYMOVE | self.LCD_MOVELEFT)

    def scrollDisplayRight(self):
        """ These commands scroll the display without changing the RAM """
        self.write4bits(self.LCD_CURSORSHIFT | self.LCD_DISPLAYMOVE | self.LCD_MOVERIGHT)

    def leftToRight(self):
        """ This is for text that flows Left to Right """
        self.displaymode |= self.LCD_ENTRYLEFT
        self.write4bits(self.LCD_ENTRYMODESET | self.displaymode)

    def rightToLeft(self):
        """ This is for text that flows Right to Left """
        self.displaymode &= ~self.LCD_ENTRYLEFT
        self.write4bits(self.LCD_ENTRYMODESET | self.displaymode)

    def autoscroll(self):
        """ This will 'right justify' text from the cursor """
        self.displaymode |= self.LCD_ENTRYSHIFTINCREMENT
        self.write4bits(self.LCD_ENTRYMODESET | self.displaymode)

    def noAutoscroll(self):
        """ This will 'left justify' text from the cursor """
        self.displaymode &= ~self.LCD_ENTRYSHIFTINCREMENT
        self.write4bits(self.LCD_ENTRYMODESET | self.displaymode)

    def write4bits(self, bits, char_mode=False):
        """ Send command to LCD """
        self.delayMicroseconds(1000)  # 1000 microsecond sleep
        bits = bin(bits)[2:].zfill(8)
        self.GPIO.output(self.pin_rs, char_mode)
        for pin in self.pins_db:
            self.GPIO.output(pin, False)
        for i in range(4):
            if bits[i] == "1":
                self.GPIO.output(self.pins_db[::-1][i], True)
        self.pulseEnable()
        for pin in self.pins_db:
            self.GPIO.output(pin, False)
        for i in range(4, 8):
            if bits[i] == "1":
                self.GPIO.output(self.pins_db[::-1][i-4], True)
        self.pulseEnable()

    def delayMicroseconds(self, microseconds):
        seconds = microseconds / float(1000000)  # divide microseconds by 1 million for seconds
        sleep(seconds)

    def pulseEnable(self):
        self.GPIO.output(self.pin_e, False)
        self.delayMicroseconds(1)       # 1 microsecond pause - enable pulse must be > 450ns
        self.GPIO.output(self.pin_e, True)
        self.delayMicroseconds(1)       # 1 microsecond pause - enable pulse must be > 450ns
        self.GPIO.output(self.pin_e, False)
        self.delayMicroseconds(1)       # commands need > 37us to settle

    def message(self, text):
        """ Send string to LCD. Newline wraps to second line"""
        for char in text:
            if char == '\n':
                self.write4bits(0xC0)  # next line
            else:
                self.write4bits(ord(char), True)

#
#
import smbus
import serial
import struct
from time import sleep, strftime
from datetime import datetime
from subprocess import *
import sys
# make sure CharLCD is in the path
#sys.path.append('/home/pi/Raspberry-Pi-Python-Code/CharLCD')
#from CharLCD import CharLCD
# instantiate lcd and specify pins
#
#
lcd = CharLCD(pin_rs=26, pin_e=19, pins_db=[13, 6, 5, 11])
lcd.clear()
# specify columns and rows
lcd.begin(16,2)
degree = chr(223)
dir_left = chr(127)
dir_right = chr(126)
dir_mem = chr(94)
t=3
x=100000
sequence=1
cmd = "ip addr show wlan0 | grep 'inet ' | awk '{print $2}' | cut -d/ -f1"

out = ''

################
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
START_CAL=bytearray([0xC0])
END_CAL=bytearray([0xC1])
RESET_FACT=bytearray([0xA0,0xAA,0xA5,0xC5])
TEMP=bytearray([0x35])
R_BOUSSOLE=bytearray([0x31])
START_CAL=bytearray([0xC0])
INCLINAISON=bytearray([0x8C])  # 8C = 14.0degree de correction
SET_INCLINAISON=bytearray([0x03,0x00,0x04,0x8C])
END_CAL=bytearray([0xC1])

lcd.message('Antenne compass\n VE2VAX  V1.4')


def run_cmd(cmd):
    p = Popen(cmd, shell=True, stdout=PIPE)
    output = p.communicate()[0]
    return output
sleep(1)
ipaddr = run_cmd(cmd)
lcd.home()
lcd.message('IP: %s' % (ipaddr))
sleep(3)
lcd.clear()
lcd.home()

#ser.write(START_CAL)
ser.write(SET_INCLINAISON)
while t !=0:
  lcd.home()
  lcd.message('+14 incl. %s' % (t))
  lcd.message(' Sec.\n')
  lcd.message(datetime.now().strftime(' %H:%M:%S \n'))
  t = t - 1
  sleep (1)

ser.write(END_CAL)
lcd.clear()
lcd.home()
lcd.message('Fin Calibration\n')
sleep(1)

lcd.clear()
lcd.home()

ser.close()
sleep(0.2)

ser.open()
while x !=0:
  ser.isOpen()
  out = ''
  ser.write(R_BOUSSOLE)
  sleep(0.05)
  while ser.inWaiting() > 0:
    out += ser.read(1)
    sleep(0.04)

    #if out != '':
       #print ">>" + out

  #x = x -1
  # remove first caracters
  out = out[1:]
  out = out[1:]
  ##remove extra  caracters
  #print(out)
  out = out[:-2]
  out = out[:-1]
  #print(out)
  #out1 = out[:-2]
  out_int=int(out)
  lcd.message(out)
  lcd.message(degree)
  if out_int  <  30:
    dir=('N ')
    dir_mem = dir_right
  if out_int >= 30 <= 60:
    dir=('NE')
  if out_int > 60 < 120:
    dir=('E ')
  if out_int >= 120 <= 150:
    dir=('SE')
  if out_int > 150 < 210:
    dir=('S ')
  if out_int >= 210 <= 240:
    dir=('SO')
  if out_int > 240 < 300:
    dir=('O ')
  if out_int >= 300 <= 330:
    dir=('NO')
  if out_int > 330:
    dir=('N')
    dir_mem = dir_left
  lcd.message(dir_mem)
  lcd.message(dir)
  lcd.message(' Bearing ')
  lcd.message('\n')
  ser.close()
  ser.open()
  outtemp = ''
  ser.write(TEMP)
  sleep(0.04)
  while ser.inWaiting() > 0:
    outtemp += ser.read(1)
    sleep(0.04)
    # remove first scaracters
  outtemp = outtemp[1:]
  outtemp = outtemp[1:]
  #print(outtemp)
  ## remove  caracters keep only the first
  outtemp1 = outtemp[:1]
  #outtemp1 = outtemp[-4:]
  #print(outtemp)
  #print(outtemp1)
  ##
  outtemp2=outtemp[1:-1]
  #print(outtemp2)
  #print(outtemp1)
  quatre='4'
  sequence = sequence +1
  if outtemp1 == quatre and sequence > 10:
     lcd.message('\n')
     lcd.message(datetime.now().strftime('   %H:%M:%S        \n'))
     lcd.home()
  else:
     lcd.message('\n')
     lcd.message('-' + outtemp2 + (degree) + 'C Temp.Ext. ')
     lcd.home()
  if  outtemp1 != quatre and sequence > 10:
     lcd.message('\n')
     lcd.message(datetime.now().strftime('   %H:%M:%S        \n'))
     lcd.home()
  else:
     lcd.message('\n')
     lcd.message('+' + outtemp2 + (degree) + 'C Temp.Ext. ')
     lcd.home()
  if sequence == 20:
     sequence = 0
x = x - 1
ser.close()
exit()
