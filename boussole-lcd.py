#!/usr/bin/python
# Normand Labossiere VE2VAX /VA2NQ bousole antenne system
# avec  sonde de temperature
# Calibrate GY-26 I2C Compass
# Default address 0x70
#!/usr/bin/python

import smbus
import time
import serial
import struct
from time import sleep, strftime
from datetime import datetime
from subprocess import *
import sys
# make sure Adafruit_CharLCD is in the path
sys.path.append('/home/pi/Adafruit-Raspberry-Pi-Python-Code/Adafruit_CharLCD')
from Adafruit_CharLCD import Adafruit_CharLCD
# instantiate lcd and specify pins
lcd = Adafruit_CharLCD(pin_rs=26, pin_e=19, pins_db=[13, 6, 5, 11])
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

lcd.message('Antenne compass\n VE2VAX  V1.5')


def run_cmd(cmd):
    p = Popen(cmd, shell=True, stdout=PIPE)
    output = p.communicate()[0]
    return output
time.sleep(1)
ipaddr = run_cmd(cmd)
lcd.home()
lcd.message('IP: %s' % (ipaddr))
time.sleep(3)
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
  time.sleep (1)
ser.write(END_CAL)
lcd.clear()
lcd.home()
lcd.message('Fin Calibration\n')
time.sleep(1)
lcd.clear()
lcd.home()
ser.flush()
ser.close()
time.sleep(0.5)

ser.open()
ser.isOpen()
ser.write(R_BOUSSOLE)
time.sleep(0.02)
while ser.inWaiting() > 0:
    out += ser.read(1)
    time.sleep(0.02)
#
out = ''
while x !=0:
  ser.isOpen()
  ser.write(R_BOUSSOLE)
  time.sleep(0.02)
  out=''
  while ser.inWaiting() > 0:
      out += ser.read(1)
      time.sleep(0.02)
  #x = x -1
  #enleve les linefeed(\n) lf , ^L et les CR ^M
  # et convert out en integer
  #enleve  dernier caractere de la string et le point decimal
  out=out[:-1]
  out=out[:-1]
  out=out[:-1]
  out.lstrip('\n')
  out.rstrip('\n')
  out.lstrip('\r')
  out.rstrip('\r')
  out_int=int(out)
  out=str(out_int)
  lcd.home()
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
  outtemp = ''
  ser.write(TEMP)
  time.sleep(0.02)
  outtemp=''
  while ser.inWaiting() > 0:
     outtemp += ser.read(1)
     time.sleep(0.02)
  #enleve CR & LF
  outtemp.lstrip('\r')
  outtemp.lstrip('\n')
  outtemp.rstrip('\r')
  outtemp.rstrip('\n')
  #enleve dernier caractere de la string
  outtemp=outtemp[:-1]
  outtemp=outtemp[:-1]
  outtemp=outtemp[:-1]
  #convert outtemp en integer avec floating point
  outtemp_flint=int(float(outtemp))
  if outtemp_flint >= 400:
      tmp2=outtemp_flint-400
      tmp2=int(tmp2/4)
      tmp2=str(tmp2)
  else:
      tmp2=str(outtemp_flint)
  sequence = sequence +1
  # si outtemp >=400 , la temperature est negative (sous zero)
  if outtemp_flint >= 400:
     if sequence > 10:
         lcd.message('\n')
         lcd.message(datetime.now().strftime('   %H:%M:%S        \n'))
         lcd.home()
     else:
         lcd.message('\n')
         lcd.message('-' + tmp2 + (degree) + 'C Temp.Ext. ')
         lcd.home()
  else:
     if sequence > 10:
        lcd.message('\n')
        lcd.message(datetime.now().strftime('   %H:%M:%S        \n'))
        lcd.home()
     else:
        lcd.message('\n')
        lcd.message('+' + tmp2 + (degree) + 'C Temp.Ext. ')
        lcd.home()
  if sequence == 20:
     sequence = 0
x = x - 1
ser.close()
exit()

