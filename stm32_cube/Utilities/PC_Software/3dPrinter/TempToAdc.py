#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Conversion of temperature value in Celsius to the ADC value corresponding to the thermistor voltage
# to be used with STEVAL-3DP001V1

"""Conversion of temperature value in Celsius to the ADC value corresponding to the thermistor voltage

Usage: python TempToAdc.py [options]

Options:
  -h, --help    Show this help
  --temp=...    Temperature in Celsius
  --r0=...      Thermistor reference resistance (in ohms)
  --t0=...      Thermistor reference temperature (Celsius)
  --beta=...    Thermistor beta value 
  
Example:
AdcToTemp.py --temp=210 --r0=100000 --t0=25 --beta=3950
"""


from math import exp
import sys
import getopt


#Convert Temperature in Celsius to ADC value 
def TempToAdcVal(temp, r0, t0, beta):

  vdd = 3.3   # voltage of the bridge divider
  rb   = 1000.0 # Resistance (in ohms) of the bridge top (R37, R38, R39 for extruder thermistors, R19, R20, R21 for bed thermistors)
  adcMaxValue = 1023.0  #the Adc is 10 bits
  calib = 0     # Calibration value for adjustement

  t0InKelvin = t0 + 273
  tempInKelvin = temp + 273 + calib
  rt = r0 * exp(beta * (1./tempInKelvin  - 1./t0InKelvin)) #thermistor resistance at this temperature
  ut = vdd * rt / (rb + rt) #Thermistor voltage (ADC input) at this temperature
  return round (adcMaxValue * ut / vdd )
  


def main(argv):

  beta = 3950.0 # beta value of the thermistor (eg on the Prusa I3 the NTC thermistor beta is 3950)
  r0 = 100000.0 # Thermistor reference resistance (in ohms)  (eg: 100000 for the 100k resitance of the Prusa I3 NTC thermistor)
  t0 = 25       # Temperature in Celsius corresponding to the thermistor reference resistance (eg: 25 for the the Prusa I3 NTC thermistor)
  temp = -1
  try:
    opts, args = getopt.getopt(argv, "h", ["help", "temp=", "r0=", "t0=", "beta="])
  except getopt.GetoptError:
    usage()
    sys.exit(2)
        
  for opt, arg in opts:
      if opt in ("-h", "--help"): 
          usage()
          sys.exit()
      elif opt == "--temp":
          temp = int(arg)          
      elif opt == "--r0":
          r0 = int(arg)
      elif opt == "--r0":
          r0 = int(arg)
      elif opt == "--t0":
          t0 = int(arg)
      elif opt == "--beta":
        beta = int(arg)
  print int(round(TempToAdcVal(temp, r0, t0, beta)))


def usage():
    print __doc__

if __name__ == "__main__":
    main(sys.argv[1:])
