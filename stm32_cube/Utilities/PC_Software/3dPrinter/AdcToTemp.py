#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Conversion of ADC values (thermistor voltage) to temperature in degree Celsius
# to be used with STEVAL-3DP001V1

"""Conversion of ADC values (thermistor voltage) to temperature in degree Celsius

Usage: python AdcToTemp.py [options]

Options:
  -h, --help    Show this help
  --adc=...     Adc value from 1 to 1022 
  --r0=...      Thermistor reference resistance (in ohms)
  --t0=...      Thermistor reference temperature (degree Celsius)
  --beta=...    Thermistor beta value 
  
Example:
AdcToTemp.py --adc=1008 --r0=100000 --t0=25 --beta=3950
"""


from math import log
import sys
import getopt


#Convert adc value to Temperature in degree Celsius
def adcValToTemp(adcVal, r0, t0, beta):
  vdd = 3.3   # voltage of the bridge divider
  rb   = 1000.0 # Resistance (in ohms) of the bridge top (R37, R38, R39 for extruder thermistors, R19, R20, R21 for bed thermistors)
  adcMaxValue = 1023.0  #the Adc is 10 bits
  calib = 0     # Calibration value for adjustement

  ut = adcVal * vdd / adcMaxValue   #thermistor voltage (ADC input)
  t0InKelvin = t0 + 273
  return 1/((log(((ut*rb/vdd)/(1-(ut/vdd)))/r0)/beta)+(1./t0InKelvin)) - 273 - calib

def main(argv):

  beta = 3950.0 # beta value of the thermistor (eg on the Prusa I3 the NTC thermistor beta is 3950)
  r0 = 100000.0 # Thermistor reference resistance (in ohms)  (eg: 100000 for the 100k resitance of the Prusa I3 NTC thermistor)
  t0 = 25       # Temperature in degree Celsius corresponding to the thermistor reference resistance (eg: 25 for the the Prusa I3 NTC thermistor)
  adcVal = -1
  adcValFound = 0
  try:
    opts, args = getopt.getopt(argv, "h", ["help", "adc=", "r0=", "t0=", "beta="])
  except getopt.GetoptError:
    usage()
    sys.exit(2)
        
  for opt, arg in opts:
      if opt in ("-h", "--help"):
          usage()
          sys.exit()
      elif opt == "--adc":
          adcVal = int(arg) 
          adcValFound = 1         
      elif opt == "--r0":
          r0 = int(arg)
      elif opt == "--r0":
          r0 = int(arg)
      elif opt == "--t0":
          t0 = int(arg)
      elif opt == "--beta":
        beta = int(arg)
    
  if (adcValFound == 0):
    print "adc parameter was not set. Please specify it!"
    usage()
    sys.exit()
  
  if (adcVal < 1) or (adcVal >1022):
      print adcVal, "is an incorrect Adc value (range is from 1 to 1022)\n"
      sys.exit()
    
  print int(round(adcValToTemp(adcVal, r0, t0, beta)))


def usage():
    print __doc__

if __name__ == "__main__":
    main(sys.argv[1:])
