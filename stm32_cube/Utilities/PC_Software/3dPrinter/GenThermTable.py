#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Generation of the thermistor table for the file thermistortables.h
# to be used with STEVAL-3DP001V1

"""Generation of the thermistor table for the file thermistortables.h

Usage: python GenThermTable.py [options]

Options:
  -h, --help      Show this help
  --nbent=...     Number of entries in the thermistor table
  --r0=...        Thermistor reference resistance (in ohms)
  --t0=...        Thermistor reference temperature (degree Celsius)
  --beta=...      Thermistor beta value 
  
Example:
GenThermTable.py --nbent=20 --r0=100000 --t0=25 --beta=3950
"""


from math import log
import sys
import getopt


adcMaxValue = 1023.0  #the Adc is 10 bits
vdd = 3.3   # voltage of the bridge divider
rb   = 1000.0 # Resistance (in ohms) of the bridge top (R37, R38, R39 for extruder thermistors, R19, R20, R21 for bed thermistors)
calib = 0     # Calibration value for adjustement
  
#Convert adc value to Temperature in degree Celsius
def adcValToTemp(adcVal, r0, t0, beta):
  ut = adcVal * vdd / adcMaxValue   #thermistor voltage (ADC input)
  t0InKelvin = t0 + 273
  return 1/((log(((ut*rb/vdd)/(1-(ut/vdd)))/r0)/beta)+(1./t0InKelvin)) - 273 - calib


def main(argv):

  beta = 3950 # beta value of the thermistor (eg on the Prusa I3 the NTC thermistor beta is 3950)
  r0 = 100000 # Thermistor reference resistance (in ohms)  (eg: 100000 for the 100k resitance of the Prusa I3 NTC thermistor)
  t0 = 25       # Temperature in degree Celsius corresponding to the thermistor reference resistance (eg: 25 for the the Prusa I3 NTC thermistor)
  nbent = 20
  try:
    opts, args = getopt.getopt(argv, "h", ["help", "nbent=", "r0=", "t0=", "beta="])
  except getopt.GetoptError:
    usage()
    sys.exit(2)
        
  for opt, arg in opts:
      if opt in ("-h", "--help"):
          usage()
          sys.exit()
      elif opt == "--nbent":
          nbent = int(arg)          
      elif opt == "--r0":
          r0 = int(arg)
      elif opt == "--r0":
          r0 = int(arg)
      elif opt == "--t0":
          t0 = int(arg)
      elif opt == "--beta":
        beta = int(arg)
  
  inc = int(adcMaxValue/(nbent - 1));
  adcVal = range(1, int(adcMaxValue), inc);
  

  print "// Thermistor lookup table for STEVAL-3DP001V1"
  print "// ./GenThermTable.py --nbent=%s --r0=%s --t0=%s --beta=%s " % (nbent, r0, t0, beta)
  print "// adcMaxValue: %s" % (adcMaxValue)
  print "// vdd: %s" % (vdd)
  print "// rb: %s" % (rb)
  print "// calib: %s" % (calib)    
  
  print "short temptable[][2] = {"

  for val in adcVal:
      if val == adcVal[-1]:
          #print "   {       %4s *OVERSAMPLENR,       %4s}" % (val, int(adcValToTemp(val, r0, t0, beta)))
          print "   {       %4s *OVERSAMPLENR,       %4s} //safety" % (val, int(0))
      else:
          print "   {       %4s *OVERSAMPLENR,       %4s}," % (val, int(round(adcValToTemp(val, r0, t0, beta))))
  print "};"  
  

def usage():
    print __doc__

if __name__ == "__main__":
    main(sys.argv[1:])
