/*
  Emon.h - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

#ifndef EmonLib_h
#define EmonLib_h

// define theoretical vref calibration constant for use in readvcc()
// 1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
// override in your code with value for your specific AVR chip
// determined by procedure described under "Calibrating the internal reference voltage" at
// http://openenergymonitor.org/emon/buildingblocks/calibration
#ifndef READVCC_CALIBRATION_CONST
#define READVCC_CALIBRATION_CONST 1126400L
#endif

#define ADC_BITS    10
#define ADC_COUNTS  (1<<ADC_BITS)

  void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL);
  void current(unsigned int _inPinI, double _ICAL);

  void voltageTX(double _VCAL, double _PHASECAL);
  void currentTX(unsigned int _channel, double _ICAL);

  void calcVI(unsigned int crossings, unsigned int timeout);
  double calcIrms(unsigned int NUMBER_OF_SAMPLES);

  long readVcc(void);

  //Useful value variables
  double realPower,
     apparentPower,
     powerFactor,
     Vrms,
     Irms;


  //Set Voltage and current input pins
  unsigned int inPinV;
  unsigned int inPinI;

  //Calibration coefficients
  //These need to be set in order to obtain accurate results
  double VCAL;
  double ICAL;
  double PHASECAL;

  //--------------------------------------------------------------------------------------
  // Variable declaration for emon_calc procedure
  //--------------------------------------------------------------------------------------
	int sampleV;  							 //sample_ holds the raw analog read value
	int sampleI;                     

	double lastFilteredV,filteredV;          //Filtered_ is the raw analog value minus the DC offset
	double filteredI;                  
	double offsetV;                          //Low-pass filter output
	double offsetI;                          //Low-pass filter output               

	double phaseShiftedV;                             //Holds the calibrated phase shifted voltage.

	double sqV,sumV,sqI,sumI,instP,sumP;              //sq = squared, sum = Sum, inst = instantaneous

	int startV;                                       //Instantaneous voltage at start of sample window.

	uint8_t lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.

#endif
