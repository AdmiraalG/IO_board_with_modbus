This is a seed for a general I/O board with some expanders and ADC's. No rocket science, just some messing around to make some choices.
Currently I'm hanging with the Modbus driver here. I need to reform the modbus driver's delay routines into my millisecond interrupt. So the modbus driver needs to get interrupt feedback for continuing processing while the rest of the interrups keep on flowing. Even between bus characters there still is loop processing. I did it once in MPLAB CCSC putting the timout routine in interrupt and cleaned the count on each RDA interrupt. The arduino environment is way more enclosed to do this. I will never poll in loop(). Any suggestions anyone?
