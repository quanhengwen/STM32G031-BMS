# stm32g031k6-BMS
->stm32 microcontroller based battery management system<-

Battery management system controlled by STM32G031K632 bit microcontroller, the system has three separated units, power supply, analog digital conversion peripheral 
and communication unit. 

Power supply is connected directly to batteries meaning that we do not need additional external power source. Power supply main part is Texas Instruments integrated 
circuit TPS6107x which stands as a buck/boost converter generating stable 700mA and 3.3V at the output. All 4.7uF capacitors connceted to TPS6107x chip are used to give the 
maximum power at the output. Two 4.7uF capacitors instead of one 10uF capacitor are better solution due to less value noise generation. 

Analog to digital conversion peripheral is used to take analog values from various sources, like NTC thermistors and batteries. These sources give us battery voltage and
battery temperature directly from battery connector and from the battery side. These two temperature points are added to calculate the temperature change between battery
connector and battery body. ADC peripheral has an ability to generate 12 bit and 16 bit battery voltage value due to operational amplifier circuit that generates triangle shape
signal making it better for microcontroller to take measurements with 4 extra bits. LMV321 is used as a main operational amplifier.
