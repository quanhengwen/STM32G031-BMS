#stm32g031k6-BMS#

stm32 microcontroller based battery management system

Battery management system controlled by STM32G031K6 32 bit microcontroller, the system has three separated units, power supply, analog digital conversion peripheral 
and communication unit. 

Power supply is connected directly to batteries meaning that we do not need additional external power source. Power supply main part is Texas Instruments integrated 
circuit TPS6107x which stands as a buck/boost converter generating stable 700mA and 3.3V at the output. All 4.7uF capacitors connceted to TPS6107x chip are used to give the 
maximum power at the output. Two 4.7uF capacitors instead of one 10uF capacitor are better solution due to less value noise generation. 

Analog to digital conversion peripheral is used to take analog values from various sources, like NTC thermistors and batteries. These sources give us battery voltage and
battery temperature directly from battery connector and from the battery side. These two temperature points are added to calculate the temperature change between battery
connector and battery body. ADC peripheral has an ability to generate 12 bit and 16 bit battery voltage value due to operational amplifier circuit that generates triangle shape
signal making it better for microcontroller to take measurements with 4 extra bits. LMV321 is used as a main operational amplifier.

Devices are communicating in series, sharing measured values between each other. Main CPU is collecting voltage and temperature values from every unit and posting it to 
the PLC. System testing unit is using both half duplex and full duplex communication method. Half duplex method allows the system to communicate with another one but not 
in the same time whereas full duplex is making that possible. Half and full duplex communication main elements are Texas Instruments SN65HVD10 and SN65HVD73 transreceivers 
circuits. ICs datasheets recommend 10kOhms pull up resistors for receiver input line and 10kOhms pull down resistor for transmitter line. All outputs must be protected 
from electrostatic discharge. ESD protection is made using zener diodes with Vz = 5V disallowing signals to have value greater than 5V.
