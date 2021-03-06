EESchema Schematic File Version 2
LIBS:shield-rescue
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:microchip_pic12mcu
LIBS:feather-m0-shield
LIBS:shield-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "IMUTYPE Shield"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R R3
U 1 1 588DFB5B
P 3100 1400
F 0 "R3" V 3180 1400 50  0000 C CNN
F 1 "2.2k" V 3100 1400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3030 1400 50  0001 C CNN
F 3 "" H 3100 1400 50  0000 C CNN
	1    3100 1400
	-1   0    0    1   
$EndComp
$Comp
L R R2
U 1 1 588E079C
P 2900 1700
F 0 "R2" V 2980 1700 50  0000 C CNN
F 1 "2.2k" V 2900 1700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 2830 1700 50  0001 C CNN
F 3 "" H 2900 1700 50  0000 C CNN
	1    2900 1700
	-1   0    0    1   
$EndComp
$Comp
L R R4
U 1 1 588E07BB
P 3100 1700
F 0 "R4" V 3180 1700 50  0000 C CNN
F 1 "2.2k" V 3100 1700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3030 1700 50  0001 C CNN
F 3 "" H 3100 1700 50  0000 C CNN
	1    3100 1700
	-1   0    0    1   
$EndComp
$Comp
L R R6
U 1 1 588E07DD
P 3300 1700
F 0 "R6" V 3380 1700 50  0000 C CNN
F 1 "2.2k" V 3300 1700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3230 1700 50  0001 C CNN
F 3 "" H 3300 1700 50  0000 C CNN
	1    3300 1700
	-1   0    0    1   
$EndComp
$Comp
L R R5
U 1 1 588E0822
P 3300 1400
F 0 "R5" V 3380 1400 50  0000 C CNN
F 1 "2.2k" V 3300 1400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3230 1400 50  0001 C CNN
F 3 "" H 3300 1400 50  0000 C CNN
	1    3300 1400
	-1   0    0    1   
$EndComp
$Comp
L R R1
U 1 1 588E0847
P 2900 1400
F 0 "R1" V 2980 1400 50  0000 C CNN
F 1 "2.2k" V 2900 1400 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 2830 1400 50  0001 C CNN
F 3 "" H 2900 1400 50  0000 C CNN
	1    2900 1400
	-1   0    0    1   
$EndComp
$Comp
L 24C64WP IC1
U 1 1 588E37A7
P 4250 2650
F 0 "IC1" H 4250 2400 39  0000 C CNN
F 1 "24C64WP" H 4250 2900 39  0000 C CNN
F 2 "footprints:24C64WP" H 4250 2650 60  0001 C CNN
F 3 "" H 4250 2650 60  0001 C CNN
	1    4250 2650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X08 P1
U 1 1 588E4561
P 5600 1600
F 0 "P1" H 5600 2050 50  0000 C CNN
F 1 "CONN_01X08" V 5700 1600 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Angled_1x08" H 5600 1600 50  0001 C CNN
F 3 "" H 5600 1600 50  0000 C CNN
	1    5600 1600
	1    0    0    -1  
$EndComp
NoConn ~ 2150 1550
NoConn ~ 2150 1650
NoConn ~ 2150 1750
NoConn ~ 2150 1950
NoConn ~ 2150 2150
NoConn ~ 2150 2250
NoConn ~ 2150 2350
NoConn ~ 1250 2650
NoConn ~ 1250 2550
NoConn ~ 1250 2450
NoConn ~ 1250 2350
NoConn ~ 1250 2250
NoConn ~ 1250 2150
NoConn ~ 1250 2050
NoConn ~ 1250 1750
NoConn ~ 1250 1650
NoConn ~ 1250 1350
NoConn ~ 1250 1150
NoConn ~ 3800 2500
NoConn ~ 3800 2600
NoConn ~ 3800 2700
NoConn ~ 4700 2600
NoConn ~ 2150 2450
$Comp
L FeatherM0Shield FM0S1
U 1 1 588EE21E
P 750 1450
F 0 "FM0S1" H 2000 1600 60  0000 C CNN
F 1 "FeatherM0Shield" H 1700 1900 60  0000 C CNN
F 2 "footprints:FeatherM0Shield" H 750 1450 60  0001 C CNN
F 3 "" H 750 1450 60  0001 C CNN
	1    750  1450
	1    0    0    -1  
$EndComp
NoConn ~ 1250 1550
$Comp
L +5VD #PWR?
U 1 1 59273A4C
P 750 800
F 0 "#PWR?" H 750 650 50  0001 C CNN
F 1 "+5VD" H 750 940 50  0000 C CNN
F 2 "" H 750 800 50  0001 C CNN
F 3 "" H 750 800 50  0001 C CNN
	1    750  800 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 59273A72
P 700 3150
F 0 "#PWR?" H 700 2900 50  0001 C CNN
F 1 "GND" H 700 3000 50  0000 C CNN
F 2 "" H 700 3150 50  0001 C CNN
F 3 "" H 700 3150 50  0001 C CNN
	1    700  3150
	1    0    0    -1  
$EndComp
Text Notes 5800 1750 0    47   ~ 0
SDA1
Text Notes 5800 1650 0    47   ~ 0
SCL1
Text Notes 5800 1950 0    47   ~ 0
SDA2
Text Notes 5800 1850 0    47   ~ 0
SCL2
Text Notes 5800 1450 0    47   ~ 0
VCC
Text Notes 5800 1550 0    47   ~ 0
GND
Text Notes 5800 1350 0    47   ~ 0
SDA0
Text Notes 5800 1250 0    47   ~ 0
SCL0
Wire Wire Line
	4950 1750 4950 2800
Wire Wire Line
	2300 1000 3300 1000
Wire Wire Line
	3300 1000 4200 1000
Connection ~ 4950 1750
Wire Wire Line
	4200 1000 4200 1650
Wire Wire Line
	4100 1100 4100 1750
Wire Wire Line
	4000 1200 4000 1850
Wire Wire Line
	4100 1750 4950 1750
Wire Wire Line
	4950 1750 5400 1750
Wire Wire Line
	4200 1650 4850 1650
Wire Wire Line
	4850 1650 5400 1650
Wire Wire Line
	4000 1850 5400 1850
Connection ~ 2900 1950
Wire Wire Line
	2900 1950 5400 1950
Connection ~ 3100 2050
Wire Wire Line
	4350 2050 3100 2050
Connection ~ 3300 2200
Wire Wire Line
	4450 2200 3300 2200
Wire Wire Line
	5300 1450 5400 1450
Wire Wire Line
	4350 1250 5400 1250
Wire Wire Line
	4450 1350 5400 1350
Wire Wire Line
	700  3150 900  3150
Wire Wire Line
	900  3150 3650 3150
Wire Wire Line
	3650 3150 5300 3150
Wire Wire Line
	5300 3150 5300 1450
Wire Wire Line
	4350 1250 4350 2050
Wire Wire Line
	4450 1350 4450 2200
Wire Wire Line
	4700 800  4700 2500
Connection ~ 3650 3150
Wire Wire Line
	3800 2800 3650 2800
Wire Wire Line
	4950 2800 4700 2800
Wire Wire Line
	4850 2700 4700 2700
Wire Wire Line
	1000 1850 1250 1850
Wire Wire Line
	1000 3050 1000 1850
Wire Wire Line
	3300 3050 1000 3050
Wire Wire Line
	3300 1850 3300 2200
Wire Wire Line
	3300 2200 3300 3050
Connection ~ 2900 1200
Connection ~ 3100 1100
Wire Wire Line
	1100 1950 1250 1950
Wire Wire Line
	1100 2950 1100 1950
Wire Wire Line
	3100 2950 1100 2950
Wire Wire Line
	3100 1850 3100 2050
Wire Wire Line
	3100 2050 3100 2950
Wire Wire Line
	2900 1850 2900 1950
Wire Wire Line
	2900 1950 2900 2650
Wire Wire Line
	2900 2650 2150 2650
Wire Wire Line
	2700 2550 2150 2550
Wire Wire Line
	2700 1200 2700 2550
Wire Wire Line
	2900 1250 2900 1200
Wire Wire Line
	2400 2050 2150 2050
Wire Wire Line
	2400 1100 2400 2050
Wire Wire Line
	3100 1250 3100 1100
Connection ~ 3300 1000
Wire Wire Line
	3300 1000 3300 1250
Wire Wire Line
	2300 1850 2300 1000
Wire Wire Line
	2150 1850 2300 1850
Connection ~ 3300 1550
Wire Wire Line
	900  3150 900  1450
Wire Wire Line
	900  1450 1250 1450
Wire Wire Line
	900  1250 1250 1250
Connection ~ 2900 1550
Wire Wire Line
	900  800  900  1250
Wire Wire Line
	2500 1550 2500 800 
Connection ~ 3100 1550
Wire Wire Line
	2700 1200 2900 1200
Wire Wire Line
	2900 1200 4000 1200
Wire Wire Line
	2400 1100 3100 1100
Wire Wire Line
	3100 1100 4100 1100
Wire Wire Line
	4850 1650 4850 2700
Connection ~ 4850 1650
Wire Wire Line
	5200 800  5200 1550
Connection ~ 2500 800 
Wire Wire Line
	2500 1550 2900 1550
Wire Wire Line
	2900 1550 3100 1550
Wire Wire Line
	3100 1550 3300 1550
Connection ~ 4700 800 
Wire Wire Line
	5200 1550 5400 1550
Wire Wire Line
	750  800  900  800 
Wire Wire Line
	900  800  2500 800 
Wire Wire Line
	2500 800  4700 800 
Wire Wire Line
	4700 800  5200 800 
Wire Wire Line
	3650 2800 3650 3150
Connection ~ 900  3150
Connection ~ 900  800 
$EndSCHEMATC
