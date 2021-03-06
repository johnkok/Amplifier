EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Conn_01x01_Male J10
U 1 1 60BF7AF5
P 1250 3050
F 0 "J10" H 1358 3231 50  0000 C CNN
F 1 "PRI_TRF_L" H 1358 3140 50  0000 C CNN
F 2 "ADCH-80A+:TERMINAL_250" H 1250 3050 50  0001 C CNN
F 3 "~" H 1250 3050 50  0001 C CNN
	1    1250 3050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J11
U 1 1 60BF82F7
P 1250 3550
F 0 "J11" H 1358 3731 50  0000 C CNN
F 1 "PRI_TRF_N" H 1358 3640 50  0000 C CNN
F 2 "ADCH-80A+:TERMINAL_250" H 1250 3550 50  0001 C CNN
F 3 "~" H 1250 3550 50  0001 C CNN
	1    1250 3550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J6
U 1 1 60BF8E47
P 1250 1050
F 0 "J6" H 1358 1231 50  0000 C CNN
F 1 "MAIN_PWR_L" H 1358 1140 50  0000 C CNN
F 2 "ADCH-80A+:TERMINAL_250" H 1250 1050 50  0001 C CNN
F 3 "~" H 1250 1050 50  0001 C CNN
	1    1250 1050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J7
U 1 1 60BF8E4D
P 1250 1550
F 0 "J7" H 1358 1731 50  0000 C CNN
F 1 "MAIN_PWR_N" H 1358 1640 50  0000 C CNN
F 2 "ADCH-80A+:TERMINAL_250" H 1250 1550 50  0001 C CNN
F 3 "~" H 1250 1550 50  0001 C CNN
	1    1250 1550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J8
U 1 1 60BF9A77
P 1250 2050
F 0 "J8" H 1358 2231 50  0000 C CNN
F 1 "SEC_TRF_L" H 1358 2140 50  0000 C CNN
F 2 "ADCH-80A+:TERMINAL_250" H 1250 2050 50  0001 C CNN
F 3 "~" H 1250 2050 50  0001 C CNN
	1    1250 2050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J9
U 1 1 60BF9A7D
P 1250 2550
F 0 "J9" H 1358 2731 50  0000 C CNN
F 1 "SEC_TRF_N" H 1358 2640 50  0000 C CNN
F 2 "ADCH-80A+:TERMINAL_250" H 1250 2550 50  0001 C CNN
F 3 "~" H 1250 2550 50  0001 C CNN
	1    1250 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 1550 1750 1550
Wire Wire Line
	1750 2550 1450 2550
Wire Wire Line
	1450 3550 1750 3550
$Comp
L Device:Fuse F1
U 1 1 60BFA3D5
P 2000 1250
F 0 "F1" H 2060 1296 50  0000 L CNN
F 1 "1A" H 2060 1205 50  0000 L CNN
F 2 "Fuse:Fuseholder_Cylinder-5x20mm_Schurter_0031_8201_Horizontal_Open" V 1930 1250 50  0001 C CNN
F 3 "~" H 2000 1250 50  0001 C CNN
	1    2000 1250
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse F2
U 1 1 60BFB957
P 2850 1250
F 0 "F2" H 2910 1296 50  0000 L CNN
F 1 "10A" H 2910 1205 50  0000 L CNN
F 2 "Fuse:Fuseholder_Cylinder-5x20mm_Schurter_0031_8201_Horizontal_Open" V 2780 1250 50  0001 C CNN
F 3 "~" H 2850 1250 50  0001 C CNN
	1    2850 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 1050 2000 1050
Wire Wire Line
	2000 1100 2000 1050
Wire Wire Line
	2000 2050 1450 2050
Wire Wire Line
	2850 1400 2850 1500
Wire Wire Line
	1750 1550 1750 2550
Connection ~ 1750 2550
Wire Wire Line
	1750 2550 1750 3550
Wire Wire Line
	2000 1400 2000 2050
Wire Wire Line
	2000 1050 2850 1050
Wire Wire Line
	2850 1050 2850 1100
Connection ~ 2000 1050
$Comp
L Device:R R25
U 1 1 60C05B9A
P 2400 1700
F 0 "R25" H 2500 1650 50  0000 L CNN
F 1 "100R/1W" H 2450 1550 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0414_L11.9mm_D4.5mm_P15.24mm_Horizontal" V 2330 1700 50  0001 C CNN
F 3 "~" H 2400 1700 50  0001 C CNN
	1    2400 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 60C0673E
P 2400 2100
F 0 "C6" H 2515 2146 50  0000 L CNN
F 1 "0.01uF/630V" H 2515 2055 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D8.0mm_W5.0mm_P10.00mm" H 2438 1950 50  0001 C CNN
F 3 "~" H 2400 2100 50  0001 C CNN
	1    2400 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 1850 2400 1950
Wire Wire Line
	2400 2250 2400 2300
Wire Wire Line
	2850 3050 2850 2300
Wire Wire Line
	1450 3050 2850 3050
Wire Wire Line
	2400 2300 2850 2300
Wire Wire Line
	2400 1500 2400 1550
Wire Wire Line
	2400 1500 2850 1500
$Comp
L Connector:Conn_01x01_Male J12
U 1 1 60C38E9F
P 1300 4000
F 0 "J12" H 1408 4181 50  0000 C CNN
F 1 "SEC_OUT_L" H 1408 4090 50  0000 C CNN
F 2 "ADCH-80A+:TERMINAL_250" H 1300 4000 50  0001 C CNN
F 3 "~" H 1300 4000 50  0001 C CNN
	1    1300 4000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x01_Male J13
U 1 1 60C38EA5
P 1300 4600
F 0 "J13" H 1408 4781 50  0000 C CNN
F 1 "SEC_OUT_N" H 1408 4690 50  0000 C CNN
F 2 "ADCH-80A+:TERMINAL_250" H 1300 4600 50  0001 C CNN
F 3 "~" H 1300 4600 50  0001 C CNN
	1    1300 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0134
U 1 1 60C3B101
P 3450 4850
F 0 "#PWR0134" H 3450 4600 50  0001 C CNN
F 1 "GND" H 3455 4677 50  0000 C CNN
F 2 "" H 3450 4850 50  0001 C CNN
F 3 "" H 3450 4850 50  0001 C CNN
	1    3450 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 4800 3450 4850
$Comp
L Device:C C7
U 1 1 60C3CBA7
P 3450 4600
F 0 "C7" H 3565 4646 50  0000 L CNN
F 1 "0.1" H 3565 4555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 3488 4450 50  0001 C CNN
F 3 "~" H 3450 4600 50  0001 C CNN
	1    3450 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C8
U 1 1 60C3D5B0
P 3900 4600
F 0 "C8" H 4018 4646 50  0000 L CNN
F 1 "220" H 4018 4555 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x6.2" H 3938 4450 50  0001 C CNN
F 3 "~" H 3900 4600 50  0001 C CNN
	1    3900 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 4300 3450 4300
Wire Wire Line
	3900 4300 3900 4450
Wire Wire Line
	3900 4800 3900 4750
Wire Wire Line
	3450 4750 3450 4800
Connection ~ 3450 4800
Wire Wire Line
	3450 4800 3900 4800
Wire Wire Line
	3450 4450 3450 4300
Connection ~ 3450 4300
Wire Wire Line
	3450 4300 3900 4300
Connection ~ 3900 4300
$Comp
L power:+3.3V #PWR0136
U 1 1 60C44F45
P 9500 4750
F 0 "#PWR0136" H 9500 4600 50  0001 C CNN
F 1 "+3.3V" H 9515 4923 50  0000 C CNN
F 2 "" H 9500 4750 50  0001 C CNN
F 3 "" H 9500 4750 50  0001 C CNN
	1    9500 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0137
U 1 1 60C458A6
P 9500 3250
F 0 "#PWR0137" H 9500 3100 50  0001 C CNN
F 1 "+5V" H 9515 3423 50  0000 C CNN
F 2 "" H 9500 3250 50  0001 C CNN
F 3 "" H 9500 3250 50  0001 C CNN
	1    9500 3250
	1    0    0    -1  
$EndComp
Text HLabel 4050 2350 3    50   Input ~ 0
MAIN_TRF_CTRL
$Comp
L Device:D_Bridge_+-AA D2
U 1 1 60C589C9
P 2600 4300
F 0 "D2" H 2944 4346 50  0000 L CNN
F 1 "D_Bridge_+-AA" H 2944 4255 50  0000 L CNN
F 2 "Diode_SMD:Diode_Bridge_Vishay_DFS" H 2600 4300 50  0001 C CNN
F 3 "~" H 2600 4300 50  0001 C CNN
	1    2600 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 60C5F29F
P 2000 4300
F 0 "C5" H 2115 4346 50  0000 L CNN
F 1 "0.1" H 2115 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 2038 4150 50  0001 C CNN
F 3 "~" H 2000 4300 50  0001 C CNN
	1    2000 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 4600 2000 4600
Wire Wire Line
	2000 4450 2000 4600
Connection ~ 2000 4600
Wire Wire Line
	2000 4150 2000 4000
Connection ~ 2000 4000
Wire Wire Line
	2000 4000 2600 4000
Wire Wire Line
	2300 4300 2250 4300
Wire Wire Line
	2250 4300 2250 4700
$Comp
L power:GND #PWR0138
U 1 1 60C65B9A
P 2250 4700
F 0 "#PWR0138" H 2250 4450 50  0001 C CNN
F 1 "GND" H 2255 4527 50  0000 C CNN
F 2 "" H 2250 4700 50  0001 C CNN
F 3 "" H 2250 4700 50  0001 C CNN
	1    2250 4700
	1    0    0    -1  
$EndComp
$Comp
L Isolator:SFH617A-1X001 U5
U 1 1 60C6F01A
P 2200 5400
F 0 "U5" H 2200 5725 50  0000 C CNN
F 1 "SFH6186-4T" H 2200 5634 50  0000 C CNN
F 2 "Package_SO:SO-4_7.6x3.6mm_P2.54mm" H 2000 5200 50  0001 L CIN
F 3 "http://www.vishay.com/docs/83740/sfh617a.pdf" H 2200 5400 50  0001 L CNN
	1    2200 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 5300 1800 5300
$Comp
L power:GND #PWR0139
U 1 1 60C8C901
P 2600 5550
F 0 "#PWR0139" H 2600 5300 50  0001 C CNN
F 1 "GND" H 2605 5377 50  0000 C CNN
F 2 "" H 2600 5550 50  0001 C CNN
F 3 "" H 2600 5550 50  0001 C CNN
	1    2600 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 5500 2600 5500
Wire Wire Line
	2600 5500 2600 5550
Wire Wire Line
	1500 4000 1800 4000
$Comp
L Device:R R24
U 1 1 60C91F40
P 1600 4950
F 0 "R24" V 1500 4900 50  0000 L CNN
F 1 "2K2" V 1600 4900 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 1530 4950 50  0001 C CNN
F 3 "~" H 1600 4950 50  0001 C CNN
	1    1600 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 5100 1800 5300
Wire Wire Line
	1800 4800 1800 4000
Connection ~ 1800 4000
Wire Wire Line
	1800 4000 2000 4000
Text HLabel 2750 5300 2    50   Input ~ 0
MAIN_POL
Wire Wire Line
	2500 5300 2750 5300
Wire Wire Line
	1500 4600 1600 4600
$Comp
L Device:D D1
U 1 1 60CA66B5
P 1800 4950
F 0 "D1" V 1846 4870 50  0000 R CNN
F 1 "1N4148" V 1755 4870 50  0000 R CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 1800 4950 50  0001 C CNN
F 3 "~" H 1800 4950 50  0001 C CNN
	1    1800 4950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1600 5100 1600 5500
Wire Wire Line
	1600 5500 1900 5500
Wire Wire Line
	1600 4800 1600 4600
Connection ~ 1600 4600
Wire Wire Line
	1600 4600 2000 4600
$Comp
L power:+3.3VA #PWR0187
U 1 1 60D992A1
P 10650 4750
F 0 "#PWR0187" H 10650 4600 50  0001 C CNN
F 1 "+3.3VA" H 10665 4923 50  0000 C CNN
F 2 "" H 10650 4750 50  0001 C CNN
F 3 "" H 10650 4750 50  0001 C CNN
	1    10650 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+5VA #PWR0190
U 1 1 6125C307
P 10650 3250
F 0 "#PWR0190" H 10650 3100 50  0001 C CNN
F 1 "+5VA" H 10665 3423 50  0000 C CNN
F 2 "" H 10650 3250 50  0001 C CNN
F 3 "" H 10650 3250 50  0001 C CNN
	1    10650 3250
	1    0    0    -1  
$EndComp
Text HLabel 10200 1900 2    50   Input ~ 0
WR_BACK
Text HLabel 8950 1900 2    50   Input ~ 0
nRST
$Comp
L power:+3.3V #PWR0244
U 1 1 6316B233
P 8550 1400
F 0 "#PWR0244" H 8550 1250 50  0001 C CNN
F 1 "+3.3V" H 8565 1573 50  0000 C CNN
F 2 "" H 8550 1400 50  0001 C CNN
F 3 "" H 8550 1400 50  0001 C CNN
	1    8550 1400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0245
U 1 1 6316E1BA
P 8550 2400
F 0 "#PWR0245" H 8550 2150 50  0001 C CNN
F 1 "GND" H 8555 2227 50  0000 C CNN
F 2 "" H 8550 2400 50  0001 C CNN
F 3 "" H 8550 2400 50  0001 C CNN
	1    8550 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0246
U 1 1 6317A7F6
P 9800 2400
F 0 "#PWR0246" H 9800 2150 50  0001 C CNN
F 1 "GND" H 9805 2227 50  0000 C CNN
F 2 "" H 9800 2400 50  0001 C CNN
F 3 "" H 9800 2400 50  0001 C CNN
	1    9800 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 1500 8550 1400
Wire Wire Line
	8850 1900 8950 1900
Wire Wire Line
	8550 2400 8550 2300
Wire Wire Line
	9800 2400 9800 2300
Wire Wire Line
	10100 1900 10200 1900
Wire Wire Line
	9800 1500 9800 1400
$Comp
L power:+5V #PWR0247
U 1 1 631927F7
P 9800 1400
F 0 "#PWR0247" H 9800 1250 50  0001 C CNN
F 1 "+5V" H 9815 1573 50  0000 C CNN
F 2 "" H 9800 1400 50  0001 C CNN
F 3 "" H 9800 1400 50  0001 C CNN
	1    9800 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C127
U 1 1 631C10C4
P 9500 3650
F 0 "C127" H 9618 3696 50  0000 L CNN
F 1 "470" H 9618 3605 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x6.5" H 9538 3500 50  0001 C CNN
F 3 "~" H 9500 3650 50  0001 C CNN
	1    9500 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 3500 9050 3450
Connection ~ 9050 3450
Wire Wire Line
	9050 3450 9500 3450
Wire Wire Line
	9500 3450 9500 3500
Wire Wire Line
	9050 3800 9050 3900
Wire Wire Line
	9500 3800 9500 3900
$Comp
L power:GND #PWR0254
U 1 1 631F4F1F
P 9500 3900
F 0 "#PWR0254" H 9500 3650 50  0001 C CNN
F 1 "GND" H 9505 3727 50  0000 C CNN
F 2 "" H 9500 3900 50  0001 C CNN
F 3 "" H 9500 3900 50  0001 C CNN
	1    9500 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 3250 9500 3450
Connection ~ 9500 3450
$Comp
L pspice:INDUCTOR L3
U 1 1 632088FA
P 9900 3450
F 0 "L3" H 9750 3550 50  0000 C CNN
F 1 "47uH" H 9900 3400 50  0000 C CNN
F 2 "Inductor_SMD:L_Bourns-SRU8028_8.0x8.0mm" H 9900 3450 50  0001 C CNN
F 3 "~" H 9900 3450 50  0001 C CNN
	1    9900 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C129
U 1 1 63208900
P 10200 3650
F 0 "C129" H 10315 3696 50  0000 L CNN
F 1 "0.1" H 10315 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 10238 3500 50  0001 C CNN
F 3 "~" H 10200 3650 50  0001 C CNN
	1    10200 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C131
U 1 1 63208906
P 10650 3650
F 0 "C131" H 10768 3696 50  0000 L CNN
F 1 "100" H 10768 3605 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 10688 3500 50  0001 C CNN
F 3 "~" H 10650 3650 50  0001 C CNN
	1    10650 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 3450 10150 3450
Wire Wire Line
	10200 3500 10200 3450
Connection ~ 10200 3450
Wire Wire Line
	10200 3450 10650 3450
Wire Wire Line
	10650 3450 10650 3500
Wire Wire Line
	10200 3800 10200 3900
Wire Wire Line
	10650 3800 10650 3900
Wire Wire Line
	10650 3250 10650 3450
Connection ~ 10650 3450
Wire Wire Line
	9500 3450 9650 3450
$Comp
L Device:C C126
U 1 1 6321FD7E
P 9050 5150
F 0 "C126" H 9165 5196 50  0000 L CNN
F 1 "0.1" H 9165 5105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 9088 5000 50  0001 C CNN
F 3 "~" H 9050 5150 50  0001 C CNN
	1    9050 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C128
U 1 1 6321FD84
P 9500 5150
F 0 "C128" H 9618 5196 50  0000 L CNN
F 1 "470" H 9618 5105 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x6.5" H 9538 5000 50  0001 C CNN
F 3 "~" H 9500 5150 50  0001 C CNN
	1    9500 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 5000 9050 4950
Connection ~ 9050 4950
Wire Wire Line
	9050 4950 9500 4950
Wire Wire Line
	9500 4950 9500 5000
Wire Wire Line
	9050 5300 9050 5400
Wire Wire Line
	9500 5300 9500 5400
$Comp
L power:GND #PWR0262
U 1 1 6321FDB5
P 9050 5400
F 0 "#PWR0262" H 9050 5150 50  0001 C CNN
F 1 "GND" H 9055 5227 50  0000 C CNN
F 2 "" H 9050 5400 50  0001 C CNN
F 3 "" H 9050 5400 50  0001 C CNN
	1    9050 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0263
U 1 1 6321FDBB
P 9500 5400
F 0 "#PWR0263" H 9500 5150 50  0001 C CNN
F 1 "GND" H 9505 5227 50  0000 C CNN
F 2 "" H 9500 5400 50  0001 C CNN
F 3 "" H 9500 5400 50  0001 C CNN
	1    9500 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 4750 9500 4950
Connection ~ 9500 4950
$Comp
L pspice:INDUCTOR L4
U 1 1 6321FDC3
P 9900 4950
F 0 "L4" H 9750 5050 50  0000 C CNN
F 1 "47uH" H 9900 4900 50  0000 C CNN
F 2 "Inductor_SMD:L_Bourns-SRU8028_8.0x8.0mm" H 9900 4950 50  0001 C CNN
F 3 "~" H 9900 4950 50  0001 C CNN
	1    9900 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C130
U 1 1 6321FDC9
P 10200 5150
F 0 "C130" H 10315 5196 50  0000 L CNN
F 1 "0.1" H 10315 5105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 10238 5000 50  0001 C CNN
F 3 "~" H 10200 5150 50  0001 C CNN
	1    10200 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C132
U 1 1 6321FDCF
P 10650 5150
F 0 "C132" H 10768 5196 50  0000 L CNN
F 1 "100" H 10768 5105 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 10688 5000 50  0001 C CNN
F 3 "~" H 10650 5150 50  0001 C CNN
	1    10650 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 4950 10150 4950
Wire Wire Line
	10200 5000 10200 4950
Connection ~ 10200 4950
Wire Wire Line
	10200 4950 10650 4950
Wire Wire Line
	10650 4950 10650 5000
Wire Wire Line
	10200 5300 10200 5400
Wire Wire Line
	10650 5300 10650 5400
Wire Wire Line
	10650 4750 10650 4950
Connection ~ 10650 4950
Wire Wire Line
	9500 4950 9650 4950
$Comp
L power:GND #PWR0135
U 1 1 60E0C973
P 4200 3850
F 0 "#PWR0135" H 4200 3600 50  0001 C CNN
F 1 "GND" H 4205 3677 50  0000 C CNN
F 2 "" H 4200 3850 50  0001 C CNN
F 3 "" H 4200 3850 50  0001 C CNN
	1    4200 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3850 4200 3800
$Comp
L power:GND #PWR0248
U 1 1 60E0C982
P 3650 3650
F 0 "#PWR0248" H 3650 3400 50  0001 C CNN
F 1 "GND" H 3655 3477 50  0000 C CNN
F 2 "" H 3650 3650 50  0001 C CNN
F 3 "" H 3650 3650 50  0001 C CNN
	1    3650 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3600 3650 3600
Wire Wire Line
	3650 3600 3650 3650
$Comp
L pspice:INDUCTOR L1
U 1 1 60E0C990
P 5500 3600
F 0 "L1" H 5350 3700 50  0000 C CNN
F 1 "330" H 5500 3550 50  0000 C CNN
F 2 "Inductor_SMD:L_12x12mm_H4.5mm" H 5500 3600 50  0001 C CNN
F 3 "~" H 5500 3600 50  0001 C CNN
	1    5500 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C136
U 1 1 60E0C996
P 5800 3800
F 0 "C136" H 5915 3846 50  0000 L CNN
F 1 "0.1" H 5915 3755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 5838 3650 50  0001 C CNN
F 3 "~" H 5800 3800 50  0001 C CNN
	1    5800 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C137
U 1 1 60E0C99C
P 6250 3800
F 0 "C137" H 6368 3846 50  0000 L CNN
F 1 "100" H 6368 3755 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 6288 3650 50  0001 C CNN
F 3 "~" H 6250 3800 50  0001 C CNN
	1    6250 3800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R159
U 1 1 60E0C9A2
P 5500 3400
F 0 "R159" V 5400 3400 50  0000 C CNN
F 1 "11K" V 5500 3400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5430 3400 50  0001 C CNN
F 3 "~" H 5500 3400 50  0001 C CNN
	1    5500 3400
	0    1    1    0   
$EndComp
$Comp
L Device:R R158
U 1 1 60E0C9A8
P 5150 3850
F 0 "R158" V 5050 3850 50  0000 C CNN
F 1 "2K" V 5150 3850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 5080 3850 50  0001 C CNN
F 3 "~" H 5150 3850 50  0001 C CNN
	1    5150 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	4700 3600 4850 3600
Wire Wire Line
	4850 3700 4850 3600
Connection ~ 4850 3600
Wire Wire Line
	4850 3600 5250 3600
Wire Wire Line
	4700 3400 5150 3400
Wire Wire Line
	5150 3700 5150 3400
Connection ~ 5150 3400
Wire Wire Line
	5150 3400 5350 3400
Wire Wire Line
	5800 3400 5800 3600
Wire Wire Line
	5800 3600 5750 3600
Wire Wire Line
	5650 3400 5800 3400
Wire Wire Line
	5800 3650 5800 3600
Connection ~ 5800 3600
Wire Wire Line
	5800 3600 6250 3600
Wire Wire Line
	6250 3600 6250 3650
Wire Wire Line
	5150 4000 5150 4050
Wire Wire Line
	4850 4000 4850 4050
Wire Wire Line
	5800 3950 5800 4050
Wire Wire Line
	6250 3950 6250 4050
$Comp
L power:GND #PWR0249
U 1 1 60E0C9C1
P 4850 4050
F 0 "#PWR0249" H 4850 3800 50  0001 C CNN
F 1 "GND" H 4855 3877 50  0000 C CNN
F 2 "" H 4850 4050 50  0001 C CNN
F 3 "" H 4850 4050 50  0001 C CNN
	1    4850 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0250
U 1 1 60E0C9C7
P 5150 4050
F 0 "#PWR0250" H 5150 3800 50  0001 C CNN
F 1 "GND" H 5155 3877 50  0000 C CNN
F 2 "" H 5150 4050 50  0001 C CNN
F 3 "" H 5150 4050 50  0001 C CNN
	1    5150 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0251
U 1 1 60E0C9CD
P 5800 4050
F 0 "#PWR0251" H 5800 3800 50  0001 C CNN
F 1 "GND" H 5805 3877 50  0000 C CNN
F 2 "" H 5800 4050 50  0001 C CNN
F 3 "" H 5800 4050 50  0001 C CNN
	1    5800 4050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0252
U 1 1 60E0C9D3
P 6250 4050
F 0 "#PWR0252" H 6250 3800 50  0001 C CNN
F 1 "GND" H 6255 3877 50  0000 C CNN
F 2 "" H 6250 4050 50  0001 C CNN
F 3 "" H 6250 4050 50  0001 C CNN
	1    6250 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 3400 6250 3600
Connection ~ 6250 3600
$Comp
L power:+12V #PWR0257
U 1 1 60E1580C
P 6250 3400
F 0 "#PWR0257" H 6250 3250 50  0001 C CNN
F 1 "+12V" H 6265 3573 50  0000 C CNN
F 2 "" H 6250 3400 50  0001 C CNN
F 3 "" H 6250 3400 50  0001 C CNN
	1    6250 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C135
U 1 1 60E1CF74
P 4300 4600
F 0 "C135" H 4418 4646 50  0000 L CNN
F 1 "220" H 4418 4555 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_8x6.2" H 4338 4450 50  0001 C CNN
F 3 "~" H 4300 4600 50  0001 C CNN
	1    4300 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 4800 4300 4800
Wire Wire Line
	4300 4800 4300 4750
Connection ~ 3900 4800
Wire Wire Line
	3900 4300 4300 4300
Wire Wire Line
	4300 4300 4300 4450
$Comp
L Regulator_Switching:LM2596S-ADJ U1
U 1 1 60E0C96D
P 4200 3500
F 0 "U1" H 4200 3867 50  0000 C CNN
F 1 "LM2596S-ADJ" H 4200 3776 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-5_TabPin3" H 4250 3250 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm2596.pdf" H 4200 3500 50  0001 C CNN
	1    4200 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3400 3450 3400
Wire Wire Line
	3450 3400 3450 4300
$Comp
L Device:D D15
U 1 1 60EFEADF
P 4850 3850
F 0 "D15" H 4850 3633 50  0000 C CNN
F 1 "1N4148" H 4850 3724 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 4850 3850 50  0001 C CNN
F 3 "~" H 4850 3850 50  0001 C CNN
	1    4850 3850
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0253
U 1 1 631F22EE
P 9050 3900
F 0 "#PWR0253" H 9050 3650 50  0001 C CNN
F 1 "GND" H 9055 3727 50  0000 C CNN
F 2 "" H 9050 3900 50  0001 C CNN
F 3 "" H 9050 3900 50  0001 C CNN
	1    9050 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C125
U 1 1 631C10BE
P 9050 3650
F 0 "C125" H 9165 3696 50  0000 L CNN
F 1 "0.1" H 9165 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 9088 3500 50  0001 C CNN
F 3 "~" H 9050 3650 50  0001 C CNN
	1    9050 3650
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:LM1085-ADJ U17
U 1 1 60F3FE1A
P 8200 3450
F 0 "U17" H 8200 3692 50  0000 C CNN
F 1 "LM1085-ADJ" H 8200 3601 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-3_TabPin2" H 8200 3700 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm1085.pdf" H 8200 3450 50  0001 C CNN
	1    8200 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R160
U 1 1 60F53A9F
P 8650 3700
F 0 "R160" V 8550 3700 50  0000 C CNN
F 1 "360R" V 8650 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8580 3700 50  0001 C CNN
F 3 "~" H 8650 3700 50  0001 C CNN
	1    8650 3700
	-1   0    0    1   
$EndComp
$Comp
L Device:R R161
U 1 1 60F5886D
P 8650 4100
F 0 "R161" V 8550 4100 50  0000 C CNN
F 1 "1K1" V 8650 4100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8580 4100 50  0001 C CNN
F 3 "~" H 8650 4100 50  0001 C CNN
	1    8650 4100
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0258
U 1 1 60F5DA80
P 8650 4350
F 0 "#PWR0258" H 8650 4100 50  0001 C CNN
F 1 "GND" H 8655 4177 50  0000 C CNN
F 2 "" H 8650 4350 50  0001 C CNN
F 3 "" H 8650 4350 50  0001 C CNN
	1    8650 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 3450 8650 3450
Wire Wire Line
	8650 3550 8650 3450
Connection ~ 8650 3450
Wire Wire Line
	8650 3450 9050 3450
Wire Wire Line
	8650 3850 8650 3900
Wire Wire Line
	8650 4250 8650 4350
Wire Wire Line
	8650 3900 8200 3900
Wire Wire Line
	8200 3900 8200 3750
Connection ~ 8650 3900
Wire Wire Line
	8650 3900 8650 3950
Wire Wire Line
	8200 4250 8200 4350
$Comp
L power:GND #PWR0259
U 1 1 60F7C574
P 8200 4350
F 0 "#PWR0259" H 8200 4100 50  0001 C CNN
F 1 "GND" H 8205 4177 50  0000 C CNN
F 2 "" H 8200 4350 50  0001 C CNN
F 3 "" H 8200 4350 50  0001 C CNN
	1    8200 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C138
U 1 1 60F7C57A
P 8200 4100
F 0 "C138" H 8315 4146 50  0000 L CNN
F 1 "470pF" H 8315 4055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 8238 3950 50  0001 C CNN
F 3 "~" H 8200 4100 50  0001 C CNN
	1    8200 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 3950 8200 3900
Connection ~ 8200 3900
$Comp
L Regulator_Linear:LM1085-ADJ U18
U 1 1 60F8B274
P 8200 4950
F 0 "U18" H 8200 5192 50  0000 C CNN
F 1 "LM1085-ADJ" H 8200 5101 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-3_TabPin2" H 8200 5200 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/lm1085.pdf" H 8200 4950 50  0001 C CNN
	1    8200 4950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R162
U 1 1 60F8B27A
P 8650 5200
F 0 "R162" V 8550 5200 50  0000 C CNN
F 1 "620R" V 8650 5200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8580 5200 50  0001 C CNN
F 3 "~" H 8650 5200 50  0001 C CNN
	1    8650 5200
	-1   0    0    1   
$EndComp
$Comp
L Device:R R163
U 1 1 60F8B280
P 8650 5600
F 0 "R163" V 8550 5600 50  0000 C CNN
F 1 "1K" V 8650 5600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 8580 5600 50  0001 C CNN
F 3 "~" H 8650 5600 50  0001 C CNN
	1    8650 5600
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0260
U 1 1 60F8B286
P 8650 5850
F 0 "#PWR0260" H 8650 5600 50  0001 C CNN
F 1 "GND" H 8655 5677 50  0000 C CNN
F 2 "" H 8650 5850 50  0001 C CNN
F 3 "" H 8650 5850 50  0001 C CNN
	1    8650 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 4950 8650 4950
Wire Wire Line
	8650 5050 8650 4950
Connection ~ 8650 4950
Wire Wire Line
	8650 4950 9050 4950
Wire Wire Line
	8650 5350 8650 5400
Wire Wire Line
	8650 5750 8650 5850
Wire Wire Line
	8650 5400 8200 5400
Wire Wire Line
	8200 5400 8200 5250
Connection ~ 8650 5400
Wire Wire Line
	8650 5400 8650 5450
Wire Wire Line
	8200 5750 8200 5850
$Comp
L power:GND #PWR0261
U 1 1 60F8B297
P 8200 5850
F 0 "#PWR0261" H 8200 5600 50  0001 C CNN
F 1 "GND" H 8205 5677 50  0000 C CNN
F 2 "" H 8200 5850 50  0001 C CNN
F 3 "" H 8200 5850 50  0001 C CNN
	1    8200 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C139
U 1 1 60F8B29D
P 8200 5600
F 0 "C139" H 8315 5646 50  0000 L CNN
F 1 "470pF" H 8315 5555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.08x0.95mm_HandSolder" H 8238 5450 50  0001 C CNN
F 3 "~" H 8200 5600 50  0001 C CNN
	1    8200 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 5450 8200 5400
Connection ~ 8200 5400
$Comp
L power:+12V #PWR0267
U 1 1 60FAB5F2
P 7800 3400
F 0 "#PWR0267" H 7800 3250 50  0001 C CNN
F 1 "+12V" H 7815 3573 50  0000 C CNN
F 2 "" H 7800 3400 50  0001 C CNN
F 3 "" H 7800 3400 50  0001 C CNN
	1    7800 3400
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0268
U 1 1 60FB2433
P 7800 4900
F 0 "#PWR0268" H 7800 4750 50  0001 C CNN
F 1 "+12V" H 7815 5073 50  0000 C CNN
F 2 "" H 7800 4900 50  0001 C CNN
F 3 "" H 7800 4900 50  0001 C CNN
	1    7800 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 3450 7800 3450
Wire Wire Line
	7800 3450 7800 3400
Wire Wire Line
	7900 4950 7800 4950
Wire Wire Line
	7800 4950 7800 4900
$Comp
L power:GNDA #PWR0255
U 1 1 611C505E
P 10650 3900
F 0 "#PWR0255" H 10650 3650 50  0001 C CNN
F 1 "GNDA" H 10655 3727 50  0000 C CNN
F 2 "" H 10650 3900 50  0001 C CNN
F 3 "" H 10650 3900 50  0001 C CNN
	1    10650 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0256
U 1 1 611C6BBA
P 10200 3900
F 0 "#PWR0256" H 10200 3650 50  0001 C CNN
F 1 "GNDA" H 10205 3727 50  0000 C CNN
F 2 "" H 10200 3900 50  0001 C CNN
F 3 "" H 10200 3900 50  0001 C CNN
	1    10200 3900
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0264
U 1 1 611CDED6
P 10200 5400
F 0 "#PWR0264" H 10200 5150 50  0001 C CNN
F 1 "GNDA" H 10205 5227 50  0000 C CNN
F 2 "" H 10200 5400 50  0001 C CNN
F 3 "" H 10200 5400 50  0001 C CNN
	1    10200 5400
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0265
U 1 1 611D456C
P 10650 5400
F 0 "#PWR0265" H 10650 5150 50  0001 C CNN
F 1 "GNDA" H 10655 5227 50  0000 C CNN
F 2 "" H 10650 5400 50  0001 C CNN
F 3 "" H 10650 5400 50  0001 C CNN
	1    10650 5400
	1    0    0    -1  
$EndComp
$Comp
L Power_Supervisor:MCP130-xxxxTT U19
U 1 1 60E64E14
P 8550 1900
F 0 "U19" H 8450 1950 50  0000 R CNN
F 1 "MCP130-xxxxTT" H 9200 1550 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 9150 1550 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/11184d.pdf" H 8550 1900 50  0001 C CNN
	1    8550 1900
	1    0    0    -1  
$EndComp
$Comp
L Power_Supervisor:MCP130-xxxxTT U20
U 1 1 60E778D1
P 9800 1900
F 0 "U20" H 9700 1950 50  0000 R CNN
F 1 "MCP130-xxxxTT" H 10450 1550 50  0000 R CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 10400 1550 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/11184d.pdf" H 9800 1900 50  0001 C CNN
	1    9800 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0195
U 1 1 614186E1
P 4500 6250
F 0 "#PWR0195" H 4500 6000 50  0001 C CNN
F 1 "GND" H 4505 6077 50  0000 C CNN
F 2 "" H 4500 6250 50  0001 C CNN
F 3 "" H 4500 6250 50  0001 C CNN
	1    4500 6250
	1    0    0    -1  
$EndComp
$Comp
L power:GNDA #PWR0201
U 1 1 614186E7
P 5200 6250
F 0 "#PWR0201" H 5200 6000 50  0001 C CNN
F 1 "GNDA" H 5205 6077 50  0000 C CNN
F 2 "" H 5200 6250 50  0001 C CNN
F 3 "" H 5200 6250 50  0001 C CNN
	1    5200 6250
	1    0    0    -1  
$EndComp
$Comp
L pspice:INDUCTOR L2
U 1 1 6141EE6F
P 4850 6100
F 0 "L2" H 4850 6315 50  0000 C CNN
F 1 "FB" H 4850 6224 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4850 6100 50  0001 C CNN
F 3 "~" H 4850 6100 50  0001 C CNN
	1    4850 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 6250 5200 6100
Wire Wire Line
	5200 6100 5100 6100
Wire Wire Line
	4600 6100 4500 6100
Wire Wire Line
	4500 6100 4500 6250
Text HLabel 5400 2350 3    50   Input ~ 0
MAIN_TRF_SS
$Comp
L Relay:G2RL-2-DC12 RL1
U 1 1 616FA480
P 3650 1900
F 0 "RL1" H 4050 1300 50  0000 R CNN
F 1 "G2RL-2-DC12" H 4050 1400 50  0000 R CNN
F 2 "Relay_THT:Relay_DPDT_Omron_G2RL" H 4300 1850 50  0001 L CNN
F 3 "https://omronfs.omron.com/en_US/ecb/products/pdf/en-g2rl.pdf" H 3650 1900 50  0001 C CNN
	1    3650 1900
	-1   0    0    1   
$EndComp
Wire Wire Line
	2850 2300 3150 2300
Wire Wire Line
	3150 2300 3150 2200
Connection ~ 2850 2300
Wire Wire Line
	3150 2300 3550 2300
Wire Wire Line
	3550 2300 3550 2200
Connection ~ 3150 2300
Wire Wire Line
	2850 1500 3250 1500
Wire Wire Line
	3650 1500 3650 1600
Connection ~ 2850 1500
Wire Wire Line
	3250 1600 3250 1500
Connection ~ 3250 1500
Wire Wire Line
	3250 1500 3650 1500
$Comp
L Relay:G2RL-2-DC12 RL2
U 1 1 617224A5
P 5000 1900
F 0 "RL2" H 4370 1854 50  0000 R CNN
F 1 "G2RL-2-DC12" H 4370 1945 50  0000 R CNN
F 2 "Relay_THT:Relay_DPDT_Omron_G2RL" H 5650 1850 50  0001 L CNN
F 3 "https://omronfs.omron.com/en_US/ecb/products/pdf/en-g2rl.pdf" H 5000 1900 50  0001 C CNN
	1    5000 1900
	-1   0    0    1   
$EndComp
Wire Wire Line
	3650 1500 4600 1500
Wire Wire Line
	5000 1500 5000 1600
Connection ~ 3650 1500
Wire Wire Line
	4600 1600 4600 1500
Connection ~ 4600 1500
Wire Wire Line
	4600 1500 5000 1500
$Comp
L Device:R R26
U 1 1 61740C83
P 4300 2300
F 0 "R26" V 4200 2300 50  0000 C CNN
F 1 "47R/15W" V 4400 2300 50  0000 C CNN
F 2 "Resistor_THT:R_Radial_Power_L13.0mm_W9.0mm_P5.00mm" V 4230 2300 50  0001 C CNN
F 3 "~" H 4300 2300 50  0001 C CNN
	1    4300 2300
	0    1    1    0   
$EndComp
Wire Wire Line
	3550 2300 4150 2300
Connection ~ 3550 2300
Wire Wire Line
	4900 2300 4900 2200
Wire Wire Line
	4450 2300 4500 2300
Wire Wire Line
	4500 2200 4500 2300
Connection ~ 4500 2300
Wire Wire Line
	4500 2300 4900 2300
Wire Wire Line
	5400 2350 5400 2250
Wire Wire Line
	4050 2350 4050 2250
$Comp
L power:+12V #PWR0207
U 1 1 617973F5
P 5400 1400
F 0 "#PWR0207" H 5400 1250 50  0001 C CNN
F 1 "+12V" H 5415 1573 50  0000 C CNN
F 2 "" H 5400 1400 50  0001 C CNN
F 3 "" H 5400 1400 50  0001 C CNN
	1    5400 1400
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0266
U 1 1 6179DD42
P 4050 1400
F 0 "#PWR0266" H 4050 1250 50  0001 C CNN
F 1 "+12V" H 4065 1573 50  0000 C CNN
F 2 "" H 4050 1400 50  0001 C CNN
F 3 "" H 4050 1400 50  0001 C CNN
	1    4050 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 1600 4050 1400
Wire Wire Line
	5400 1400 5400 1600
$Comp
L Device:D D?
U 1 1 618034AE
P 3900 2500
AR Path="/60BF25B5/618034AE" Ref="D?"  Part="1" 
AR Path="/60BF2601/618034AE" Ref="D3"  Part="1" 
F 0 "D3" H 3900 2300 50  0000 L CNN
F 1 "1N4148" H 3800 2400 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 3900 2500 50  0001 C CNN
F 3 "~" H 3900 2500 50  0001 C CNN
	1    3900 2500
	0    1    1    0   
$EndComp
$Comp
L Device:D D?
U 1 1 61809CDA
P 5200 2500
AR Path="/60BF25B5/61809CDA" Ref="D?"  Part="1" 
AR Path="/60BF2601/61809CDA" Ref="D4"  Part="1" 
F 0 "D4" H 5200 2300 50  0000 L CNN
F 1 "1N4148" H 5100 2400 50  0000 L CNN
F 2 "Diode_SMD:D_SOD-323_HandSoldering" H 5200 2500 50  0001 C CNN
F 3 "~" H 5200 2500 50  0001 C CNN
	1    5200 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	3900 2350 3900 2250
Wire Wire Line
	3900 2250 4050 2250
Connection ~ 4050 2250
Wire Wire Line
	4050 2250 4050 2200
Wire Wire Line
	5200 2350 5200 2250
Wire Wire Line
	5200 2250 5400 2250
Connection ~ 5400 2250
Wire Wire Line
	5400 2250 5400 2200
$Comp
L power:+12V #PWR0273
U 1 1 6181EE52
P 5200 2750
F 0 "#PWR0273" H 5200 2600 50  0001 C CNN
F 1 "+12V" H 5215 2923 50  0000 C CNN
F 2 "" H 5200 2750 50  0001 C CNN
F 3 "" H 5200 2750 50  0001 C CNN
	1    5200 2750
	-1   0    0    1   
$EndComp
Wire Wire Line
	5200 2650 5200 2750
$Comp
L power:+12V #PWR0274
U 1 1 6182ED18
P 3900 2750
F 0 "#PWR0274" H 3900 2600 50  0001 C CNN
F 1 "+12V" H 3915 2923 50  0000 C CNN
F 2 "" H 3900 2750 50  0001 C CNN
F 3 "" H 3900 2750 50  0001 C CNN
	1    3900 2750
	-1   0    0    1   
$EndComp
Wire Wire Line
	3900 2650 3900 2750
$EndSCHEMATC
