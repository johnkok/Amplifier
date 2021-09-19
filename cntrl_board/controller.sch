EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 5
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 9000 3000 2000 1500
U 60BF255D
F0 "dsp" 50
F1 "dsp.sch" 50
F2 "MCLK" I R 11000 4350 50 
F3 "SDA" I L 9000 3350 50 
F4 "SCL" I L 9000 3500 50 
F5 "nRST" I L 9000 4000 50 
F6 "DS_DAT4" I R 11000 4200 50 
F7 "DS_DAT3" I R 11000 4100 50 
F8 "DS_DAT2" I R 11000 4000 50 
F9 "DS_DAT1" I R 11000 3900 50 
F10 "DBCLK" I R 11000 3800 50 
F11 "DLRCLK" I R 11000 3700 50 
F12 "ABCLK" I R 11000 3250 50 
F13 "AS_DAT2" I R 11000 3450 50 
F14 "AS_DAT1" I R 11000 3350 50 
F15 "ALRCLK" I R 11000 3150 50 
F16 "WR_BACK" I L 9000 4150 50 
$EndSheet
$Sheet
S 4000 3000 2000 1500
U 60BF25B5
F0 "ucu" 50
F1 "ucu.sch" 50
F2 "nRST" I R 6000 3250 50 
F3 "CODEC_SDA" I R 6000 3650 50 
F4 "CODEC_SCL" I R 6000 3750 50 
F5 "DSP_SDA" I R 6000 3950 50 
F6 "DSP_SCL" I R 6000 4050 50 
F7 "AC_IN" I R 6000 4200 50 
F8 "AC_CTRL" I R 6000 4300 50 
F9 "AMP_REF_IN" I R 6000 4400 50 
$EndSheet
$Sheet
S 6500 3000 2000 1500
U 60BF2601
F0 "ps" 50
F1 "ps.sch" 50
F2 "MAIN_TRF_CTRL" I L 6500 4300 50 
F3 "MAIN_POL" I L 6500 4200 50 
F4 "WR_BACK" I R 8500 4150 50 
F5 "nRST" I R 8500 4000 50 
$EndSheet
$Sheet
S 11500 3000 2000 1500
U 60CBC38F
F0 "codec" 50
F1 "codec.sch" 50
F2 "MCLK" I L 11500 4350 50 
F3 "SCL" I R 13500 3500 50 
F4 "nRST" I R 13500 4250 50 
F5 "SDA" I R 13500 3350 50 
F6 "DS_DAT4" I L 11500 4200 50 
F7 "DS_DAT3" I L 11500 4100 50 
F8 "DS_DAT2" I L 11500 4000 50 
F9 "DS_DAT1" I L 11500 3900 50 
F10 "DBCLK" I L 11500 3800 50 
F11 "DLRCLK" I L 11500 3700 50 
F12 "AS_DAT2" I L 11500 3450 50 
F13 "AS_DAT1" I L 11500 3350 50 
F14 "ABCLK" I L 11500 3250 50 
F15 "ALRCLK" I L 11500 3150 50 
F16 "AMP_REF_IN" I R 13500 3900 50 
$EndSheet
Wire Wire Line
	11000 3150 11500 3150
Wire Wire Line
	11000 3250 11500 3250
Wire Wire Line
	11000 3350 11500 3350
Wire Wire Line
	11000 3450 11500 3450
Wire Wire Line
	11000 3700 11500 3700
Wire Wire Line
	11000 3800 11500 3800
Wire Wire Line
	11000 3900 11500 3900
Wire Wire Line
	11000 4000 11500 4000
Wire Wire Line
	11000 4100 11500 4100
Wire Wire Line
	11000 4200 11500 4200
Wire Wire Line
	11000 4350 11500 4350
Wire Wire Line
	8500 4000 8700 4000
Wire Wire Line
	9000 4150 8500 4150
Wire Wire Line
	13500 3350 13750 3350
Wire Wire Line
	8800 2600 8800 3350
Wire Wire Line
	8800 3350 9000 3350
Wire Wire Line
	9000 3500 8850 3500
Wire Wire Line
	8850 3500 8850 2650
Wire Wire Line
	13800 3500 13500 3500
Wire Wire Line
	13750 4900 13750 4250
Wire Wire Line
	13750 4250 13500 4250
Wire Wire Line
	6000 4200 6500 4200
Wire Wire Line
	6500 4300 6000 4300
Wire Wire Line
	6000 4050 6400 4050
Wire Wire Line
	6400 4050 6400 2650
Wire Wire Line
	6400 2650 8850 2650
Wire Wire Line
	8800 2600 6350 2600
Wire Wire Line
	6350 2600 6350 3950
Wire Wire Line
	6350 3950 6000 3950
Wire Wire Line
	6000 3750 6250 3750
Wire Wire Line
	6250 3750 6250 2500
Wire Wire Line
	6250 2500 13800 2500
Wire Wire Line
	13800 2500 13800 3500
Wire Wire Line
	13750 2450 6200 2450
Wire Wire Line
	6200 2450 6200 3650
Wire Wire Line
	6200 3650 6000 3650
Wire Wire Line
	13750 2450 13750 3350
Wire Wire Line
	6000 3250 6100 3250
Wire Wire Line
	6100 3250 6100 2350
Wire Wire Line
	6100 2350 8700 2350
Wire Wire Line
	8700 2350 8700 4000
Connection ~ 8700 4000
Wire Wire Line
	8700 4000 8700 4900
Wire Wire Line
	8700 4000 9000 4000
Wire Wire Line
	8700 4900 13750 4900
Wire Wire Line
	6000 4400 6200 4400
Wire Wire Line
	6200 4400 6200 5050
Wire Wire Line
	6200 5050 13850 5050
Wire Wire Line
	13850 5050 13850 3900
Wire Wire Line
	13500 3900 13850 3900
$EndSCHEMATC