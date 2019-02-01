# DL0WH_BiTuner_Firmware
Based on the DJ0ABR tuner hardware this project is a replacement for the controller PCB and its firmware. The new PCB does have a __STM32L476RGT6__ MCU device and has got preparations for SWR metering.


## V1.2 PCB after placement and routing was done:
![Screenshot of V1.2 no polyfill](https://raw.githubusercontent.com/DF4IAH/DL0WH_BiTuner/master/Docs/09_Results/Pictures/DL0WH_BiTuner_1V2_PCB_RoutingDone.png)

![Screenshot of V1.2 complete](https://raw.githubusercontent.com/DF4IAH/DL0WH_BiTuner/master/Docs/09_Results/Pictures/DL0WH_BiTuner_1V2_PCB_Complete.png)


## V1.2 MCU Pinout:
![MCU Pinout of V1.0](https://raw.githubusercontent.com/DF4IAH/DL0WH_BiTuner/master/Docs/09_Results/Pictures/DL0WH_BiTuner_1V0_SW_Pinout.png)


## Ready for samples:
The board is again ready for production.


## Version list:
V1.2 SWD connector bad size - 2019-02-01
* Library content for the CORTEX JTAG/SWD connector was wrong, fixed.
* PCB board house failed on trace distances of 6 mils. Design expanded to 8 mils.

---

V1.1 LQFP64 bug fix - 2019-01-18
* Due to a bad LQFP64 package within the STM32L4 library (who made that crappy library?) the device did not fit at the PCB. That made a turn-around to take time and money. Never trust any library again.

---

V1.0 initial version - 2019-01-04

---

73  Uli DF4IAH, Ladenburg, 2019-02-01
