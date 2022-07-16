#if !defined(__SETTINGS_H_)
#define __SETTINGS_H_

// VOLT BMS SETTINGS

#define OVER_V_SETPOINT 4250 //cell voltage in mV
#define UNDER_V_SETPOINT 2900
#define CHARGE_V_SETPOINT 4180 // Voltage to turn off charger
#define CHARGE_HYS 160         // voltage drop required for charger to kick back on in mV
#define OVER_T_SETPOINT 14000    // Degrees F
#define UNDER_T_SETPOINT 0
#define IGNORE_TEMP 28700    // 0 - use both sensors, 1 or 2 only use that sensor
#define IGNORE_VOLT 500      //
#define BALANCE_VOLTAGE 3850 //Balance allowed over this voltage
#define BALANCE_HYS 5        // Minimum cell delta to begin balance in mV
#define S_CELLS 96           //Cells in series
#define SOC_VOLT_10 3400     //Voltage and SOC curve for voltage based SOC calc. Cell volt at 10% SOC, Cell volt at 90% SOC
#define SOC_VOLT_90 4100

// BMW I3 LIM CCS SETTINGS

#define  Voltspnt 0x0  


#endif // __SETTINGS_H_
