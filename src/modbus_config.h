#ifndef MODBUS_X_H_INCLUDED
#define MODBUS_X_H_INCLUDED

#include <stdint.h>
#include "main.h"
#include "modbus_hard.h"
#include "optic_current_data.h"

//-----------------------------------------------------------------------
// Modbus registers X macros
//-----------------------------------------------------------------------

//-----------------------------------------------------------------------
// define
//-----------------------------------------------------------------------
#define MB_LIMIT_REG	    1//check limit
#define MB_CALLBACK_REG	    1//use write callback
//#define MB_USER_ARG1_REG	1//use user argument (for example: run user callback after write function)
//#define MB_USER_ARG2_REG	1//not implement now
// If not define "MB_REG_END_TO_END" then number register is determined in "a" field from X-macros
//#define MB_REG_END_TO_END

#define REG_END_REGISTER                Reg_End

//  MAIN_BUF_Start_Table_Mask
#define USER_FUNC		(0x20)
#define USER_ARG		(0x10)
#define READ_R		    (0)
#define WRITE_R		    (0x01)	// 0 bit                        <--|
#define CB_WR		    (0x02)	// 1 bit                        <--|
#define LIM_SIGN		(0x04)	// 2 bit for limit              <--|
#define LIM_UNSIGN	    (0x08)  // 3 bit for limit	            <--|
#define LIM_MASK	    (0x0C)	// 2 and 3 bit for limit        <--|____________
//                                                                              |
#define LIM_BIT_MASK	        LIM_MASK
//	 Number		Name for enum	       Arg1  Default   Min	   Max     __________Options________
//										      Value   Level   Level   |                         |
//														     or Mask  |                         |
#define MB_BUF_TABLE\
	X_BUF(0,	REG_0,                  0,		0,		0,      0,      READ_R)\
	X_BUF(1,	REG_VOLT_INST,          0,	    0,	    0,		0,	    READ_R)\
    X_BUF(2,	REG_VOLT_MID,           0,	    0,	    0,		0,	    READ_R)\
    X_BUF(3,	REG_CUR_INST,           0,	    0,	    0,		0,	    READ_R)\
    X_BUF(4,	REG_CUR_MID,            0,	    0,	    0,		0,	    READ_R)\
    X_BUF(5,	REG_CUR_DIDT,           0,	    0,	    0,		0,	    READ_R)\
    X_BUF(6,	REG_CUR_MID_MV,         0,	    0,	    0,		0,	    READ_R)\
    X_BUF(9,	REG_STARTRACK,      	0,	    0,	    0,		0,	    WRITE_R)\
    X_BUF(10,	REG_ALARM,              0,	    0,	    0,		0,	    READ_R)\
    X_BUF(11,	REG_TRIG_ALARM,         0,	    0,	    0,		0,	    READ_R)\
 	X_BUF(12,	REG_DI_STATUS,          0,	    0,	    0,		0,	    READ_R)\
    X_BUF(13,	REG_DO_STATUS,          0,	    0,	    0,		0,	    READ_R)\
    X_BUF(14,	REG_MEASURE_LAN_ERROR,  0,      0,	    0,		0,	    READ_R)\
    X_BUF(15,	REG_PROTECTION_RESET,   0,	    0,	    0,		0,	    WRITE_R)\
    X_BUF(16,	REG_CALIB,      	    0,	    0,	    0,		0,	    WRITE_R)\
    X_BUF(18,	REG_SET_AUTO_RESET,     0,	    0,      0,      0,      WRITE_R | CB_WR)\
    X_BUF(19,	REG_AUTO_RESET_S,       0,	    0,      0,      0,      WRITE_R | CB_WR)\
    X_BUF(20,	REG_CUR_SHUNT_MV,       0,	    600,    100,    2000,   WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(21,	REG_CUR_SHUNT_A,        0,	    5000,   10,		10000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(23,	REG_DELTA_V_VALUE,      0,	    15,	    0,		0xFFFF, WRITE_R | CB_WR)\
    X_BUF(24,	REG_CALIB_MID_VALUE,    0,	    0,	    0,		0xFFFF, WRITE_R | CB_WR)\
    X_BUF(25,	REG_CALIB_INST_VALUE,   0,	    0,	    0,		0xFFFF, WRITE_R | CB_WR)\
    X_BUF(26,	REG_CALIB_V_MID_VALUE,  0,	    0,	    0,		0xFFFF, WRITE_R | CB_WR)\
    X_BUF(30,	REG_RS_BAUD,	        0,		1,		0,		0x03,	WRITE_R | CB_WR | LIM_MASK)\
	X_BUF(31,	REG_RS_DELAY,           0,	    5,		0,      100,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(32,	REG_RS_ADDR,            0,		1,		1,		0xFA,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(33,	REG_RS_PARITY_STOP, 	0,	    0,	    0,		0x03,	WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(35,	REG_RESET_TO_DEFAULT,   0,	    0,	    0,		0,	    WRITE_R)\
    X_BUF(36,	REG_DATA_SAVE,          0,	    0,	    0,		0,	    WRITE_R)\
    X_BUF(38,	REG_EXTERNAL_ALARM,     0,	    0,	    0,	    0,      WRITE_R)\
    X_BUF(40,	REG_PROTECT_ENABLE_BIT, 0,	    0,	    0,		PR_ALARM_ALL_MSK,\
                                                                        WRITE_R | CB_WR | LIM_MASK)\
    X_BUF(42,	REG_CUR_MAX_P,          0,	    0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(43,	REG_CUR_MAX_N,          0,	    0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(44,	REG_CUR_OL_P_VALUE,     0,      0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(45,	REG_CUR_OL_P_TIME_MS,   0,	    0,	    0,	    60000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(46,	REG_CUR_OL_N_VALUE,     0,      0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(47,	REG_CUR_OL_N_TIME_MS,   0,	    0,	    0,	    60000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(48,	REG_CUR_DIDT_P_VALUE,   0,      0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(49,	REG_CUR_DIDT_P_TIME_MS, 0,	    0,	    0,	    60000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(50,	REG_CUR_DIDT_N_VALUE,   0,      0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(51,	REG_CUR_DIDT_N_TIME_MS, 0,	    0,	    0,	    60000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(52,	REG_VOLT_MIN,           0,	    0,	    -2000,	2000,   WRITE_R | CB_WR | LIM_SIGN)\
    X_BUF(53,	REG_VOLT_MIN_TIME_MS,   0,	    0,	    0,	    60000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(54,	REG_VOLT_MAX,           0,	    0,	    -2000,	2000,   WRITE_R | CB_WR | LIM_SIGN)\
    X_BUF(55,	REG_VOLT_MAX_TIME_MS,   0,	    0,	    0,	    60000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(61,	REG_DI_1_MODE,          0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(62,	REG_DI_2_MODE,          0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(63,	REG_DI_3_MODE,          0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(64,	REG_DI_4_MODE,          0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(71,	REG_DO_1_MODE,          0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(72,	REG_DO_2_MODE,          0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(73,	REG_DO_3_MODE,          0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(74,	REG_DO_4_MODE,          0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(75,	REG_DO_1_ALARM_MSK,     0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(76,	REG_DO_2_ALARM_MSK,     0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(77,	REG_DO_3_ALARM_MSK,     0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(78,	REG_DO_4_ALARM_MSK,     0,	    0,	    0,	    0,      WRITE_R | CB_WR)\
    X_BUF(80,	REG_DO_ON,              0,	    0,	    0,		0,	    WRITE_R)\
    X_BUF(81,	REG_DO_OFF,             0,	    0,	    0,		0,	    WRITE_R)\
    X_BUF(97,	REG_GOTRACK,      	    0,	    0,	    0,		0,	    WRITE_R)\
	X_BUF(99,	REG_TRECK_SCALE,        0,	    0,      0,      3,      WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(TRACK_CUR_REG_START,\
                REG_TRECK_START,	    0,	    0,      0,      0,      READ_R)\
	X_BUF(TRACK_VOLT_REG_START + TRACK_POINT_NUM,\
                REG_TRECK_END,          0,	    0,      0,      0,      READ_R)\
	X_BUF(1129,	Reg_End,				0,	    0,      0,      0,      READ_R)\

#endif /* MODBUS_X_H_INCLUDED */
