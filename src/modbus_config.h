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
	X_BUF(0,	REG_0,		            0,		0,		0,      0,      READ_R)\
    X_BUF(1,	REG_CUR_MID,            0,	    0,	    0,		0,	    READ_R)\
    X_BUF(2,	REG_CUR_DIDT,           0,	    0,	    0,		0,	    READ_R)\
    X_BUF(3,	REG_CUR_INST,           0,	    0,	    0,		0,	    READ_R)\
    X_BUF(4,	REG_VOLT_INST,          0,	    0,	    0,		0,	    READ_R)\
    X_BUF(5,	REG_RESET,      	    0,	    0,	    0,		0,	    WRITE_R)\
    X_BUF(9,	REG_USER_CONTROL,       0,	    0,	    0,		0,	    WRITE_R)\
	X_BUF(15,	REG_CUR_TEST,           0,	    0,	    0,		0xFFFF,	WRITE_R | CB_WR)\
    X_BUF(16,	REG_CUR_ADD,			0,	    0,      0,      0,      WRITE_R | CB_WR)\
	X_BUF(18,	REG_DI_Bit,             0,	    0,	    0,		0,	    READ_R)\
    X_BUF(20,	REG_CUR_MAX_P,          0,	    0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(21,	REG_CUR_MAX_N,          0,	    0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(22,	REG_CUR_OL_P_VALUE,     0,      0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(23,	REG_CUR_OL_P_TIME_MS,   0,	    0,	    0,	    100,    WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(24,	REG_CUR_OL_N_VALUE,     0,      0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(25,	REG_CUR_OL_N_TIME_MS,   0,	    0,	    0,	    100,    WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(26,	REG_CUR_DIDT_P_VALUE,   0,      0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(27,	REG_CUR_DIDT_P_TIME_MS, 0,	    0,	    0,	    100,    WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(28,	REG_CUR_DIDT_N_VALUE,   0,      0,	    0,		20000,  WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(29,	REG_CUR_DIDT_N_TIME_MS, 0,	    0,	    0,	    100,    WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(30,	REG_DO_1_BIT_PROTECTION,0,	    0,	    0,		0xFFFF, WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(31,	REG_DO_2_BIT_PROTECTION,0,	    0,	    0,		0xFFFF, WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(32,	REG_CUR_SHUNT_MV,       0,	    60,	    0,		0xFFFF, WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(33,	REG_CUR_SHUNT_A,        0,	    5000,   0,		0xFFFF, WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(34,	REG_CUR_R_1_HOHM_OA_1,  0,	    0,	    0,		0xFFFF,	WRITE_R | CB_WR)\
    X_BUF(35,	REG_CUR_R_2_HOHM_OA_1,  0,	    0,	    0,		0xFFFF,	WRITE_R | CB_WR)\
    X_BUF(36,	REG_CUR_R_1_HOHM_OA_2,  0,	    0,	    0,		0xFFFF,	WRITE_R | CB_WR)\
    X_BUF(37,	REG_CUR_R_2_HOHM_OA_2,  0,	    0,	    0,		0xFFFF,	WRITE_R | CB_WR)\
	X_BUF(50,	REG_RS_BAUD,	        0,		1,		0,		0x03,	WRITE_R | CB_WR | LIM_MASK)\
	X_BUF(51,	REG_RS_DELAY,           0,	    5,		0,      100,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(52,	REG_RS_ADDR,            0,		1,		1,		0xFA,	WRITE_R | CB_WR | LIM_UNSIGN)\
	X_BUF(53,	REG_RS_PARITY_STOP, 	0,	    0,	    0,		0x03,	WRITE_R | CB_WR | LIM_UNSIGN)\
    X_BUF(97,	REG_GOTRACK,      	    0,	    0,	    0,		0,	    WRITE_R)\
	X_BUF(98,	REG_STARTRACK,      	0,	    0,	    0,		0,	    WRITE_R)\
	X_BUF(99,	REG_COUNTRACK,			0,	    0,      0,      0,      READ_R)\
	X_BUF(TRACK_VOLT_REG_START + TRACK_VOLT_REG_NUM,	Reg_End,				0,	    0,      0,      0,      READ_R)\


#endif /* MODBUS_X_H_INCLUDED */
