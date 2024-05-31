#ifndef OPTIC_CURRENT_DATA_H_INCLUDED
#define OPTIC_CURRENT_DATA_H_INCLUDED

#define OPTIC_BUFFER_SIZE               15 //power of 2

#define OPTIC_LAN_ERROR_ATTEMPT         20

#define TRECK_PRE_BUF                   127 //power of 2
#define TRACK_POINT_NUM                 512
#define TRACK_CUR_REG_START             100
#define TRACK_CUR_REG_MAIN              TRACK_CUR_REG_START + TRECK_PRE_BUF + 1
#define TRACK_VOLT_REG_START            612
#define TRACK_VOLT_REG_MAIN             TRACK_VOLT_REG_START + TRECK_PRE_BUF + 1
#define TRACK_MAIN_SIZE                 TRACK_POINT_NUM-TRECK_PRE_BUF-1

#define PROTECTION_RESET_VALUE          0xC01	//3073
#define CALIBRATION_VALUE               0xD01	//3329

// Device mode
#define  ALARM_MODE_WAIT_RESET          0
#define  ALARM_MODE_AUTO_RESET          1

#define MIN_ALARM_TIME_MS               2
#define MIN_ALARM_RESET_AMOUNT          10

#define PR_ALARM_CUR_MAX_P_POS          0
#define PR_ALARM_CUR_MAX_N_POS          1
#define PR_ALARM_CUR_OL_P_POS           2
#define PR_ALARM_CUR_OL_N_POS           3
#define PR_ALARM_CUR_DIDT_P_POS         4
#define PR_ALARM_CUR_DIDT_N_POS         5
#define PR_ALARM_VOLT_MIN_POS           6
#define PR_ALARM_VOLT_MAX_POS           7
#define PR_ALARM_LAN_ERROR_POS          8
#define PR_ALARM_MANUAL_POS             9

#define PR_ALARM_CUR_MAX_P_MSK          (0x1UL << PR_ALARM_CUR_MAX_P_POS)
#define PR_ALARM_CUR_MAX_N_MSK          (0x1UL << PR_ALARM_CUR_MAX_N_POS)
#define PR_ALARM_CUR_OL_P_MSK           (0x1UL << PR_ALARM_CUR_OL_P_POS)
#define PR_ALARM_CUR_OL_N_MSK           (0x1UL << PR_ALARM_CUR_OL_N_POS)
#define PR_ALARM_CUR_DIDT_P_MSK         (0x1UL << PR_ALARM_CUR_DIDT_P_POS)
#define PR_ALARM_CUR_DIDT_N_MSK         (0x1UL << PR_ALARM_CUR_DIDT_N_POS)
#define PR_ALARM_VOLT_MIN_MSK           (0x1UL << PR_ALARM_VOLT_MIN_POS)
#define PR_ALARM_VOLT_MAX_MSK           (0x1UL << PR_ALARM_VOLT_MAX_POS)
#define PR_ALARM_LAN_ERROR_MSK          (0x1UL << PR_ALARM_LAN_ERROR_POS)
#define PR_ALARM_MANUAL_MSK             (0x1UL << PR_ALARM_MANUAL_POS)

#define PR_ALARM_ALL_MSK                0x3FF
#define PR_ALARM_STARTRECK_MSK          0x2FF

#define ALARM_NUMBER_TIMER              8
#define ALARM_TIMERS_LIST               {PR_ALARM_CUR_MAX_P_MSK,\
                                        PR_ALARM_CUR_MAX_N_MSK,\
                                        PR_ALARM_CUR_OL_P_MSK,\
                                        PR_ALARM_CUR_OL_N_MSK,\
                                        PR_ALARM_CUR_DIDT_P_MSK,\
                                        PR_ALARM_CUR_DIDT_N_MSK,\
                                        PR_ALARM_VOLT_MIN_MSK,\
                                        PR_ALARM_VOLT_MAX_MSK}


#define PR_ALARM_NO_ALARM               0
#define PR_ALARM_CUR_MAX_P              PR_ALARM_CUR_MAX_P_MSK
#define PR_ALARM_CUR_MAX_N              PR_ALARM_CUR_MAX_N_MSK
#define PR_ALARM_CUR_OL_P               PR_ALARM_CUR_OL_P_MSK
#define PR_ALARM_CUR_OL_N               PR_ALARM_CUR_OL_N_MSK
#define PR_ALARM_CUR_DIDT_P             PR_ALARM_CUR_DIDT_P_MSK
#define PR_ALARM_CUR_DIDT_N             PR_ALARM_CUR_DIDT_N_MSK
#define PR_ALARM_VOLT_MIN               PR_ALARM_VOLT_MIN_MSK)
#define PR_ALARM_VOLT_MAX               PR_ALARM_VOLT_MAX_MSK
#define PR_ALARM_LAN_ERROR              PR_ALARM_LAN_ERROR_MSK
#define PR_ALARM_MANUAL                 PR_ALARM_MANUAL_MSK


typedef enum
{
    OPTIC_STATE_IDLE,			// Ready to get a frame from Master
    OPTIC_STATE_RCVE,			// Frame is being received
    OPTIC_STATE_WAIT,	        // Frame is wait to parse
    OPTIC_STATE_PARSE,			// Frame is being parsed (may take some time)
} OpticState_t;

typedef enum
{
    OPTIC_EVENTS_NONE = 0,
    OPTIC_EVENTS_FRAME_ERROR,
    OPTIC_EVENTS_UART_FIN,
} OpticEvents_t;

typedef struct
{
    uint32_t        index;
    OpticState_t    state;
    OpticEvents_t   events;
    uint8_t         buf[OPTIC_BUFFER_SIZE+1];
} OpticStruct_t;

typedef enum
{
    OPT_RET_OK = 0,
    OPT_RET_ERROR,
} OptError_t;

typedef enum
{
    PROTECTION_EVENT_NO_EVENTS = 0,
    PROTECTION_EVENT_WAIT_RESET,
    PROTECTION_EVENT_WRITETRECK,
} Protection_Events_t;

typedef struct
{
    uint32_t lan_error_count;
    uint32_t lan_error;
    uint16_t wait_reset_flag;
    uint16_t startreck;
    uint16_t trig_alarm;
    uint16_t active_timer_alarm;
    uint32_t mb_save_index;
    Protection_Events_t event;
    uint16_t startreck_write_pos;
    int16_t treck_pre_cur_buf[TRECK_PRE_BUF+1];
    int16_t treck_pre_volt_buf[TRECK_PRE_BUF+1];
    uint8_t reset_count[ALARM_NUMBER_TIMER];
} ProtectionStruct_t;

void oc_optic_current_init(void);
void shunt_calibration();

#endif /* OPTIC_CURRENT_DATA_H_INCLUDED */
