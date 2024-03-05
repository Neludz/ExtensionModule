#ifndef OPTIC_CURRENT_DATA_H_INCLUDED
#define OPTIC_CURRENT_DATA_H_INCLUDED

#define OPTIC_BUFFER_SIZE           15 //power of 2

#define TRACK_POINT_NUM             500
#define TRACK_CUR_REG_START         100
#define TRACK_CUR_REG_NUM           TRACK_POINT_NUM
#define TRACK_VOLT_REG_START        600
#define TRACK_VOLT_REG_NUM          (TRACK_POINT_NUM / 2)   //8 bit data in 16 bit reg

#define TRECK_PRE_BUF           63 //power of 2


typedef enum
{
    OPTIC_STATE_IDLE,			// Ready to get a frame from Master
    OPTIC_STATE_RCVE,			// Frame is being received
    OPTIC_STATE_WAIT,	        // Frame is wait to parse
    OPTIC_STATE_PARSE,			// Frame is being parsed (may take some time)
} OpticState_t;

typedef enum
{
    OPTIC_EVENTS_OK = 0,
    OPTIC_EVENTS_FRAME_ERROR,
    OPTIC_EVENTS_UART_FIN,
} OpticEvents_t;

typedef struct
{
    uint32_t        index;
    OpticState_t    state;
    OpticEvents_t   events;
    uint8_t         buf[OPTIC_BUFFER_SIZE];
} OpticStruct_t;




typedef enum
{
    PROTECTION_ALARM_NO_ALARM = 0,
    PROTECTION_ALARM_CUR_MAX_P = 1,
    PROTECTION_ALARM_CUR_MAX_N,
    PROTECTION_ALARM_CUR_OL_P,
    PROTECTION_ALARM_CUR_OL_N,
    PROTECTION_ALARM_CUR_DIDT_P,
    PROTECTION_ALARM_CUR_DIDT_N,
} Protection_Alarm_t;

typedef enum
{
    PROTECTION_EVENT_NO_EVENTS = 0,
    PROTECTION_EVENT_WAIT_RESET,
    PROTECTION_EVENT_STARTRECK,
} Protection_Events_t;

#define PROTECTION_NO_ALARM             0
#define PROTECTION_ANY_ALARM            1
#define PROTECTION_CUR_MAX_P            (1 << VALUE_CUR_MAX_P)
#define PROTECTION_CUR_MAX_N            (1 << VALUE_CUR_MAX_N)
#define PROTECTION_CUR_OL_P             (1 << VALUE_CUR_OL_P)
#define PROTECTION_CUR_OL_N             (1 << VALUE_CUR_OL_N)
#define PROTECTION_CUR_DIDT_P           (1 << VALUE_CUR_DIDT_P)
#define PROTECTION_CUR_DIDT_N           (1 << VALUE_CUR_DIDT_N)

typedef struct
{
    uint32_t    current_alarm_bit;
    Protection_Alarm_t  trig_alarm;
    uint32_t    mb_save_index;
    Protection_Events_t event;
    int16_t current_offset_discrete;
    int16_t protection_mvx10_cur_max_p;
    int16_t protection_mvx10_cur_max_n;
    int16_t protection_mvx10_cur_ol_p;
    uint16_t protection_value_time_ms_ol_p;
    int16_t protection_mvx10_cur_ol_n;
    uint16_t protection_value_time_ms_ol_n;
    int16_t protection_mvx10_cur_didt_p;
    uint16_t protection_value_time_ms_didt_p;
    int16_t protection_mvx10_cur_didt_n;
    uint16_t protection_value_time_ms_didt_n;
    uint16_t startreck_write_pos;
    int16_t treck_pre_buf[TRECK_PRE_BUF];
} ProtectionStruct_t;


void oc_optic_current_init(void);


#endif /* OPTIC_CURRENT_DATA_H_INCLUDED */
