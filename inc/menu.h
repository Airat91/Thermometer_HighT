#include "stdint.h"

#ifndef MENU_H
#define MENU_H 1

/*========== DEFINES ==========*/

#define MAKE_MENU(Name, Next, Previous, Parent, Child, Child_num, Page, Text) \
    extern menuItem Next;     \
    extern menuItem Previous; \
    extern menuItem Parent;   \
    extern menuItem Child;  \
    menuItem Name = {(void*)&Next, (void*)&Previous, (void*)&Parent, (void*)&Child, (uint16_t)Child_num, (uint16_t)Page, { Text }}

#define PREVIOUS   ((menuItem*)pgm_read_word(&selectedMenuItem->Previous))
#define NEXT       ((menuItem*)pgm_read_word(&selectedMenuItem->Next))
#define PARENT     ((menuItem*)pgm_read_word(&selectedMenuItem->Parent))
#define CHILD      ((menuItem*)pgm_read_word(&selectedMenuItem->Child))
#define SELECT		(pgm_read_byte(&selectedMenuItem->Select))
#define NULL_ENTRY  Null_Menu
#define EDITED_VAL  edit_value

/*========== TYPEDEFS ==========*/

typedef enum {
    MAIN_PAGE = 0,
    MAIN_MENU,
    COMMON_INFO,
    DCTS_VER,
    V_PWR,
    MEAS_CHANNELS,
    MEAS_CH_0,
    MEAS_CH_1,
    MEAS_CH_2,
    MEAS_CH_3,
    MEAS_CH_4,
    MEAS_CH_5,
    TMPR_CALIB,
    TMPR_COEF_A,
    TMPR_COEF_B,
    CONNECTION,
    MDB_ADDR,
    MDB_BITRATE,
    MDB_RECIEVED_CNT,
    MDB_SEND_CNT,
    MDB_OVERRUN_ERR,
    MDB_PARITY_ERR,
    MDB_FRAME_ERR,
    MDB_NOISE_ERR,
    DISPLAY,
    LIGHT_LVL,
    SKIN,
    AM2302_PIN,
    PIN_CONFIG,
    SAVE_CHANGES,
    SAVING,
    TIME,
    TIME_HOUR,
    TIME_MIN,
    TIME_SEC,
    DATE,
    DATE_DAY,
    DATE_MONTH,
    DATE_YEAR,
    EDIT,
} menu_page_t;

typedef struct {
    void            *Next;
    void            *Previous;
    void            *Parent;
    void            *Child;
    uint16_t        Child_num;
    menu_page_t     Page;
    const char      Text[20];
} menuItem;



/*========= GLOBAL VARIABLES ==========*/

extern menuItem main_page;
extern menuItem save_changes;
extern menuItem* selectedMenuItem;
extern menuItem edit_value;

/*========== FUNCTION PROTOTYPES ==========*/

void menu_init (void);

void menuChange(menuItem* NewMenu);

#endif // MENU_H
