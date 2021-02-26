#include "menu.h"
#include "main.h"
#include "buttons.h"
#include "dcts.h"
#include "dcts_config.h"
#include "string.h"

/**
  * @defgroup menu
  * @brief work with menu
  */


#define NULL_ENTRY Null_Menu
static menuItem Null_Menu = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = 0,
    .Text = {0},
};

menuItem* selectedMenuItem;
static menuItem* menuStack[10];
static volatile uint8_t menuStackTop;

//                  NAME           NEXT            PREV            PARENT          CHILD        GHILD_NUM   PAGE                    TEXT
MAKE_MENU       (main_page,     main_menu,      main_menu,      NULL_ENTRY,     NULL_ENTRY,     0,          MAIN_PAGE,          "DISPLAY");
MAKE_MENU       (main_menu,     main_page,      main_page,      NULL_ENTRY,     common_info,    4,          MAIN_MENU,          "MENU");
  MAKE_MENU     (common_info,   meas_channels,  date,           main_menu,      dcts_ver,       2,          COMMON_INFO,        "INFO");
    MAKE_MENU   (dcts_ver,      v_pwr,          v_pwr,          common_info,    NULL_ENTRY,     0,          DCTS_VER,           "dcts");
    MAKE_MENU   (v_pwr,         dcts_ver,       dcts_ver,       common_info,    NULL_ENTRY,     0,          V_PWR,              "u_in");
  MAKE_MENU     (meas_channels, connection,     common_info,    main_menu,      meas_ch_0,      6,          MEAS_CHANNELS,      "CHANNELS");
    MAKE_MENU   (meas_ch_0,     meas_ch_1,      meas_ch_5,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_0,          "tpr");
    MAKE_MENU   (meas_ch_1,     meas_ch_2,      meas_ch_0,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_1,          "tpr_adc");
    MAKE_MENU   (meas_ch_2,     meas_ch_3,      meas_ch_1,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_2,          "tpr_ult");
    MAKE_MENU   (meas_ch_3,     meas_ch_4,      meas_ch_2,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_3,          "ref_adc");
    MAKE_MENU   (meas_ch_4,     meas_ch_5,      meas_ch_3,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_4,          "data_t");
    MAKE_MENU   (meas_ch_5,     meas_ch_0,      meas_ch_4,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_5,          "data_h");
  MAKE_MENU     (connection,    display,        meas_channels,  main_menu,      mdb_addr,       8,          CONNECTION,         "CONNECT");
    MAKE_MENU   (mdb_addr,      bitrate,        noise_err,      connection,     NULL_ENTRY,     0,          MDB_ADDR,           "Addr");
    MAKE_MENU   (bitrate,       recieved_cnt,   mdb_addr,       connection,     NULL_ENTRY,     0,          MDB_BITRATE,        "Bitrate");
    MAKE_MENU   (recieved_cnt,  send_cnt,       bitrate,        connection,     NULL_ENTRY,     0,          MDB_RECIEVED_CNT,   "recv_cnt");
    MAKE_MENU   (send_cnt,      overrun_err,    recieved_cnt,   connection,     NULL_ENTRY,     0,          MDB_SEND_CNT,       "send_cnt");
    MAKE_MENU   (overrun_err,   parity_err,     send_cnt,       connection,     NULL_ENTRY,     0,          MDB_OVERRUN_ERR,    "ourr_err");
    MAKE_MENU   (parity_err,    frame_err,      overrun_err,    connection,     NULL_ENTRY,     0,          MDB_PARITY_ERR,     "par_err");
    MAKE_MENU   (frame_err,     noise_err,      parity_err,     connection,     NULL_ENTRY,     0,          MDB_FRAME_ERR,      "fr_err");
    MAKE_MENU   (noise_err,     mdb_addr,       frame_err,      connection,     NULL_ENTRY,     0,          MDB_NOISE_ERR,      "n_err");
  MAKE_MENU     (display,       time,           connection,     main_menu,      light_lvl,      1,          DISPLAY,            "DISPLAY");
    MAKE_MENU   (light_lvl,     NULL_ENTRY,     NULL_ENTRY,     display,        NULL_ENTRY,     0,          LIGHT_LVL,          "Light");
  MAKE_MENU     (time,          date,           display,        main_menu,      time_hour,      3,          TIME,               "SET T");
    MAKE_MENU   (time_hour,     time_min,       time_sec,       time,           NULL_ENTRY,     0,          TIME_HOUR,          "hour");
    MAKE_MENU   (time_min,      time_sec,       time_hour,      time,           NULL_ENTRY,     0,          TIME_MIN,           "minute");
    MAKE_MENU   (time_sec,      time_hour,      time_min,       time,           NULL_ENTRY,     0,          TIME_SEC,           "sec");
  MAKE_MENU     (date,          common_info,    time,           main_menu,      date_day,       3,          DATE,               "SET DATE");
    MAKE_MENU   (date_day,      date_month,     date_year,      date,           NULL_ENTRY,     0,          DATE_DAY,           "day");
    MAKE_MENU   (date_month,    date_year,      date_day,       date,           NULL_ENTRY,     0,          DATE_MONTH,         "month");
    MAKE_MENU   (date_year,     date_day,       date_month,     date,           NULL_ENTRY,     0,          DATE_YEAR,          "year");

MAKE_MENU       (save_changes,  NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     NULL_ENTRY,     0,          SAVE_CHANGES,       "SAVE");


/*========== FUNCTIONS ==========*/

void menu_init (void){

    selectedMenuItem = &main_page;
}

void menuChange(menuItem* NewMenu){
    if (NewMenu != &NULL_ENTRY){
        selectedMenuItem = NewMenu;
    }
}
