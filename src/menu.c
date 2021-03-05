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


static menuItem Null_Menu = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = 0,
    .Text = {0},
};
menuItem edit_value = {
    .Next = (void*)0,
    .Previous = (void*)0,
    .Parent = (void*)0,
    .Child = (void*)0,
    .Child_num = 0,
    .Page = EDIT,
    .Text = {0},
};

menuItem* selectedMenuItem;

//                  NAME           NEXT            PREV            PARENT          CHILD        GHILD_NUM   PAGE                    TEXT
MAKE_MENU       (main_page,     main_menu,      main_menu,      NULL_ENTRY,     main_menu,      0,          MAIN_PAGE,          "DISPLAY");
MAKE_MENU       (main_menu,     main_page,      main_page,      main_page,      common_info,    4,          MAIN_MENU,          "MENU");
  MAKE_MENU     (common_info,   meas_channels,  save_changes,   main_menu,      dcts_ver,       2,          COMMON_INFO,        "INFO");
    MAKE_MENU   (dcts_ver,      v_pwr,          v_pwr,          common_info,    NULL_ENTRY,     0,          DCTS_VER,           "dcts");
    MAKE_MENU   (v_pwr,         dcts_ver,       dcts_ver,       common_info,    NULL_ENTRY,     0,          V_PWR,              "u_in");
  MAKE_MENU     (meas_channels, tmpr_calib,     common_info,    main_menu,      meas_ch_0,      6,          MEAS_CHANNELS,      "CHANNELS");
    MAKE_MENU   (meas_ch_0,     meas_ch_1,      meas_ch_5,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_0,          "tpr");
    MAKE_MENU   (meas_ch_1,     meas_ch_2,      meas_ch_0,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_1,          "tpr_adc");
    MAKE_MENU   (meas_ch_2,     meas_ch_3,      meas_ch_1,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_2,          "tpr_ult");
    MAKE_MENU   (meas_ch_3,     meas_ch_4,      meas_ch_2,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_3,          "ref_adc");
    MAKE_MENU   (meas_ch_4,     meas_ch_5,      meas_ch_3,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_4,          "data_t");
    MAKE_MENU   (meas_ch_5,     meas_ch_0,      meas_ch_4,      meas_channels,  NULL_ENTRY,     0,          MEAS_CH_5,          "data_h");
  MAKE_MENU     (tmpr_calib,    connection,     meas_channels,  main_menu,      tmpr_coef_a,    2,          TMPR_CALIB,         "T CALIB");
    MAKE_MENU   (tmpr_coef_a,   tmpr_coef_b,    tmpr_coef_b,    tmpr_calib,     EDITED_VAL,     0,          TMPR_COEF_A,        "coef_a");
    MAKE_MENU   (tmpr_coef_b,   tmpr_coef_a,    tmpr_coef_a,    tmpr_calib,     EDITED_VAL,     0,          TMPR_COEF_B,        "coef_b");
  MAKE_MENU     (connection,    display,        tmpr_calib,     main_menu,      mdb_addr,       8,          CONNECTION,         "CONNECT");
    MAKE_MENU   (mdb_addr,      bitrate,        noise_err,      connection,     EDITED_VAL,     0,          MDB_ADDR,           "Address");
    MAKE_MENU   (bitrate,       recieved_cnt,   mdb_addr,       connection,     EDITED_VAL,     0,          MDB_BITRATE,        "Bitrate");
    MAKE_MENU   (recieved_cnt,  send_cnt,       bitrate,        connection,     NULL_ENTRY,     0,          MDB_RECIEVED_CNT,   "recieue_cnt");
    MAKE_MENU   (send_cnt,      overrun_err,    recieved_cnt,   connection,     NULL_ENTRY,     0,          MDB_SEND_CNT,       "send_cnt");
    MAKE_MENU   (overrun_err,   parity_err,     send_cnt,       connection,     NULL_ENTRY,     0,          MDB_OVERRUN_ERR,    "ouerrun_err");
    MAKE_MENU   (parity_err,    frame_err,      overrun_err,    connection,     NULL_ENTRY,     0,          MDB_PARITY_ERR,     "paritet_err");
    MAKE_MENU   (frame_err,     noise_err,      parity_err,     connection,     NULL_ENTRY,     0,          MDB_FRAME_ERR,      "frame_err");
    MAKE_MENU   (noise_err,     mdb_addr,       frame_err,      connection,     NULL_ENTRY,     0,          MDB_NOISE_ERR,      "noise_err");
  MAKE_MENU     (display,       data_pin,       connection,     main_menu,      light_lvl,      2,          DISPLAY,            "DISPLAY");
    MAKE_MENU   (light_lvl,     skin_select,    skin_select,    display,        EDITED_VAL,     0,          LIGHT_LVL,          "Light");
    MAKE_MENU   (skin_select,   light_lvl,      light_lvl,      display,        EDITED_VAL,     0,          SKIN,               "Skin");
  MAKE_MENU     (data_pin,      time,           display,        main_menu,      pin_config,     1,          AM2302_PIN,         "DATA_PIN");
    MAKE_MENU   (pin_config,    NULL_ENTRY,     NULL_ENTRY,     data_pin,       EDITED_VAL,     0,          PIN_CONFIG,         "Config");
  MAKE_MENU     (time,          date,           data_pin,       main_menu,      time_hour,      3,          TIME,               "SET TIME");
    MAKE_MENU   (time_hour,     time_min,       time_sec,       time,           EDITED_VAL,     0,          TIME_HOUR,          "hour");
    MAKE_MENU   (time_min,      time_sec,       time_hour,      time,           EDITED_VAL,     0,          TIME_MIN,           "minute");
    MAKE_MENU   (time_sec,      time_hour,      time_min,       time,           EDITED_VAL,     0,          TIME_SEC,           "second");
  MAKE_MENU     (date,          save_changes,   time,           main_menu,      date_day,       3,          DATE,               "SET DATE");
    MAKE_MENU   (date_day,      date_month,     date_year,      date,           EDITED_VAL,     0,          DATE_DAY,           "day");
    MAKE_MENU   (date_month,    date_year,      date_day,       date,           EDITED_VAL,     0,          DATE_MONTH,         "month");
    MAKE_MENU   (date_year,     date_day,       date_month,     date,           EDITED_VAL,     0,          DATE_YEAR,          "year");
  MAKE_MENU     (save_changes,  common_info,    date,           main_menu,      saving,         1,          SAVE_CHANGES,       "SAUE");
    MAKE_MENU   (saving,        NULL_ENTRY,     NULL_ENTRY,     save_changes,   EDITED_VAL,     0,          SAVING,             "Sauing");


/*========== FUNCTIONS ==========*/

void menu_init (void){
    selectedMenuItem = &main_page;
}

void menuChange(menuItem* NewMenu){
    if ((NewMenu != &NULL_ENTRY)&&(NewMenu != &EDITED_VAL)){
        selectedMenuItem = NewMenu;
    }
}
