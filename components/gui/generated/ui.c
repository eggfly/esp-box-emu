// SquareLine LVGL GENERATED FILE
// EDITOR VERSION: SquareLine Studio 1.1.1
// LVGL VERSION: 8.3.3
// PROJECT: emu

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////
lv_obj_t *ui_romscreen;
lv_obj_t *ui_header;
void ui_event_settingsbutton( lv_event_t * e);
lv_obj_t *ui_settingsbutton;
lv_obj_t *ui_Screen1_Label2;
lv_obj_t *ui_Screen1_Label1;
lv_obj_t *ui_playbutton;
lv_obj_t *ui_Screen1_Label3;
lv_obj_t *ui_rompanel;
lv_obj_t *ui_boxartpanel;
lv_obj_t *ui_boxart;
lv_obj_t *ui_settingsscreen;
lv_obj_t *ui_header1;
void ui_event_closebutton( lv_event_t * e);
lv_obj_t *ui_closebutton;
lv_obj_t *ui_Screen1_Label4;
lv_obj_t *ui_Screen1_Label5;
lv_obj_t *ui_settingspanel;
lv_obj_t *ui_volumepanel;
lv_obj_t *ui_volumebar;
lv_obj_t *ui_mutebutton;
lv_obj_t *ui_settingsscreen_Label1;
lv_obj_t *ui_volumedownbutton;
lv_obj_t *ui_settingsscreen_Label2;
lv_obj_t *ui_volumeupbutton;
lv_obj_t *ui_settingsscreen_Label3;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
    #error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_settingsbutton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_screen_change( ui_settingsscreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0);
}
}
void ui_event_closebutton( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_CLICKED) {
      _ui_screen_change( ui_romscreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0);
}
}

///////////////////// SCREENS ////////////////////
void ui_romscreen_screen_init(void)
{
ui_romscreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_romscreen, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_header = lv_obj_create(ui_romscreen);
lv_obj_set_height( ui_header, 75);
lv_obj_set_width( ui_header, lv_pct(100));
lv_obj_set_align( ui_header, LV_ALIGN_TOP_MID );
lv_obj_clear_flag( ui_header, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_settingsbutton = lv_btn_create(ui_header);
lv_obj_set_width( ui_settingsbutton, 48);
lv_obj_set_height( ui_settingsbutton, 48);
lv_obj_set_align( ui_settingsbutton, LV_ALIGN_LEFT_MID );
lv_obj_add_flag( ui_settingsbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_settingsbutton, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Screen1_Label2 = lv_label_create(ui_settingsbutton);
lv_obj_set_width( ui_Screen1_Label2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Screen1_Label2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Screen1_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Screen1_Label2,LV_SYMBOL_SETTINGS);

ui_Screen1_Label1 = lv_label_create(ui_header);
lv_obj_set_width( ui_Screen1_Label1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Screen1_Label1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Screen1_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Screen1_Label1,"Select Rom");

ui_playbutton = lv_btn_create(ui_header);
lv_obj_set_width( ui_playbutton, 48);
lv_obj_set_height( ui_playbutton, 48);
lv_obj_set_align( ui_playbutton, LV_ALIGN_RIGHT_MID );
lv_obj_add_state( ui_playbutton, LV_STATE_CHECKED );     /// States
lv_obj_add_flag( ui_playbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_playbutton, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Screen1_Label3 = lv_label_create(ui_playbutton);
lv_obj_set_width( ui_Screen1_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Screen1_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Screen1_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Screen1_Label3,LV_SYMBOL_PLAY);

ui_rompanel = lv_obj_create(ui_romscreen);
lv_obj_set_width( ui_rompanel, 220);
lv_obj_set_height( ui_rompanel, 165);
lv_obj_set_align( ui_rompanel, LV_ALIGN_BOTTOM_LEFT );
lv_obj_add_flag( ui_rompanel, LV_OBJ_FLAG_SCROLL_ON_FOCUS | LV_OBJ_FLAG_SCROLL_ONE );   /// Flags
lv_obj_set_scroll_dir(ui_rompanel, LV_DIR_VER);

ui_boxartpanel = lv_obj_create(ui_romscreen);
lv_obj_set_width( ui_boxartpanel, 100);
lv_obj_set_height( ui_boxartpanel, 165);
lv_obj_set_align( ui_boxartpanel, LV_ALIGN_BOTTOM_RIGHT );
lv_obj_clear_flag( ui_boxartpanel, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_boxart = lv_img_create(ui_boxartpanel);
lv_obj_set_width( ui_boxart, 100);
lv_obj_set_height( ui_boxart, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_boxart, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_boxart, LV_OBJ_FLAG_ADV_HITTEST );   /// Flags
lv_obj_clear_flag( ui_boxart, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

lv_obj_add_event_cb(ui_settingsbutton, ui_event_settingsbutton, LV_EVENT_ALL, NULL);

}
void ui_settingsscreen_screen_init(void)
{
ui_settingsscreen = lv_obj_create(NULL);
lv_obj_clear_flag( ui_settingsscreen, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_header1 = lv_obj_create(ui_settingsscreen);
lv_obj_set_height( ui_header1, 75);
lv_obj_set_width( ui_header1, lv_pct(100));
lv_obj_set_align( ui_header1, LV_ALIGN_TOP_MID );
lv_obj_clear_flag( ui_header1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_closebutton = lv_btn_create(ui_header1);
lv_obj_set_width( ui_closebutton, 48);
lv_obj_set_height( ui_closebutton, 48);
lv_obj_set_align( ui_closebutton, LV_ALIGN_LEFT_MID );
lv_obj_add_flag( ui_closebutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_closebutton, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Screen1_Label4 = lv_label_create(ui_closebutton);
lv_obj_set_width( ui_Screen1_Label4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Screen1_Label4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Screen1_Label4, LV_ALIGN_CENTER );
lv_label_set_text(ui_Screen1_Label4,LV_SYMBOL_CLOSE);

ui_Screen1_Label5 = lv_label_create(ui_header1);
lv_obj_set_width( ui_Screen1_Label5, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Screen1_Label5, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Screen1_Label5, LV_ALIGN_CENTER );
lv_label_set_text(ui_Screen1_Label5,"Settings");

ui_settingspanel = lv_obj_create(ui_settingsscreen);
lv_obj_set_height( ui_settingspanel, 165);
lv_obj_set_width( ui_settingspanel, lv_pct(100));
lv_obj_set_align( ui_settingspanel, LV_ALIGN_BOTTOM_MID );
lv_obj_add_flag( ui_settingspanel, LV_OBJ_FLAG_SCROLL_ON_FOCUS | LV_OBJ_FLAG_SCROLL_ONE );   /// Flags

ui_volumepanel = lv_obj_create(ui_settingsscreen);
lv_obj_set_height( ui_volumepanel, 50);
lv_obj_set_width( ui_volumepanel, lv_pct(100));
lv_obj_set_align( ui_volumepanel, LV_ALIGN_CENTER );
lv_obj_clear_flag( ui_volumepanel, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_volumebar = lv_bar_create(ui_volumepanel);
lv_bar_set_value(ui_volumebar,25,LV_ANIM_OFF);
lv_obj_set_width( ui_volumebar, 150);
lv_obj_set_height( ui_volumebar, 10);
lv_obj_set_x( ui_volumebar, 20 );
lv_obj_set_y( ui_volumebar, 0 );
lv_obj_set_align( ui_volumebar, LV_ALIGN_CENTER );

ui_mutebutton = lv_btn_create(ui_volumepanel);
lv_obj_set_width( ui_mutebutton, 32);
lv_obj_set_height( ui_mutebutton, 32);
lv_obj_set_align( ui_mutebutton, LV_ALIGN_LEFT_MID );
lv_obj_add_flag( ui_mutebutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_mutebutton, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_settingsscreen_Label1 = lv_label_create(ui_mutebutton);
lv_obj_set_width( ui_settingsscreen_Label1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_settingsscreen_Label1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_settingsscreen_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_settingsscreen_Label1,LV_SYMBOL_MUTE);

ui_volumedownbutton = lv_btn_create(ui_volumepanel);
lv_obj_set_width( ui_volumedownbutton, 32);
lv_obj_set_height( ui_volumedownbutton, 32);
lv_obj_set_x( ui_volumedownbutton, 52 );
lv_obj_set_y( ui_volumedownbutton, 0 );
lv_obj_set_align( ui_volumedownbutton, LV_ALIGN_LEFT_MID );
lv_obj_add_flag( ui_volumedownbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_volumedownbutton, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_settingsscreen_Label2 = lv_label_create(ui_volumedownbutton);
lv_obj_set_width( ui_settingsscreen_Label2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_settingsscreen_Label2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_settingsscreen_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_settingsscreen_Label2,LV_SYMBOL_VOLUME_MID);

ui_volumeupbutton = lv_btn_create(ui_volumepanel);
lv_obj_set_width( ui_volumeupbutton, 32);
lv_obj_set_height( ui_volumeupbutton, 32);
lv_obj_set_x( ui_volumeupbutton, -10 );
lv_obj_set_y( ui_volumeupbutton, 0 );
lv_obj_set_align( ui_volumeupbutton, LV_ALIGN_RIGHT_MID );
lv_obj_add_flag( ui_volumeupbutton, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_volumeupbutton, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_settingsscreen_Label3 = lv_label_create(ui_volumeupbutton);
lv_obj_set_width( ui_settingsscreen_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_settingsscreen_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_settingsscreen_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_settingsscreen_Label3,LV_SYMBOL_VOLUME_MAX);

lv_obj_add_event_cb(ui_closebutton, ui_event_closebutton, LV_EVENT_ALL, NULL);

}

void ui_init( void )
{
lv_disp_t *dispp = lv_disp_get_default();
lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
lv_disp_set_theme(dispp, theme);
ui_romscreen_screen_init();
ui_settingsscreen_screen_init();
lv_disp_load_scr( ui_romscreen);
}
