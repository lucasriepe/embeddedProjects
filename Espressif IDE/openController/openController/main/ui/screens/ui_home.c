// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_home_screen_init(void)
{
ui_home = lv_obj_create(NULL);
lv_obj_clear_flag( ui_home, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_btnSwitchBlue = lv_btn_create(ui_home);
lv_obj_set_width( ui_btnSwitchBlue, 100);
lv_obj_set_height( ui_btnSwitchBlue, 100);
lv_obj_set_x( ui_btnSwitchBlue, -300 );
lv_obj_set_y( ui_btnSwitchBlue, -150 );
lv_obj_set_align( ui_btnSwitchBlue, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_btnSwitchBlue, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_btnSwitchBlue, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_labelBTNSwitch = lv_label_create(ui_btnSwitchBlue);
lv_obj_set_width( ui_labelBTNSwitch, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_labelBTNSwitch, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_labelBTNSwitch, LV_ALIGN_CENTER );
lv_label_set_text(ui_labelBTNSwitch,"SWITCH");
lv_obj_set_style_text_font(ui_labelBTNSwitch, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_funcBTNBlue1 = lv_btn_create(ui_home);
lv_obj_set_width( ui_funcBTNBlue1, 100);
lv_obj_set_height( ui_funcBTNBlue1, 100);
lv_obj_set_x( ui_funcBTNBlue1, -150 );
lv_obj_set_y( ui_funcBTNBlue1, -150 );
lv_obj_set_align( ui_funcBTNBlue1, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_funcBTNBlue1, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_funcBTNBlue1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_labelBTNblue1 = lv_label_create(ui_funcBTNBlue1);
lv_obj_set_width( ui_labelBTNblue1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_labelBTNblue1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_labelBTNblue1, LV_ALIGN_CENTER );
lv_label_set_text(ui_labelBTNblue1,"FUNC 1");
lv_obj_set_style_text_font(ui_labelBTNblue1, &lv_font_montserrat_20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Slider1 = lv_slider_create(ui_home);
lv_slider_set_value( ui_Slider1, 0, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_Slider1)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_Slider1, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_Slider1, 700);
lv_obj_set_height( ui_Slider1, 20);
lv_obj_set_x( ui_Slider1, 0 );
lv_obj_set_y( ui_Slider1, 200 );
lv_obj_set_align( ui_Slider1, LV_ALIGN_CENTER );


ui_labelSliderBlue = lv_label_create(ui_home);
lv_obj_set_width( ui_labelSliderBlue, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_labelSliderBlue, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_labelSliderBlue, 0 );
lv_obj_set_y( ui_labelSliderBlue, 170 );
lv_obj_set_align( ui_labelSliderBlue, LV_ALIGN_CENTER );
lv_label_set_text(ui_labelSliderBlue,"0");

lv_obj_add_event_cb(ui_btnSwitchBlue, ui_event_btnSwitchBlue, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_funcBTNBlue1, ui_event_funcBTNBlue1, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Slider1, ui_event_Slider1, LV_EVENT_ALL, NULL);
uic_labelSliderBlue = ui_labelSliderBlue;

}
