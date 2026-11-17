FIELDDEF(type);  // 0
FIELDDEF(size);  // 1
FIELDDEF(reserved_1);  // 2
FIELDDEF(reserved_2);  // 3
FIELDDEF(reserved_3);  // 4
FIELDDEF(focus_ft_high);  // 5
FIELDDEF(focus_ft_low);  // 6
FIELDDEF(focus_in);  // 7
FIELDDEF(iris_high);  // 8
FIELDDEF(iris_low);  // 9
SHORTFIELDDEF(focus_exp, 4);  // 10 - high
SHORTFIELDDEF(focus_sig_high, 4);  // 10 - low
FIELDDEF(focus_sig_low);  // 11
SHORTFIELDDEF(zoom_exp, 4);  // 12 - high
SHORTFIELDDEF(zoom_sig_high, 4);  // 12 - low
FIELDDEF(zoom_sig_low);  // 13
MULTIFIELDDEF(lens_model_name, 30);  // 14 - 43
FIELDDEF(master_gain_high);  // 44
FIELDDEF(master_gain_low);  // 45
FIELDDEF(shutter_speed_high);  // 46
FIELDDEF(shutter_speed_low);  // 47
SHORTFIELDDEF(reserved_4, 4);  // 48 - high
SHORTFIELDDEF(shutter_mode, 4);  // 48 - low
FIELDDEF(shutter_speed_decimal);  // 49
FIELDDEF(gamma_mode);  // 50
FIELDDEF(reserved_5);  // 51
SHORTFIELDDEF(atw_wb, 2);  // 52 - top 2 bits
SHORTFIELDDEF(under_over, 2);  // 52 - next 2 bits
SHORTFIELDDEF(color_temp_high, 4);  // 52 - bottom nibble
FIELDDEF(color_temp_low);  // 53
FIELDDEF(nd_filter);  // 54
FIELDDEF(cc_filter);  // 55
SHORTFIELDDEF(iris_info, 2);  // 56
SHORTFIELDDEF(focus_info, 2);  // 56
SHORTFIELDDEF(zoom_info, 2);  // 56
SHORTFIELDDEF(model_info, 2);  // 56
SHORTFIELDDEF(agc_enabled, 1);  // 57
SHORTFIELDDEF(gain_mode, 1);  // 57
SHORTFIELDDEF(iA_zoom_info, 2);  // 57
SHORTFIELDDEF(rb_gain_info, 1);  // 57
SHORTFIELDDEF(iris_type, 1);  // 57
SHORTFIELDDEF(iZoom_sm, 1);  // 57
SHORTFIELDDEF(nd_disp_type, 1);  // 57
SHORTFIELDDEF(vfr_mode, 4);  // 58 - high
SHORTFIELDDEF(vfr_frame_rate_high, 4);  // 58 - low
FIELDDEF(vfr_frame_rate_low);  // 59
SHORTFIELDDEF(iA_zoom_exp, 4);  // 60 - high
SHORTFIELDDEF(iA_zoom_high, 4);  // 60 - low
FIELDDEF(iA_zoom_low);  // 61
SHORTFIELDDEF(iso_select, 4);  // 62
SHORTFIELDDEF(e1_gain_mode, 1);  // 62
SHORTFIELDDEF(awb_color_temp, 1);  // 62
SHORTFIELDDEF(color_temp_mag, 1);  // 62
SHORTFIELDDEF(awb_channel, 1);  // 62
FIELDDEF(color_temp_GMg_high);  // 63
FIELDDEF(color_temp_GMg_low);  // 64
