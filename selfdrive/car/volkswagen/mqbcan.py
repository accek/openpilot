MSG_STEERING = "HCA_01"
MSG_EPS = "LH_EPS_03"
MSG_LKA_HUD = "LDW_02"
MSG_ACC_BUTTONS = "GRA_ACC_01"
MSG_ACC_1 = "ACC_06"
MSG_ACC_2 = "ACC_07"
MSG_ACC_HUD_1 = "ACC_02"
MSG_ACC_HUD_2 = "ACC_04"
MSG_TSK = "TSK_06"


def create_steering_control(values, apply_steer, lkas_enabled):
  values.setdefault("EA_ACC_Wunschgeschwindigkeit", 327.36)
  values.update({
    "HCA_01_Status_HCA": 5 if lkas_enabled else 3,
    "HCA_01_LM_Offset": abs(apply_steer),
    "HCA_01_LM_OffSign": 1 if apply_steer < 0 else 0,
    "HCA_01_Vib_Freq": 18,
    "HCA_01_Sendestatus": 1 if lkas_enabled else 0,
  })
  return values


def create_eps_update(values, ea_simulated_torque):
  values.update({
    # Absolute driver torque input and sign, with EA inactivity mitigation
    "EPS_Lenkmoment": abs(ea_simulated_torque),
    "EPS_VZ_Lenkmoment": 1 if ea_simulated_torque < 0 else 0,
  })
  return values


def create_tsk_update(values, stock_values):
  acc_1 = stock_values.get(MSG_ACC_1)
  if acc_1:
    acc_status = acc_1["ACC_Status_ACC"]
    tsk_status = values["TSK_Status"]
    if (acc_status in (3, 4) and tsk_status == 2) or tsk_status == 6:
      values.update({
        "TSK_Status": acc_status,
        "TSK_zul_Regelabw": acc_1["ACC_zul_Regelabw_unten"],
      })
  return values


def create_lka_hud_control(values, mads_enabled, lat_active, hud_alert, hud_control):
  values.update({
    "LDW_Status_LED_gelb": 1 if mads_enabled else 0,
    "LDW_Status_LED_gruen": 1 if lat_active else 0,
    "LDW_Lernmodus_links": 3 if hud_control.leftLaneDepart else 1 + hud_control.leftLaneVisible,
    "LDW_Lernmodus_rechts": 3 if hud_control.rightLaneDepart else 1 + hud_control.rightLaneVisible,
  })
  if hud_alert > 0:
    values["LDW_Texte"] = hud_alert
  return values


def create_acc_buttons_control(values, frame=None, buttons=0, cancel=False, resume=False):
  accel_cruise = 1 if buttons == 1 else 0
  decel_cruise = 1 if buttons == 2 else 0
  resume_cruise = 1 if buttons == 3 else 0
  set_cruise = 1 if buttons == 4 else 0

  values.update({
    "COUNTER": (frame + 1) % 0x10 if frame is not None else (values["COUNTER"] + 1) % 16,
    "GRA_Abbrechen": cancel,
    "GRA_Tip_Wiederaufnahme": resume or resume_cruise,
    "GRA_Tip_Setzen": set_cruise,
    "GRA_Tip_Runter": decel_cruise,
    "GRA_Tip_Hoch": accel_cruise,
  })

  return values


def acc_control_value(cruise_available, gas_pressed, acc_faulted, long_active):
  acc_control = 0
  if acc_faulted:
    acc_control = 6
  elif long_active:
    if gas_pressed:
      acc_control = 4
    else:
      acc_control = 3
  elif cruise_available:
    acc_control = 2
  return acc_control


def acc_hud_status_value(cruise_available, gas_pressed, acc_faulted, long_active):
  # TODO: happens to resemble the ACC control value for now, but extend this for init
  return acc_control_value(cruise_available, gas_pressed, acc_faulted, long_active)


def create_acc_accel_control_1(values, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  values.update({
    "ACC_Typ": acc_type,
    "ACC_Status_ACC": acc_control,
    "ACC_StartStopp_Info": acc_enabled,
    "ACC_Sollbeschleunigung_02": accel if acc_enabled else 3.01,
    "ACC_zul_Regelabw_unten": 0.2,  # TODO: dynamic adjustment of comfort-band
    "ACC_zul_Regelabw_oben": 0.2,  # TODO: dynamic adjustment of comfort-band
    "ACC_neg_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_pos_Sollbeschl_Grad_02": 4.0 if acc_enabled else 0,  # TODO: dynamic adjustment of jerk limits
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
  })
  return values


def create_acc_accel_control_2(values, acc_type, acc_enabled, accel, acc_control, stopping, starting, esp_hold):
  if starting:
    acc_hold_type = 4  # hold release / startup
  elif esp_hold:
    acc_hold_type = 3  # hold standby
  elif stopping:
    acc_hold_type = 1  # hold request
  else:
    acc_hold_type = 0

  values.update({
    "ACC_Anhalteweg": 0.3 if stopping else 20.46,  # Distance to stop (stopping coordinator handles terminal roll-out)
    "ACC_Freilauf_Info": 2 if acc_enabled else 0,
    "ACC_Folgebeschl": 3.02,  # Not using secondary controller accel unless and until we understand its impact
    "ACC_Sollbeschleunigung_02": accel if acc_enabled else 3.01,
    "ACC_Anforderung_HMS": acc_hold_type,
    "ACC_Anfahren": starting,
    "ACC_Anhalten": stopping,
  })

  return values


def create_acc_hud_control_1(values, acc_hud_status, set_speed, set_speed_reached, lead_distance, target_distance_bars):
  values = {
    "ACC_Status_Anzeige": acc_hud_status,
    "ACC_Wunschgeschw_02": set_speed if set_speed < 250 else 327.36,
    "ACC_Wunschgeschw_erreicht": acc_hud_status == 3 and set_speed < 250 and set_speed_reached,
    "ACC_Gesetzte_Zeitluecke": target_distance_bars + 1,
    "ACC_Display_Prio": 2,
    "ACC_Abstandsindex": lead_distance,
    "ACC_Tachokranz": acc_hud_status in (3, 4),
  }
  return values


def create_acc_hud_control_2(values, acc_hud_status, set_speed, set_speed_reached, lead_distance, target_distance_bars):
  values.update({
    "ACC_Status_Zusatzanz": 2 if acc_hud_status == 3 and lead_distance > 0 else 0,
  })
  return values

