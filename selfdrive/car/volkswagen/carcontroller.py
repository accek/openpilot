from cereal import car
import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.selfdrive.car import DT_CTRL, apply_driver_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.volkswagen import mqbcan, pqcan
from openpilot.selfdrive.car.volkswagen.values import CANBUS, CarControllerParams, VolkswagenFlags
from openpilot.selfdrive.controls.lib.drive_helpers import VOLKSWAGEN_V_CRUISE_MIN

VisualAlert = car.CarControl.HUDControl.VisualAlert
LongCtrlState = car.CarControl.Actuators.LongControlState
ButtonType = car.CarState.ButtonEvent.Type


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    super().__init__(dbc_name, CP, VM)
    self.CCP = CarControllerParams(CP)
    self.CCS = pqcan if CP.flags & VolkswagenFlags.PQ else mqbcan
    self.packer_pt = CANPacker(dbc_name)
    self.ext_bus = CANBUS.pt if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera else CANBUS.cam

    self.apply_steer_last = 0
    self.forwarded_counters = {}
    self.eps_timer_soft_disable_alert = False
    self.hca_frame_timer_running = 0
    self.hca_frame_same_torque = 0
    self.last_button_frame = 0

    sub_services = ['longitudinalPlanSP', 'driverMonitoringState']
    if CP.openpilotLongitudinalControl:
      sub_services.append('radarState')
    self.sm = messaging.SubMaster(sub_services)

    self.param_s = Params()
    self.last_speed_limit_sign_tap_prev = False
    self.speed_limit = 0.
    self.speed_limit_offset = 0
    self.timer = 0
    self.final_speed_kph = 0
    self.init_speed = 0
    self.current_speed = 0
    self.v_set_dis = 0
    self.v_set_dis_prev = 0
    self.v_cruise_min = 0
    self.button_type = 0
    self.button_select = 0
    self.button_count = 0
    self.target_speed = 0
    self.t_interval = 7
    self.slc_active_stock = False
    self.sl_force_active_timer = 0
    self.v_tsc_state = 0
    self.slc_state = 0
    self.m_tsc_state = 0
    self.cruise_button = None
    self.last_cruise_button = None
    self.speed_diff = 0
    self.v_tsc = 0
    self.m_tsc = 0
    self.steady_speed = 0
    self.acc_type = -1
    self.send_count = 0

  def update(self, CC, CS, now_nanos):
    self.sm.update(0)

    if not self.CP.pcmCruiseSpeed:
      if self.sm.updated['longitudinalPlanSP']:
        self.v_tsc_state = self.sm['longitudinalPlanSP'].visionTurnControllerState
        self.slc_state = self.sm['longitudinalPlanSP'].speedLimitControlState
        self.m_tsc_state = self.sm['longitudinalPlanSP'].turnSpeedControlState
        self.speed_limit = self.sm['longitudinalPlanSP'].speedLimit
        self.speed_limit_offset = self.sm['longitudinalPlanSP'].speedLimitOffset
        self.v_tsc = self.sm['longitudinalPlanSP'].visionTurnSpeed
        self.m_tsc = self.sm['longitudinalPlanSP'].turnSpeed

      self.v_cruise_min = VOLKSWAGEN_V_CRUISE_MIN[CS.params_list.is_metric] * (CV.KPH_TO_MPH if not CS.params_list.is_metric else 1)

    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    if CS.out.carFaultedNonCritical:
      # Simply forward messages if the car is faulted (e.g. Emergency Assist is active)

      # TODO(accek): adjust panda safety to allow this
      self.forward_message(CS, self.CCS.MSG_STEERING, CANBUS.pt, can_sends)
      if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT:
        self.forward_message(CS, self.CCS.MSG_EPS, CANBUS.cam, can_sends)
      if self.CP.openpilotLongitudinalControl:
        self.forward_message(CS, self.CCS.MSG_ACC_1, CANBUS.pt, can_sends)
        self.forward_message(CS, self.CCS.MSG_ACC_2, CANBUS.pt, can_sends)
        self.forward_message(CS, self.CCS.MSG_TSK, CANBUS.cam, can_sends)
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_1, CANBUS.pt, can_sends)
        self.forward_message(CS, self.CCS.MSG_ACC_HUD_2, CANBUS.pt, can_sends)
      self.forward_message(CS, self.CCS.MSG_LKA_HUD, CANBUS.pt, can_sends)
      self.forward_message(CS, self.CCS.MSG_ACC_BUTTONS, CANBUS.cam, can_sends)

      new_actuators = actuators.as_builder()
      new_actuators.steer = 0
      new_actuators.steerOutputCan = 0

      return new_actuators, can_sends

    if not self.CP.pcmCruiseSpeed:
      if not self.last_speed_limit_sign_tap_prev and CS.params_list.last_speed_limit_sign_tap:
        self.sl_force_active_timer = self.frame
        self.param_s.put_bool_nonblocking("LastSpeedLimitSignTap", False)
      self.last_speed_limit_sign_tap_prev = CS.params_list.last_speed_limit_sign_tap

      sl_force_active = CS.params_list.speed_limit_control_enabled and (self.frame < (self.sl_force_active_timer * DT_CTRL + 2.0))
      sl_inactive = not sl_force_active and (not CS.params_list.speed_limit_control_enabled or (True if self.slc_state == 0 else False))
      sl_temp_inactive = not sl_force_active and (CS.params_list.speed_limit_control_enabled and (True if self.slc_state == 1 else False))
      slc_active = not sl_inactive and not sl_temp_inactive

      self.slc_active_stock = slc_active

    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      # Logic to avoid HCA state 4 "refused":
      #   * Don't steer unless HCA is in state 3 "ready" or 5 "active"
      #   * Don't steer at standstill
      #   * Don't send > 3.00 Newton-meters torque
      #   * Don't send the same torque for > 6 seconds
      #   * Don't send uninterrupted steering for > 360 seconds
      # MQB racks reset the uninterrupted steering timer after a single frame
      # of HCA disabled; this is done whenever output happens to be zero.

      if CC.latActive:
        new_steer = int(round(actuators.steer * self.CCP.STEER_MAX))
        apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.CCP)
        self.hca_frame_timer_running += self.CCP.STEER_STEP
        if self.apply_steer_last == apply_steer:
          self.hca_frame_same_torque += self.CCP.STEER_STEP
          if self.hca_frame_same_torque > self.CCP.STEER_TIME_STUCK_TORQUE / DT_CTRL:
            apply_steer -= (1, -1)[apply_steer < 0]
            self.hca_frame_same_torque = 0
        else:
          self.hca_frame_same_torque = 0
        hca_enabled = abs(apply_steer) > 0
      else:
        hca_enabled = False
        apply_steer = 0

      if not hca_enabled:
        self.hca_frame_timer_running = 0

      self.eps_timer_soft_disable_alert = self.hca_frame_timer_running > self.CCP.STEER_TIME_ALERT / DT_CTRL
      self.apply_steer_last = apply_steer

      self.forward_message(CS, self.CCS.MSG_STEERING, CANBUS.pt, can_sends, self.CCS.create_steering_control,
                           apply_steer, hca_enabled, ignore_counter=True, require_stock_values=False)

    if self.CP.flags & VolkswagenFlags.STOCK_HCA_PRESENT and self.can_forward_message(CS, self.CCS.MSG_EPS):
      # Pacify VW Emergency Assist driver inactivity detection by changing its view of driver steering input torque
      # to include small simulated inputs. See commaai/openpilot#23274 for background.
      sim_segment_frames = int(self.CCP.STEER_DRIVER_EA_SIMULATED)   # 1Nm/s
      sim_frame = self.frame % (2*sim_segment_frames)
      sign = 1 if CS.out.steeringTorque >= 0 else -1
      sim_torque = sim_frame if sim_frame < sim_segment_frames else 2*sim_segment_frames - sim_frame
      sim_torque = min(sim_torque, abs(2*self.apply_steer_last))
      if not self.sm.valid['driverMonitoringState'] or not self.sm.alive['driverMonitoringState'] \
          or self.sm['driverMonitoringState'].isDistracted:
        sim_torque = 0
      ea_simulated_torque = clip(CS.out.steeringTorque - sign*sim_torque, -self.CCP.STEER_MAX, self.CCP.STEER_MAX)
      self.forward_message(CS, self.CCS.MSG_EPS, CANBUS.cam, can_sends, self.CCS.create_eps_update, ea_simulated_torque)

    # **** Acceleration Controls ******************************************** #

    if self.CP.openpilotLongitudinalControl and (self.can_forward_message(CS, self.CCS.MSG_ACC_1) or self.can_forward_message(CS, self.CCS.MSG_ACC_2)):
      cancel_pressed = any(be.type == ButtonType.cancelCruise for be in CS.out.buttonEvents)
      acc_active = CC.longActive and not CS.out.brakePressed and not cancel_pressed
      acc_control = self.CCS.acc_control_value(CS.out.cruiseState.available, CC.cruiseControl.override,
                                               CS.out.accFaulted, acc_active)
      accel = clip(actuators.accel, self.CCP.ACCEL_MIN, self.CCP.ACCEL_MAX) if CC.longActive else 0
      stopping = actuators.longControlState == LongCtrlState.stopping
      starting = actuators.longControlState == LongCtrlState.pid and (CS.esp_hold_confirmation or CS.out.vEgo < self.CP.vEgoStopping)
      lead_accel = self.calculate_lead_accel() \
        if self.sm.valid['radarState'] and self.sm.alive['radarState'] else None
      self.forward_message(CS, self.CCS.MSG_ACC_1, CANBUS.pt, can_sends, self.CCS.create_acc_accel_control_1, CS.acc_type, accel,
                                                         acc_control, stopping, starting, CS.esp_hold_confirmation, lead_accel)
      self.forward_message(CS, self.CCS.MSG_ACC_2, CANBUS.pt, can_sends, self.CCS.create_acc_accel_control_2, CS.acc_type, accel,
                                                         acc_control, stopping, starting, CS.esp_hold_confirmation, lead_accel)

    if self.CP.openpilotLongitudinalControl and self.can_forward_message(CS, self.CCS.MSG_TSK):
      self.forward_message(CS, self.CCS.MSG_TSK, CANBUS.cam, can_sends, self.CCS.create_tsk_update, CS.stock_values)

    # **** HUD Controls ***************************************************** #

    if self.frame % self.CCP.LDW_STEP == 0 and self.can_forward_message(CS, self.CCS.MSG_LKA_HUD):
      hud_alert = 0
      if hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw):
        hud_alert = self.CCP.LDW_MESSAGES["laneAssistTakeOver"]
      self.forward_message(CS, self.CCS.MSG_LKA_HUD, CANBUS.pt, can_sends, self.CCS.create_lka_hud_control,
                           CS.madsEnabled, CC.latActive, hud_alert, hud_control)

    if self.CP.openpilotLongitudinalControl and (self.can_forward_message(CS, self.CCS.MSG_ACC_HUD_1) or self.can_forward_message(CS, self.CCS.MSG_ACC_HUD_2)):
      lead_distance = self.calculate_lead_distance(CS.out, hud_control, CS.upscale_lead_car_signal) \
        if self.sm.valid['radarState'] and self.sm.alive['radarState'] else 0
      acc_hud_status = self.CCS.acc_hud_status_value(CS.out.cruiseState.available, CS.out.gasPressed, CS.out.accFaulted, CC.longActive)
      # FIXME: follow the recent displayed-speed updates, also use mph_kmh toggle to fix display rounding problem?
      set_speed = hud_control.setSpeed * CV.MS_TO_KPH
      current_speed = CS.out.vEgo * CV.MS_TO_KPH
      set_speed_reached = abs(set_speed - current_speed) <= 3
      self.forward_message(CS, self.CCS.MSG_ACC_HUD_1, CANBUS.pt, can_sends, self.CCS.create_acc_hud_control_1,
                           acc_hud_status, set_speed, set_speed_reached, lead_distance, hud_control.leadDistanceBars)
      self.forward_message(CS, self.CCS.MSG_ACC_HUD_2, CANBUS.pt, can_sends, self.CCS.create_acc_hud_control_2,
                           acc_hud_status, set_speed, set_speed_reached, lead_distance, hud_control.leadDistanceBars)

    # **** Stock ACC Button Controls **************************************** #

    if self.CP.pcmCruise and (CC.cruiseControl.cancel or CC.cruiseControl.resume):
      self.forward_message(CS, self.CCS.MSG_ACC_BUTTONS, CANBUS.cam, can_sends, self.CCS.create_acc_buttons_control,
                           cancel=CC.cruiseControl.cancel, resume=CC.cruiseControl.resume)
    elif self.CP.openpilotLongitudinalControl:
      self.forward_message(CS, self.CCS.MSG_ACC_BUTTONS, CANBUS.cam, can_sends, self.CCS.create_acc_buttons_control,
                           frame='auto')

    if not (CC.cruiseControl.cancel or CC.cruiseControl.resume) and CS.out.cruiseState.enabled:
      if not self.CP.pcmCruiseSpeed:
        self.cruise_button = self.get_cruise_buttons(CS, CC.vCruise)
        if self.cruise_button is not None:
          if self.acc_type == -1:
            if self.button_count >= 2 and self.v_set_dis_prev != self.v_set_dis:
              self.acc_type = 1 if abs(self.v_set_dis - self.v_set_dis_prev) >= 10 and self.last_cruise_button in (1, 2) else \
                              0 if abs(self.v_set_dis - self.v_set_dis_prev) < 10 and self.last_cruise_button not in (1, 2) else 1
            if self.send_count >= 10 and self.v_set_dis_prev == self.v_set_dis:
              self.cruise_button = 3 if self.cruise_button == 1 else 4
          if self.acc_type == 0:
            self.cruise_button = 1 if self.cruise_button == 1 else 2  # accel, decel
          elif self.acc_type == 1:
            self.cruise_button = 3 if self.cruise_button == 1 else 4  # resume, set
          if self.frame % self.CCP.BTN_STEP == 0:
            self.forward_message(self.CCS.MSG_ACC_BUTTONS, self.ext_bus, can_sends, self.CCS.create_acc_buttons_control,
                                 frame=(self.frame // self.CCP.BTN_STEP), buttons=self.cruise_button, ignore_counter=True)
            self.send_count += 1
        else:
          self.send_count = 0
        self.last_cruise_button = self.cruise_button

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / self.CCP.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    self.v_set_dis_prev = self.v_set_dis
    self.frame += 1
    return new_actuators, can_sends

  # multikyd methods, sunnyhaibin logic
  def get_cruise_buttons_status(self, CS):
    if not CS.out.cruiseState.enabled:
      for be in CS.out.buttonEvents:
        if be.type in (ButtonType.accelCruise, ButtonType.resumeCruise,
                       ButtonType.decelCruise, ButtonType.setCruise) and be.pressed:
          self.timer = 40
        elif be.type in (ButtonType.gapAdjustCruiseUp, ButtonType.gapAdjustCruiseDown) and be.pressed:
          self.timer = 300
    elif self.timer:
      self.timer -= 1
    else:
      return 1
    return 0

  def get_target_speed(self, v_cruise_kph_prev):
    v_cruise_kph = v_cruise_kph_prev
    if self.slc_state > 1:
      v_cruise_kph = (self.speed_limit + self.speed_limit_offset) * CV.MS_TO_KPH
      if not self.slc_active_stock:
        v_cruise_kph = v_cruise_kph_prev
    return v_cruise_kph

  def get_button_type(self, button_type):
    self.type_status = "type_" + str(button_type)
    self.button_picker = getattr(self, self.type_status, lambda: "default")
    return self.button_picker()

  def reset_button(self):
    if self.button_type != 3:
      self.button_type = 0

  def type_default(self):
    self.button_type = 0
    return None

  def type_0(self):
    self.button_count = 0
    self.target_speed = self.init_speed
    self.speed_diff = self.target_speed - self.v_set_dis
    if self.target_speed > self.v_set_dis:
      self.button_type = 1
    elif self.target_speed < self.v_set_dis and self.v_set_dis > self.v_cruise_min:
      self.button_type = 2
    return None

  def type_1(self):
    cruise_button = 1
    self.button_count += 1
    if self.target_speed <= self.v_set_dis:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_2(self):
    cruise_button = 2
    self.button_count += 1
    if self.target_speed >= self.v_set_dis or self.v_set_dis <= self.v_cruise_min:
      self.button_count = 0
      self.button_type = 3
    elif self.button_count > 5:
      self.button_count = 0
      self.button_type = 3
    return cruise_button

  def type_3(self):
    cruise_button = None
    self.button_count += 1
    if self.button_count > self.t_interval:
      self.button_type = 0
    return cruise_button

  def get_curve_speed(self, target_speed_kph, v_cruise_kph_prev):
    if self.v_tsc_state != 0:
      vision_v_cruise_kph = self.v_tsc * CV.MS_TO_KPH
      if int(vision_v_cruise_kph) == int(v_cruise_kph_prev):
        vision_v_cruise_kph = 255
    else:
      vision_v_cruise_kph = 255
    if self.m_tsc_state > 1:
      map_v_cruise_kph = self.m_tsc * CV.MS_TO_KPH
      if int(map_v_cruise_kph) == 0.0:
        map_v_cruise_kph = 255
    else:
      map_v_cruise_kph = 255
    curve_speed = self.curve_speed_hysteresis(min(vision_v_cruise_kph, map_v_cruise_kph) + 2 * CV.MPH_TO_KPH)
    return min(target_speed_kph, curve_speed)

  def get_button_control(self, CS, final_speed, v_cruise_kph_prev):
    self.init_speed = round(min(final_speed, v_cruise_kph_prev) * (CV.KPH_TO_MPH if not CS.params_list.is_metric else 1))
    self.v_set_dis = round(CS.out.cruiseState.speed * (CV.MS_TO_MPH if not CS.params_list.is_metric else CV.MS_TO_KPH))
    cruise_button = self.get_button_type(self.button_type)
    return cruise_button

  def curve_speed_hysteresis(self, cur_speed: float, hyst=(0.75 * CV.MPH_TO_KPH)):
    if cur_speed > self.steady_speed:
      self.steady_speed = cur_speed
    elif cur_speed < self.steady_speed - hyst:
      self.steady_speed = cur_speed
    return self.steady_speed

  def get_cruise_buttons(self, CS, v_cruise_kph_prev):
    cruise_button = None
    if not self.get_cruise_buttons_status(CS):
      pass
    elif CS.out.cruiseState.enabled:
      set_speed_kph = self.get_target_speed(v_cruise_kph_prev)
      if self.slc_state > 1:
        target_speed_kph = set_speed_kph
      else:
        target_speed_kph = min(v_cruise_kph_prev, set_speed_kph)
      if self.v_tsc_state != 0 or self.m_tsc_state > 1:
        self.final_speed_kph = self.get_curve_speed(target_speed_kph, v_cruise_kph_prev)
      else:
        self.final_speed_kph = target_speed_kph

      cruise_button = self.get_button_control(CS, self.final_speed_kph, v_cruise_kph_prev)  # MPH/KPH based button presses
    return cruise_button

  def calculate_lead_distance(self, CS, hud_control, upscale):
    lead_one = self.sm["radarState"].leadOne
    lead_two = self.sm["radarState"].leadTwo
    v_ego = max(2.5, CS.vEgo)
    min_value = 64 if upscale else 1
    max_value = 1023 if upscale else 15
    max_relative_time = 4.0
    min_relative_time = 1.0

    distance = None
    if lead_one.status and (not lead_two.status or lead_one.dRel < lead_two.dRel):
      distance = lead_one.dRel
    elif lead_two.status:
      distance = lead_two.dRel
    if distance is not None:
      t_lead = distance / v_ego
      scale_fraction = (t_lead - min_relative_time) / (max_relative_time - min_relative_time)
      return round(clip(scale_fraction, 0.0, 1.0) * (max_value - min_value)) + min_value
    else:
      return max_value if hud_control.leadVisible else 0

  def calculate_lead_accel(self):
    lead_one = self.sm["radarState"].leadOne
    lead_two = self.sm["radarState"].leadTwo
    if lead_one.status and (not lead_two.status or lead_one.dRel < lead_two.dRel):
      return lead_one.aRel
    elif lead_two.status:
      return lead_two.aRel
    return None

  def can_forward_message(self, CS, msg_name):
    stock_values = CS.stock_values.get(msg_name)
    if stock_values is None:
      return False
    counter = stock_values.get("COUNTER")
    if counter is None:
      return True
    prev_counter = self.forwarded_counters.get(msg_name)
    return counter != prev_counter

  def forward_message(self, CS, msg_name, to_bus, can_sends, hook=None, *args, ignore_counter=False, require_stock_values=True, **kwargs):
    stock_values = CS.stock_values.get(msg_name)
    if stock_values is None:
      if require_stock_values:
        return False
      else:
        stock_values = {}
    counter = stock_values.get("COUNTER")
    prev_counter = self.forwarded_counters.get(msg_name)
    if counter is not None and counter == prev_counter and not ignore_counter:
      return False
    self.forwarded_counters[msg_name] = counter
    new_values = stock_values.copy()
    new_values.pop("CHECKSUM", None)
    if ignore_counter:
      new_values.pop("COUNTER", None)
    if hook is not None:
      new_values = hook(new_values, *args, **kwargs)
      if new_values is None:
        return False
    can_sends.append(self.packer_pt.make_can_msg(msg_name, to_bus, new_values))
