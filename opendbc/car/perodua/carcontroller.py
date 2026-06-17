from opendbc.can.packer import CANPacker

from opendbc.car.can_definitions import CanData
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.perodua.peroduacan import create_can_steer_command, create_accel_command, perodua_buttons,\
                                       create_brake_command, create_hud
from opendbc.car.perodua.values import DBC, CarControllerParams, cluster_speed_ratio
from numpy import clip, interp
from opendbc.car import DT_CTRL, Bus
from opendbc.car.common.conversions import Conversions as CV

from bisect import bisect_left

BRAKE_THRESHOLD = 0.01
BRAKE_MAG = [BRAKE_THRESHOLD,.32,.46,.61,.76,.90,1.06,1.21,1.35,1.51,4.0]
PUMP_VALS = [0, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1.0]
PUMP_RESET_INTERVAL = 1.5
PUMP_RESET_DURATION = 0.1

BRAKE_M = 1.4

# the ECU responds to the gap between ACC_CMD and its internal speed, so the lookahead sets
# how much speed error one m/s^2 of requested accel produces. A bigger error is needed at
# higher speed to make the ECU downshift / open the throttle against the reduced engine headroom.
SPEED_LOOKAHEAD_BP = [0., 2.9, 8.3, 16.7, 27.8]  # m/s
SPEED_LOOKAHEAD_V = [1.0, 1.2, 1.4, 1.6, 1.8]    # s

# Transfer-inversion gap controller (replaces the lookahead for positive accel).
# kph of (ACC_CMD - vEgo) speed gap to command per 1 m/s^2 of requested accel. The DNGA ACC ECU
# is a weak speed servo: on route 4abf798c.../000000ee it needs ~30-40 kph of gap for ~1 m/s^2 at
# city speed and its authority halves ~every 13 kph. Gentle at low speed (servo responsive there),
# aggressive at high speed. The 70/110 km/h points are EXTRAPOLATED - refit from a highway log.
GAP_GAIN_BP = [1.5, 4.2, 8.3, 13.9, 19.4, 30.6]   # m/s  (~5,15,30,50,70,110 km/h)
GAP_GAIN_V  = [5.,  10., 22., 38.,  55.,  90.]      # kph of gap per (m/s^2)
GAP_DEADBAND_KPH = 4.0     # base gap to clear the ECU deadband when accel intent>0
LEAD_GAP_CAP_KPH = 12.0    # cap commanded gap when a lead is visible (preserve fine following)


class BrakingStatus:
  STANDSTILL_INIT = 0
  BRAKE_HOLD = 1
  PUMP_RESET = 2


def apply_steer_torque_limits(apply_torque, apply_torque_last, driver_torque, blinkerOn, LIMITS):

  # limits due to driver torque and lane change
  reduced_torque_mult = 10 if blinkerOn else 1.5
  driver_max_torque = 255 + driver_torque * reduced_torque_mult
  driver_min_torque = -255 - driver_torque * reduced_torque_mult
  max_steer_allowed = max(min(255, driver_max_torque), 0)
  min_steer_allowed = min(max(-255, driver_min_torque), 0)
  apply_torque = clip(apply_torque, min_steer_allowed, max_steer_allowed)

  # slow rate if steer torque increases in magnitude
  if apply_torque_last > 0:
    apply_torque = clip(apply_torque, max(apply_torque_last - LIMITS.STEER_DELTA_DOWN, -LIMITS.STEER_DELTA_UP),
                        apply_torque_last + LIMITS.STEER_DELTA_UP)
  else:
    apply_torque = clip(apply_torque, apply_torque_last - LIMITS.STEER_DELTA_UP,
                        min(apply_torque_last + LIMITS.STEER_DELTA_DOWN, LIMITS.STEER_DELTA_UP))

  return int(round(float(apply_torque)))


# reset pump every PUMP_RESET_INTERVAL seconds for. Reset to zero for PUMP_RESET_DURATION
def standstill_brake(min_accel, ts_last, ts_now, prev_status):
  brake = min_accel
  status = prev_status

  dt = ts_now - ts_last
  if prev_status == BrakingStatus.PUMP_RESET and dt > PUMP_RESET_DURATION:
    status = BrakingStatus.BRAKE_HOLD
    ts_last = ts_now

  if prev_status == BrakingStatus.BRAKE_HOLD and dt > PUMP_RESET_INTERVAL:
    status = BrakingStatus.PUMP_RESET
    ts_last = ts_now

  if prev_status == BrakingStatus.STANDSTILL_INIT and dt > PUMP_RESET_INTERVAL:
    status = BrakingStatus.PUMP_RESET
    ts_last = ts_now

  if status == BrakingStatus.PUMP_RESET:
    brake = 0

  return brake, status, ts_last


def psd_brake(apply_brake, last_pump):
  # reversed engineered from Ativa stock braking
  # this is necessary for noiseless pump braking
  pump = PUMP_VALS[bisect_left(BRAKE_MAG, apply_brake)]

  # make sure the pump value decrease and increases within 0.1
  # to prevent brake bleeding.
  # TODO does it really prevent brake bleed?
  if abs(pump - last_pump) > 0.1:
    pump = last_pump + clip(pump - last_pump, -0.1, 0.1)
  last_pump = pump

  if apply_brake >= BRAKE_THRESHOLD:
    brake_req = 1
  else:
    brake_req = 0

  return pump, brake_req, last_pump


# 100hz rate_limit
def rate_limit_positive_speed(x, last):
  if x > last:
    return min(x, last + 0.006)
  else:
    return x


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.last_steer = 0
    self.steer_rate_limited = False
    self.steering_direction = False
    self.brake_pressed = False
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    self.brake = 0
    self.last_pump = 0

    # standstill globals
    self.prev_ts = 0.
    self.standstill_status = BrakingStatus.STANDSTILL_INIT
    self.min_standstill_accel = 0

    self.stockLdw = False
    self.frame = 0

    self.using_stock_acc = False
    self.fingerprint = CP.carFingerprint

    self.last_des_speed = 0
    self.keepalive_enabled = CP.openpilotLongitudinalControl

  def update(self, CC, CS, now_nanos):
    can_sends = []

    # Use openpilot engagement for gating
    lat_active = CC.latActive
    long_active = CC.longActive
    actuators = CC.actuators
    lead_visible = CC.hudControl.leadVisible
    rlane_visible = CC.hudControl.rightLaneVisible
    llane_visible = CC.hudControl.leftLaneVisible
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # steer
    steer_max_interp = interp(CS.out.vEgo, self.params.STEER_BP, self.params.STEER_LIM_TORQ)
    new_steer = int(round(actuators.torque * steer_max_interp))

    isBlinkerOn = CS.out.leftBlinker != CS.out.rightBlinker
    apply_steer = apply_steer_torque_limits(new_steer, self.last_steer, CS.out.steeringTorqueEps, isBlinkerOn, self.params)

    ts = self.frame * DT_CTRL

    # speed and brake. dnga is speed controlled: the positive-accel PID is done by the car, which
    # only responds to a sizeable gap between ACC_CMD and its internal speed. Size the gap from the
    # inverted transfer and clip to the set speed (strong request -> target near set speed, ECU ramps).
    acceleration = (actuators.accel - CS.stock_brake_mag * 0.85) if CS.out.vEgo > 0.25 else actuators.accel
    if acceleration > 0.:
      gain = float(interp(CS.out.vEgo, GAP_GAIN_BP, GAP_GAIN_V))   # kph of gap per m/s^2
      gap_kph = GAP_DEADBAND_KPH + gain * acceleration
      if lead_visible:                       # behind a lead stay gentle; lean on the brake path
        gap_kph = min(gap_kph, LEAD_GAP_CAP_KPH)
      set_speed_kph = CS.out.cruiseState.speedCluster * CV.MS_TO_KPH
      gap_kph = float(clip(gap_kph, 0., max(0., set_speed_kph - CS.out.vEgo * CV.MS_TO_KPH)))
      # plain float: capnp actuators.speed rejects numpy scalars (clip/interp return numpy)
      des_speed = float(CS.out.vEgo + gap_kph * CV.KPH_TO_MS)
    else:
      speed_lookahead_s = float(interp(CS.out.vEgo, SPEED_LOOKAHEAD_BP, SPEED_LOOKAHEAD_V))
      des_speed = max(CS.out.vEgo + acceleration * speed_lookahead_s, 0.)

    # the ECU tracks ACC_CMD against its internal cluster-scale speed, which reads ~5% above
    # wheel speed. Sent in wheel-speed scale the command carries a speed-proportional deficit
    # (~5 kph at 100 km/h) that swallows the accel request, so convert the target into the
    # cluster scale. Uses the same ratio as the set-speed conversion in carstate so that at
    # steady state ACC_CMD == SET_SPEED.
    acc_cmd_speed = des_speed * cluster_speed_ratio(CS.out.vEgo)
    apply_brake = 0 if (CS.out.gasPressed or actuators.accel >= 0) else clip(abs(actuators.accel / BRAKE_M), 0., 1.25)
    apply_brake = max(CS.stock_brake_mag * 0.6, apply_brake)

    if apply_brake > 0:
      self.last_des_speed = CS.out.vEgo
    else:
      self.last_des_speed = des_speed

    # positive des_speed rate limit
    # if CS.out.vEgo > 8.33:
    #   des_speed = rate_limit_positive_speed(des_speed, self.last_des_speed)

    # reduce max brake when below 10kmh to reduce jerk. TODO: more elegant way to do this?
    if CS.out.vEgo < 2.8:
      apply_brake = clip(apply_brake, 0., 0.8)

    # always clear dtc for dnga for the first 10s
    if self.frame <= 1000:
      can_sends.append(CanData(2015, b'\x01\x04\x00\x00\x00\x00\x00\x00', 0))

    if (self.frame % 2) == 0:
      # allow stock LDP passthrough
      self.stockLdw = CS.laneDepartWarning
      if self.stockLdw and not lat_active:
        apply_steer = -CS.ldpSteerV

      steer_req = (lat_active or self.stockLdw) and CS.lkas_latch and not CS.lkaDisabled
      can_sends.append(create_can_steer_command(self.packer, apply_steer, steer_req, (self.frame / 2) % 16))

    # CAN controlled longitudinal
    if (self.frame % 5) == 0:
      # spam cancel if op cancel
      if pcm_cancel_cmd:
        can_sends.append(perodua_buttons(self.packer, 0, 0, 1))

      # standstill logic
      if long_active and apply_brake > 0 and CS.out.standstill:
        if self.standstill_status == BrakingStatus.STANDSTILL_INIT:
          self.min_standstill_accel = apply_brake + 0.2
        apply_brake, self.standstill_status, self.prev_ts = standstill_brake(self.min_standstill_accel, self.prev_ts, ts, self.standstill_status)
      else:
        self.standstill_status = BrakingStatus.STANDSTILL_INIT
        self.prev_ts = ts

      # PSD brake logic
      pump, brake_req, self.last_pump = psd_brake(apply_brake, self.last_pump)

      can_sends.append(create_accel_command(self.packer, CS.out.cruiseState.speedCluster,
                       CS.out.cruiseState.available, long_active, lead_visible,
                       acc_cmd_speed, apply_brake, pump, CS.distance_val))

      # Let stock AEB kick in only when system not engaged
      aeb = not long_active and CS.aebV
      can_sends.append(create_brake_command(self.packer, long_active, brake_req, pump, apply_brake, aeb))
      can_sends.append(create_hud(self.packer, CS.out.cruiseState.available and CS.lkas_latch, lat_active, llane_visible,
                                  rlane_visible, self.stockLdw, CS.out.stockFcw, CS.out.stockAeb, CS.frontDepartWarning,
                                  CS.stock_lkc_off, CS.stock_fcw_off))

    self.last_steer = apply_steer
    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_steer / self.params.STEER_MAX
    new_actuators.speed = des_speed

    self.frame += 1
    return new_actuators, can_sends
