import math
from opendbc.car import DT_CTRL, structs
from opendbc.car.perodua.values import CAR
from opendbc.car.perodua.interface import CarInterface

FINGERPRINT = {i: {} for i in range(8)}


def _make_ci():
  CP = CarInterface.get_params(CAR.MYVI, FINGERPRINT, [], alpha_long=False, is_release=False, docs=False)
  return CarInterface(CP)


def test_perodua_longitudinal_update_runs_and_is_sane():
  """Drive CarController.update() across the full accel range, speeds and lead states.

  Regression guard: a numpy scalar assigned to the capnp actuators.speed field raised a
  KjException and crash-looped `card` on-car (routes 000000fd/000000fe). The stock
  test_car_interfaces only calls update() with a default CarControl (accel == 0), so it
  never entered the positive-accel branch and missed it. Here we exercise that branch.
  """
  ci = _make_ci()
  ci.update([])  # populate CS.out + stock_* attrs from (empty) CAN
  now = 0
  for accel in (-3.0, -1.5, -0.2, 0.0, 0.2, 0.8, 1.5, 2.0):
    for v_ego in (0.0, 5.0, 13.0, 25.0):
      for lead in (False, True):
        for long_active in (False, True):
          ci.CS.out.vEgo = v_ego
          ci.CS.out.cruiseState.speedCluster = 25.0  # 90 km/h, leave headroom
          CC = structs.CarControl()
          CC.enabled = True
          CC.latActive = True
          CC.longActive = long_active
          CC.actuators.accel = accel
          CC.hudControl.leadVisible = lead
          out, _ = ci.apply(CC.as_reader(), now)  # reaching here means speed serialized cleanly
          now += int(DT_CTRL * 1e9)
          assert math.isfinite(out.speed) and out.speed >= 0.0, (accel, v_ego)
          assert out.speed <= 25.0 + 1e-3, f"des_speed {out.speed} exceeds set speed (accel={accel}, v={v_ego})"
          assert math.isfinite(out.accel)
