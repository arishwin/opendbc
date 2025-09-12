#!/usr/bin/env python3
from opendbc.car import structs
from opendbc.car import get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.perodua.values import CAR
from opendbc.car.perodua.carcontroller import CarController
from opendbc.car.perodua.carstate import CarState


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "perodua"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.toyota)]
    ret.safetyConfigs[0].safetyParam = 1

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.steerLimitTimer = 0.1              # time before steerLimitAlert is issued
    ret.steerActuatorDelay = 0.48          # Steering wheel actuator delay in seconds

    ret.lateralTuning.init('pid')

    ret.centerToFront = ret.wheelbase * 0.44
    ret.tireStiffnessFactor = 0.7933       # arbitruary number, don't touch

    ret.openpilotLongitudinalControl = True
    ret.lateralParams.torqueBP, ret.lateralParams.torqueV = [[0.], [255]]
    ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0.], [0.]]
    ret.longitudinalTuning.kpBP = [0., 5., 20.]
    ret.longitudinalTuning.kiBP = [5, 7, 28]

    if candidate == CAR.MYVI:
      ret.lateralTuning.pid.kiBP, ret.lateralTuning.pid.kpBP = [[0., 20], [0., 20]]
      ret.lateralTuning.pid.kiV, ret.lateralTuning.pid.kpV = [[0.09, 0.12], [0.10, 0.14]]
      ret.lateralTuning.pid.kf = 0.00012
      ret.longitudinalTuning.kpV = [1.0, 0.8, 0.8]
      ret.longitudinalTuning.kiV = [0.08, 0.04, 0.01]
      ret.wheelSpeedFactor = 1.31

    else:
      ret.dashcamOnly = True
      ret.safetyModel = structs.CarParams.SafetyModel.noOutput

    ret.minEnableSpeed = -1
    ret.stopAccel = -1.0
    ret.enableBsm = True
    ret.stoppingDecelRate = 0.10 # reach stopping target smoothly

    return ret