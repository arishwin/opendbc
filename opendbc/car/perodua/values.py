from dataclasses import dataclass, field

from numpy import interp

from opendbc.car import CarSpecs, PlatformConfig, Platforms, DbcDict, Bus
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness

HUD_MULTIPLIER = 1.04

# The cluster (and the ACC ECU's internal speed used to track ACC_CMD) reads above true
# wheel speed. ACC_CMD must be sent in this cluster scale and the driver's set speed
# converted out of it, with the same ratio in both directions so the loop is consistent:
# at steady state ACC_CMD == SET_SPEED == the ECU's internal speed.
# Breakpoints take vEgo in m/s and keep the historical Myvi calibration: over the real
# 0-40 m/s range this evaluates to ~1.061 down to ~1.049.
CLUSTER_SPEED_RATIO_BP = [0., 140.]
CLUSTER_SPEED_RATIO_V = [1.0615, 1.0170]

Ecu = CarParams.Ecu


def cluster_speed_ratio(v_ego: float) -> float:
  return float(interp(v_ego, CLUSTER_SPEED_RATIO_BP, CLUSTER_SPEED_RATIO_V))


@dataclass
class PeroduaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'perodua_general_pt'})


@dataclass
class PeroduaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.toyota_a]))


class CAR(Platforms):
  """
  For illustration, we create a PERODUA brand class with a single model:
  Myvi PSD. If you'd like to keep everything under one `CAR` class, you can do so.
  """
  MYVI = PeroduaPlatformConfig(
    [PeroduaCarDocs("Perodua Myvi AV 2022-25", "Perodua Smart Drive Assist")],
    # TODO: see if steer ratio correct, comma logs reports 13
    CarSpecs(mass=1025.,wheelbase=2.5,steerRatio=17.44,tireStiffnessFactor=0.9871,centerToFrontRatio=0.44)
  )


class CarControllerParams:
  def __init__(self, CP):

    self.STEER_MAX = CP.lateralParams.torqueV[0]
    self.STEER_STEP = 4  # TODO: check if need change
    self.STEER_BP = CP.lateralParams.torqueBP
    self.STEER_LIM_TORQ = CP.lateralParams.torqueV
    self.BRAKE_SCALE = 3.3

    # for torque limit calculation
    self.STEER_DELTA_UP = 10
    self.STEER_DELTA_DOWN = 30


DBC = CAR.create_dbc_map()


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions):
  # For now FW-based query not implemented
  return set()


# TODO: maybe fix this, i have no idea how this works
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.SHORT_TESTER_PRESENT_REQUEST, StdQueries.OBD_VERSION_REQUEST],
      [StdQueries.SHORT_TESTER_PRESENT_RESPONSE, StdQueries.OBD_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine],
      bus=0,
    )],        # no active queries
  extra_ecus=[],      # or minimal list
)
