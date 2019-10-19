from cereal import car, log
from common.realtime import sec_since_boot
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.drive_helpers import create_event, get_events, EventTypes as ET
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.car.volkswagen.values import CAR, FINGERPRINTS, ECU_FINGERPRINT, ECU
from selfdrive.car.volkswagen.carstate import CarState, get_mqb_gateway_can_parser, get_mqb_extended_can_parser
from common.params import Params
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint, is_ecu_disconnected
from selfdrive.car.interfaces import CarInterfaceBase

class CanBus(object):
  def __init__(self):
    self.gateway = 0
    self.extended = 2

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController):
    self.CP = CP

    self.frame = 0
    self.acc_active_prev = 0
    self.gas_pressed_prev = False
    self.brake_pressed_prev = False
    self.engageable = False
    self.CC = None

    # *** init the major players ***
    canbus = CanBus()
    self.CS = CarState(CP, canbus)
    self.VM = VehicleModel(CP)
    self.gw_cp = get_mqb_gateway_can_parser(CP, canbus)
    self.ex_cp = get_mqb_extended_can_parser(CP, canbus)

    # sending if read only is False
    if CarController is not None:
      self.CC = CarController(canbus, CP.carFingerprint)


  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 4.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), vin="", has_relay=False):
    ret = car.CarParams.new_message()

    # FIXME: Temp hack while working out fingerprint side
    has_auto_trans = True

    ret.carFingerprint = candidate
    ret.isPandaBlack = has_relay
    ret.carVin = vin
    tire_stiffness_factor = 1.

    ret.openpilotLongitudinalControl = False

    if candidate == CAR.GENERICMQB:
      # Set common MQB parameters that will apply globally
      ret.carName = "volkswagen"
      ret.safetyModel = car.CarParams.SafetyModel.volkswagen
      ret.enableCruise = True # Stock ACC still controls acceleration and braking
      ret.steerControlType = car.CarParams.SteerControlType.torque
      ret.steerLimitAlert = True # Enable UI alert when steering torque is maxed out

      # Additional common MQB parameters that may be overridden per-vehicle
      ret.steerRatio = 15.6
      ret.steerRateCost = 0.4
      ret.steerActuatorDelay = 0.05 # Hopefully all MQB racks are similar here
      ret.steerMaxBP = [0.]  # m/s
      ret.steerMaxV = [1.]

      # FIXME: Per-vehicle parameters need to be reintegrated.
      # For the time being, per-vehicle stuff is being archived since we
      # can't auto-detect very well yet. Now that tuning is figured out,
      # averaged params should work reasonably on a range of cars. Owners
      # can tweak here, as needed, until we have car type auto-detection.

      ret.mass = 1700 + STD_CARGO_KG
      ret.wheelbase = 2.75
      ret.centerToFront = ret.wheelbase * 0.45
      ret.steerRatio = 15.6
      ret.lateralTuning.pid.kf = 0.00006
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP  = [[0., 50 * CV.KPH_TO_MS], [0., 50 * CV.KPH_TO_MS]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.30, 0.50], [0.20, 0.20]]
      tire_stiffness_factor = 0.6

    # FIXME: Need to find a clean way to handle e-Golf without Getriebe_11 message
    if has_auto_trans:
      ret.transmissionType = car.CarParams.TransmissionType.automatic
    else:
      ret.transmissionType = car.CarParams.TransmissionType.manual

    # FIXME: follow 0.6.5 Comma refactoring to ensure camera-side is detected okay
    # ret.enableCamera = is_ecu_disconnected(fingerprint[0], FINGERPRINTS, ECU_FINGERPRINT, candidate, ECU.CAM) or has_relay
    ret.enableCamera = True
    ret.steerRatioRear = 0.

    # No support for OP longitudinal control on Volkswagen at this time.
    ret.gasMaxBP = [0.]
    ret.gasMaxV = [.5]
    ret.brakeMaxBP = [0.]
    ret.brakeMaxV = [1.]
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]
    ret.longitudinalTuning.kpBP = [5., 35.]
    ret.longitudinalTuning.kpV = [2.4, 1.5]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.36]
    ret.stoppingControl = True
    ret.startAccel = 0.8

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    return ret

  # returns a car.CarState
  def update(self, c, can_strings):
    canMonoTimes = []

    params = Params()

    self.gw_cp.update_strings(can_strings)
    self.ex_cp.update_strings(can_strings)

    self.CS.update(self.gw_cp, self.ex_cp)

    # create message
    ret = car.CarState.new_message()

    #ret.canValid = self.gw_cp.can_valid and self.ex_cp.can_valid
    ret.canValid = True

    # speeds
    ret.vEgo = self.CS.v_ego
    ret.aEgo = self.CS.a_ego
    ret.vEgoRaw = self.CS.v_ego_raw
    ret.yawRate = self.CS.yaw_rate * CV.DEG_TO_RAD
    ret.standstill = self.CS.standstill
    ret.wheelSpeeds.fl = self.CS.v_wheel_fl
    ret.wheelSpeeds.fr = self.CS.v_wheel_fr
    ret.wheelSpeeds.rl = self.CS.v_wheel_rl
    ret.wheelSpeeds.rr = self.CS.v_wheel_rr

    # steering wheel
    ret.steeringAngle = self.CS.angle_steers
    ret.steeringRate = self.CS.angle_steers_rate

    # torque and user override. Driver awareness
    # timer resets when the user uses the steering wheel.
    ret.steeringPressed = self.CS.steer_override
    ret.steeringTorque = self.CS.steer_torque_driver

    # ACC cruise state
    ret.cruiseState.available = self.CS.acc_enabled
    ret.cruiseState.enabled = self.CS.acc_active
    ret.cruiseState.speed = self.CS.cruise_set_speed

    # Blinker updates
    # TODO: We have a signal for actual external blinker state, need to import that in addition to the turnstalk
    ret.leftBlinker = bool(self.CS.left_blinker_on)
    ret.rightBlinker = bool(self.CS.right_blinker_on)

    # Doors open, seatbelt unfastened
    ret.doorOpen = not self.CS.door_all_closed
    ret.seatbeltUnlatched = not self.CS.seatbelt

    # Gas, brakes and shifting
    ret.gas = self.CS.pedal_gas / 100
    ret.gasPressed = self.CS.pedal_gas > 0
    ret.brake = self.CS.user_brake / 250 # FIXME: approximated, verify
    ret.brakePressed = bool(self.CS.brake_pressed)
    ret.brakeLights = bool(self.CS.brake_lights)
    ret.gearShifter = self.CS.gear_shifter

    # Update the EON metric configuration to match the car at first startup,
    # or if there's been a change.
    if self.CS.is_metric != self.CS.is_metric_prev:
      params.put("IsMetric", "1" if self.CS.is_metric == 1 else "0")

    # Update dynamic vehicle mass calculated by the drivetrain coordinator.
    # NOTE: At this time, OP's ParamsLearner probably won't make use of a mass
    # value that gets changed after startup.
    # FIXME: Can't actually do this without adding mass to CarState in capnp
    # ret.mass = self.CS.mass

    buttonEvents = []

    # Process button press or release events from ACC steering wheel or
    # control stalk buttons.
    if self.CS.gra_acc_buttons != self.CS.gra_acc_buttons_prev:
      if self.CS.gra_acc_buttons["main"] != self.CS.gra_acc_buttons_prev["main"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'altButton3'
        be.pressed = bool(self.CS.gra_acc_buttons["main"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["set"] != self.CS.gra_acc_buttons_prev["set"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'setCruise'
        be.pressed = bool(self.CS.gra_acc_buttons["set"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["resume"] != self.CS.gra_acc_buttons_prev["resume"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'resumeCruise'
        be.pressed = bool(self.CS.gra_acc_buttons["resume"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["cancel"] != self.CS.gra_acc_buttons_prev["cancel"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'cancel'
        be.pressed = bool(self.CS.gra_acc_buttons["cancel"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["accel"] != self.CS.gra_acc_buttons_prev["accel"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'accelCruise'
        be.pressed = bool(self.CS.gra_acc_buttons["accel"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["decel"] != self.CS.gra_acc_buttons_prev["decel"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'decelCruise'
        be.pressed = bool(self.CS.gra_acc_buttons["decel"])
        buttonEvents.append(be)
      if self.CS.gra_acc_buttons["timegap"] != self.CS.gra_acc_buttons_prev["timegap"]:
        be = car.CarState.ButtonEvent.new_message()
        be.type = 'gapAdjustCruise'
        be.pressed = bool(self.CS.gra_acc_buttons["timegap"])
        buttonEvents.append(be)

    # blinkers
    if self.CS.left_blinker_on != self.CS.prev_left_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'leftBlinker'
      be.pressed = bool(self.CS.left_blinker_on)
      buttonEvents.append(be)

    if self.CS.right_blinker_on != self.CS.prev_right_blinker_on:
      be = car.CarState.ButtonEvent.new_message()
      be.type = 'rightBlinker'
      be.pressed = bool(self.CS.right_blinker_on)
      buttonEvents.append(be)

    events = []

    # Vehicle operation safety checks and events
    if ret.doorOpen:
      events.append(create_event('doorOpen', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.seatbeltUnlatched:
      events.append(create_event('seatbeltNotLatched', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if ret.gearShifter == 'reverse':
      events.append(create_event('reverseGear', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    if not ret.gearShifter == 'drive' and not ret.gearShifter == 'eco':
      events.append(create_event('wrongGear', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    # TODO: pending add of alerts.py event
    # if ret.clutchPressed:
    #   events.append(create_event('clutchPressed', [ET.NO_ENTRY]))
    if self.CS.esp_disabled:
      events.append(create_event('espDisabled', [ET.NO_ENTRY, ET.SOFT_DISABLE]))
    if self.CS.park_brake:
      events.append(create_event('parkBrake', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if not self.CS.acc_enabled:
      events.append(create_event('wrongCarMode', [ET.NO_ENTRY, ET.USER_DISABLE]))

    # Vehicle health safety checks and events
    if self.CS.acc_error:
      events.append(create_event('radarFault', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))
    if self.CS.steer_error:
      events.append(create_event('steerTempUnavailable', [ET.NO_ENTRY, ET.IMMEDIATE_DISABLE]))

    # Per the Comma safety model, disable on pedals rising edge or when brake
    # is pressed and speed isn't zero.
    if (ret.gasPressed and not self.gas_pressed_prev) or \
            (ret.brakePressed and (not self.brake_pressed_prev or not ret.standstill)):
      events.append(create_event('pedalPressed', [ET.NO_ENTRY, ET.USER_DISABLE]))
    if ret.gasPressed:
      events.append(create_event('pedalPressed', [ET.PRE_ENABLE]))

    if self.CS.CP.openpilotLongitudinalControl:
      # Engagement and longitudinal control by openpilot, using vision or radar
      # fusion. Send OP engagement events based on the user ACC set and resume
      # buttons. The ACC gap adjustment increase and decrease buttons should
      # not trigger engagement.
      # TODO: For future visiond or radar fusion use; not currently supported.
      for b in buttonEvents:
        if b.type in ["setCruise", "resumeCruise"] and b.pressed:
          events.append(create_event('buttonEnable', [ET.ENABLE]))
        if b.type in ["cancel"] and b.pressed:
          events.append(create_event('buttonCancel', [ET.USER_DISABLE]))
    else:
      # Engagement and longitudinal control using stock ACC. Observe the car's
      # ACC engagement events and set OP engagement to match. If an OP safety
      # issue would prevent OP engagement, we prevent stock ACC from engaging
      # by filtering set/resume button presses later in CarController.
      self.engageable = not bool(get_events(events, [ET.NO_ENTRY]))
      if self.CS.acc_active and not self.acc_active_prev:
        events.append(create_event('pcmEnable', [ET.ENABLE]))
      # Make sure we disengage if stock ACC does, just in case stock ACC goes
      # away for a reason not already handled by OP above.
      if not self.CS.acc_active:
        events.append(create_event('pcmDisable', [ET.USER_DISABLE]))
      self.acc_active_prev = self.CS.acc_active

    ret.events = events
    ret.buttonEvents = buttonEvents
    ret.canMonoTimes = canMonoTimes

    # update previous brake/gas pressed
    self.gas_pressed_prev = ret.gasPressed
    self.brake_pressed_prev = ret.brakePressed

    # cast to reader so it can't be modified
    return ret.as_reader()

  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                   c.hudControl.visualAlert,
                   c.hudControl.audibleAlert,
                   c.hudControl.leftLaneVisible,
                   c.hudControl.rightLaneVisible,
                   self.engageable)
    self.frame += 1
    return can_sends
