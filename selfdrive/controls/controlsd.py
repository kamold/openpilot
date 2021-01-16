#!/usr/bin/env python3
import os
from cereal import car, log
from common.numpy_fast import clip
from common.realtime import sec_since_boot, config_realtime_process, Priority, Ratekeeper, DT_CTRL
from common.profiler import Profiler
from common.params import Params, put_nonblocking
import cereal.messaging as messaging
from selfdrive.config import Conversions as CV
from selfdrive.boardd.boardd import can_list_to_can_capnp
from selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from selfdrive.controls.lib.lane_planner import CAMERA_OFFSET
from selfdrive.controls.lib.drive_helpers import update_v_cruise, initialize_v_cruise
from selfdrive.controls.lib.longcontrol import LongControl, STARTING_TARGET_SPEED
from selfdrive.controls.lib.latcontrol_pid import LatControlPID
from selfdrive.controls.lib.latcontrol_indi import LatControlINDI
from selfdrive.controls.lib.latcontrol_lqr import LatControlLQR
from selfdrive.controls.lib.events import Events, ET
from selfdrive.controls.lib.alertmanager import AlertManager
from selfdrive.controls.lib.vehicle_model import VehicleModel
from selfdrive.controls.lib.planner import LON_MPC_STEP
from selfdrive.locationd.calibrationd import Calibration
from selfdrive.hardware import HARDWARE

LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1
STEER_ANGLE_SATURATION_TIMEOUT = 1.0 / DT_CTRL
STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees

SIMULATION = "SIMULATION" in os.environ
NOSENSOR = "NOSENSOR" in os.environ

ThermalStatus = log.ThermalData.ThermalStatus
State = log.ControlsState.OpenpilotState
HwType = log.HealthData.HwType
LongitudinalPlanSource = log.Plan.LongitudinalPlanSource
Desire = log.PathPlan.Desire
LaneChangeState = log.PathPlan.LaneChangeState
LaneChangeDirection = log.PathPlan.LaneChangeDirection
EventName = car.CarEvent.EventName


class Controls:
  print("STARTING def __INIT___", flush=True)

  def __init__(self, sm=None, pm=None, can_sock=None):
    config_realtime_process(3, Priority.CTRL_HIGH)

    # Setup sockets
    self.pm = pm
    if self.pm is None:
      self.pm = messaging.PubMaster(['sendcan', 'controlsState', 'carState',
                                     'carControl', 'carEvents', 'carParams'])

    self.sm = sm
    if self.sm is None:
      self.sm = messaging.SubMaster(['thermal', 'health', 'model', 'liveCalibration', 'frontFrame',
                                     'dMonitoringState', 'plan', 'pathPlan', 'liveLocationKalman'])

    self.can_sock = can_sock
    if can_sock is None:
      can_timeout = None if os.environ.get('NO_CAN_TIMEOUT', False) else 100
      self.can_sock = messaging.sub_sock('can', timeout=can_timeout)

    # wait for one health and one CAN packet
    hw_type = messaging.recv_one(self.sm.sock['health']).health.hwType
    has_relay = hw_type in [HwType.blackPanda, HwType.uno, HwType.dos]
    print("Waiting for CAN messages...")
    get_one_can(self.can_sock)

    self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], has_relay)

    # read params
    print("STARTING car params", flush=True)

    params = Params()
    self.is_metric = params.get("IsMetric", encoding='utf8') == "1"
    self.is_ldw_enabled = params.get("IsLdwEnabled", encoding='utf8') == "1"
    internet_needed = (params.get("Offroad_ConnectivityNeeded", encoding='utf8') is not None) and (params.get("DisableUpdates") != b"1")
    community_feature_toggle = params.get("CommunityFeaturesToggle", encoding='utf8') == "1"
    openpilot_enabled_toggle = params.get("OpenpilotEnabledToggle", encoding='utf8') == "1"
    passive = params.get("Passive", encoding='utf8') == "1" or \
              internet_needed or not openpilot_enabled_toggle

    print ("STARTING car params DUMP", params, self.is_metric, self.is_ldw_enabled, internet_needed, community_feature_toggle, openpilot_enabled_toggle, passive, flush=True)

    # detect sound card presence and ensure successful init
    print("STARTING sounds_available check", flush=True)
    
    sounds_available = HARDWARE.get_sound_card_online()
    
    print("STARTING sounds_available check DUMP", sounds_available, flush=True)

    print("STARTING car_recognized", flush=True)
    
    car_recognized = self.CP.carName != 'mock'
    
    print ("STARTING car_recognized DUMP", car_recognized, flush=True)
    
    # If stock camera is disconnected, we loaded car controls and it's not dashcam mode
    print("STARTING 'If stock camera is disconnected, we loaded car controls and it's not dashcam mode'", flush=True)
    
    controller_available = self.CP.enableCamera and self.CI.CC is not None and not passive and not self.CP.dashcamOnly
    community_feature_disallowed = self.CP.communityFeature and not community_feature_toggle
    self.read_only = not car_recognized or not controller_available or \
                       self.CP.dashcamOnly or community_feature_disallowed
    if self.read_only:
      self.CP.safetyModel = car.CarParams.SafetyModel.noOutput
      
    print("STARTING 'If stock camera is disconnected, we loaded car controls and it's not dashcam mode' DUMP", controller_available, community_feature_disallowed, self.read_only, flush=True)

    # Write CarParams for radard and boardd safety mode
    print("STARTING params, self stuff", flush=True)
    
    cp_bytes = self.CP.to_bytes()
    params.put("CarParams", cp_bytes)
    put_nonblocking("CarParamsCache", cp_bytes)

    self.CC = car.CarControl.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.LoC = LongControl(self.CP, self.CI.compute_gb)
    self.VM = VehicleModel(self.CP)
    
    print("STARTING params, self stuff DUMP", cp_bytes, params.put, put_nonblocking, self.CC, self.AM, self.events, self.LoC, self.VM, flush=True)

    print("STARTING Lateral Tuning PID selection", flush=True)
    print("STARTING 'if self.CP.lateralTuning.which() == 'pid':'", flush=True)
    if self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP)
      print("STARTING 'if self.CP.lateralTuning.which() == 'pid':' DUMP, if you see this then the if command was successful", self.LaC, flush=True)
    print("STARTING 'elif self.CP.lateralTuning.which() == 'indi':'", flush=True)
    elif self.CP.lateralTuning.which() == 'indi':
      self.LaC = LatControlINDI(self.CP)
      print("STARTING 'elif self.CP.lateralTuning.which() == 'indi':' DUMP, if you see this then the if command was successful", self.LaC, flush=True)
    print("STARTING 'elif self.CP.lateralTuning.which() == 'lqr':'", flush=True)
    elif self.CP.lateralTuning.which() == 'lqr':
      self.LaC = LatControlLQR(self.CP)
      print("STARTING 'elif self.CP.lateralTuning.which() == 'lqr':' DUMP, if you see this then the if command was successful", self.LaC, flush=True)

    print("STARTING conclusion of Lateral Tuning PID selection DUMP", self.LaC, flush=True)

    print("STARTING self events list", flush=True)
    
    self.state = State.disabled
    self.enabled = False
    self.active = False
    self.can_rcv_error = False
    self.soft_disable_timer = 0
    self.v_cruise_kph = 255
    self.v_cruise_kph_last = 0
    self.mismatch_counter = 0
    self.can_error_counter = 0
    self.last_blinker_frame = 0
    self.saturated_count = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]

    self.sm['liveCalibration'].calStatus = Calibration.CALIBRATED
    self.sm['thermal'].freeSpace = 1.
    self.sm['dMonitoringState'].events = []
    self.sm['dMonitoringState'].awarenessStatus = 1.
    self.sm['dMonitoringState'].faceDetected = False

    self.startup_event = get_startup_event(car_recognized, controller_available, hw_type)
    
    print("STARTING self events list DUMP", self.state, self.enable, self.active, self.can_rcv_error, self.soft_disable_timer, self.v_cruise_kph, self.v_cruise_kph_last, self.mismatch_counter, self.can_error_counter, flush=True)
    print("STARTING self events list DUMP PT.2", self.last_blinker_frame, self.saturated_count, self.distance_traveled, self.last_functional_fan_frame, self.events_prev, self.current_alert_types, flush=True)
    print("STARTING self events list DUMP PT.3", self.sm['liveCalibration'].calStatus, self.sm['thermal'].freeSpace, self.sm['dMonitoringState'].events, self.sm['dMonitoringState'].awarenessStatus, self.sm['dMonitoringState'].faceDetected, flush=True)

    print("STARTING 4 if commands", flush=True)
    print("STARTING 'if not sounds_available:'", flush=True)
    if not sounds_available:
      self.events.add(EventName.soundsUnavailable, static=True)
      print("STARTING 'if not sounds_available:' DUMP, if you see this then the if command was successful", flush=True)
    print("STARTING 'if internet_needed:'", flush=True)
    if internet_needed:
      self.events.add(EventName.internetConnectivityNeeded, static=True)
      print("STARTING 'if internet_needed:' DUMP, if you see this then the if command was successful", flush=True)
    print("STARTING 'if community_feature_disallowed:'", flush=True)
    if community_feature_disallowed:
      self.events.add(EventName.communityFeatureDisallowed, static=True)
      print("STARTING 'if community_feature_disallowed:' DUMP, if you see this then the if command was successful", flush=True)
    print("STARTING 'if not car_recognized:'", flush=True)
    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)
      print("STARTING 'if not car_recognized:' DUMP, if you see this then the if command was successful", flush=True)
    print("STARTING 'if hw_type == HwType.whitePanda:'", flush=True)
    if hw_type == HwType.whitePanda:
      self.events.add(EventName.whitePandaUnsupportedDEPRECATED, static=True)
      print("STARTING 'if hw_type == HwType.whitePanda:' DUMP, if you see this then the if command was successful", flush=True)

    # controlsd is driven by can recv, expected at 100Hz
    print("STARTING controlsd is driven by can recv, expected at 100Hz", flush=True)
    
    self.rk = Ratekeeper(100, print_delay_threshold=None)
    self.prof = Profiler(False)  # off by default
    
    print("STARTING controlsd is driven by can recv, expected at 100Hz DUMP", self.rk, self.prof, flush=True)

  print("STARTING def update_events", flush=True)
  def update_events(self, CS):
    """Compute carEvents from carState"""
    
    self.events.clear()
    self.events.add_from_msg(CS.events)
    self.events.add_from_msg(self.sm['dMonitoringState'].events)
    print("STARTING if you see this then self.events is ok", flush=True)

    # Handle startup event
    print("STARTING Handle startup event", flush=True)
    
    print("STARTING self.startup_event", flush=True)
    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None
      print("STARTING 'if self.startup_event is not None:' DUMP, if you see this then the if command was successful", self.startup_event, flush=True)

    # Create events for battery, temperature, disk space, and memory
    print("STARTING self.sm['thermal'].batteryPercent", flush=True)
    if self.sm['thermal'].batteryPercent < 1 and self.sm['thermal'].chargingError:
      # at zero percent battery, while discharging, OP should not allowed
      self.events.add(EventName.lowBattery)
      print("STARTING 'if self.sm['thermal'].batteryPercent < 1 and self.sm['thermal'].chargingError:' DUMP, if you see this then the if command was successful", flush=True)
    print("STARTING self.sm['thermal'].thermalStatus", flush=True)
    if self.sm['thermal'].thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
      print("STARTING 'if self.sm['thermal'].thermalStatus >= ThermalStatus.red:' DUMP, if you see this then the if command was successful", flush=True)
    print("STARTING self.sm['thermal'].freeSpace", flush=True)
    if self.sm['thermal'].freeSpace < 0.07:
      # under 7% of space free no enable allowed
      self.events.add(EventName.outOfSpace)
      print("STARTING 'if self.sm['thermal'].freeSpace < 0.07:' DUMP, if you see this then the if command was successful", flush=True)
    print("STARTING self.sm['thermal'].memUsedPercent", flush=True)
    if self.sm['thermal'].memUsedPercent > 90:
      self.events.add(EventName.lowMemory)
      print("STARTING 'if self.sm['thermal'].memUsedPercent > 90:' DUMP, if you see this then the if command was successful", flush=True)

    # Alert if fan isn't spinning for 5 seconds
    print("STARTING # Alert if fan isn't spinning for 5 seconds", flush=True)
    
    print("STARTING if self.sm['health'].hwType in [HwType.uno, HwType.dos]:", flush=True)
    if self.sm['health'].hwType in [HwType.uno, HwType.dos]:
      print("STARTING if self.sm['health'].fanSpeedRpm == 0 and self.sm['thermal'].fanSpeed > 50:", flush=True)
      if self.sm['health'].fanSpeedRpm == 0 and self.sm['thermal'].fanSpeed > 50:
        print("STARTING if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 5.0:", flush=True)
        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 5.0:
          self.events.add(EventName.fanMalfunction)
          print("STARTING self.events.add(EventName.fanMalfunction)", flush=True)
      else:
        self.last_functional_fan_frame = self.sm.frame
        print("STARTING self.last_functional_fan_frame = self.sm.frame", flush=True)

    # Handle calibration status
    print("STARTING # Handle calibration status", flush=True)
    
    print("STARTING cal_status", flush=True)
    cal_status = self.sm['liveCalibration'].calStatus
    print("STARTING cal_status DUMP", cal_status, flush=True)
    print("STARTING if cal_status != Calibration.CALIBRATED:", flush=True)
    if cal_status != Calibration.CALIBRATED:
      print("STARTING if cal_status == Calibration.UNCALIBRATED:", flush=True)
      if cal_status == Calibration.UNCALIBRATED:
        self.events.add(EventName.calibrationIncomplete)
        print("STARTING self.events.add(EventName.calibrationIncomplete)", flush=True)
      else:
        self.events.add(EventName.calibrationInvalid)
        print("STARTING self.events.add(EventName.calibrationInvalid)", flush=True)

    # Handle lane change
    print("STARTING # Handle lane change", flush=True)
    
    print("STARTING if self.sm['pathPlan'].laneChangeState == LaneChangeState.preLaneChange:", flush=True)
    if self.sm['pathPlan'].laneChangeState == LaneChangeState.preLaneChange:
      direction = self.sm['pathPlan'].laneChangeDirection
      print("STARTING direction = self.sm['pathPlan'].laneChangeDirection DUMP", direction, flush=True)
      print("STARTING if (CS.leftBlindspot and direction == LaneChangeDirection.left) or", flush=True)
      if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
         (CS.rightBlindspot and direction == LaneChangeDirection.right):
        self.events.add(EventName.laneChangeBlocked)
        print("STARTING self.events.add(EventName.laneChangeBlocked)", flush=True)
      else:
        print("STARTING if direction == LaneChangeDirection.left:", flush=True)
        if direction == LaneChangeDirection.left:
          self.events.add(EventName.preLaneChangeLeft)
          print("STARTING self.events.add(EventName.preLaneChangeLeft)", flush=True)
        else:
          self.events.add(EventName.preLaneChangeRight)
          print("STARTING self.events.add(EventName.preLaneChangeRight)", flush=True)
    print("STARTING elif self.sm['pathPlan'].laneChangeState in [LaneChangeState.laneChangeStarting,", flush=True)
    elif self.sm['pathPlan'].laneChangeState in [LaneChangeState.laneChangeStarting,
                                                 LaneChangeState.laneChangeFinishing]:
      self.events.add(EventName.laneChange)
      print("STARTING self.events.add(EventName.laneChange)", flush=True)

    if self.can_rcv_error or (not CS.canValid and self.sm.frame > 5 / DT_CTRL):
      self.events.add(EventName.canError)
    if (self.sm['health'].safetyModel != self.CP.safetyModel and self.sm.frame > 2 / DT_CTRL) or \
       self.mismatch_counter >= 200:
      self.events.add(EventName.controlsMismatch)
    if not self.sm.alive['plan'] and self.sm.alive['pathPlan']:
      # only plan not being received: radar not communicating
      self.events.add(EventName.radarCommIssue)
    elif not self.sm.all_alive_and_valid():
      self.events.add(EventName.commIssue)
    if not self.sm['pathPlan'].mpcSolutionValid:
      self.events.add(EventName.plannerError)
    if not self.sm['liveLocationKalman'].sensorsOK and not NOSENSOR:
      if self.sm.frame > 5 / DT_CTRL:  # Give locationd some time to receive all the inputs
        self.events.add(EventName.sensorDataInvalid)
    if not self.sm['liveLocationKalman'].gpsOK and (self.distance_traveled > 1000):
      # Not show in first 1 km to allow for driving out of garage. This event shows after 5 minutes
      if not (SIMULATION or NOSENSOR):  # TODO: send GPS in carla
        self.events.add(EventName.noGps)
    if not self.sm['pathPlan'].paramsValid:
      self.events.add(EventName.vehicleModelInvalid)
    if not self.sm['liveLocationKalman'].posenetOK:
      self.events.add(EventName.posenetInvalid)
    if not self.sm['liveLocationKalman'].deviceStable:
      self.events.add(EventName.deviceFalling)
    if not self.sm['plan'].radarValid:
      self.events.add(EventName.radarFault)
    if self.sm['plan'].radarCanError:
      self.events.add(EventName.radarCanError)
    if log.HealthData.FaultType.relayMalfunction in self.sm['health'].faults:
      self.events.add(EventName.relayMalfunction)
    if self.sm['plan'].fcw:
      self.events.add(EventName.fcw)
    if not self.sm.alive['frontFrame'] and (self.sm.frame > 5 / DT_CTRL) and not SIMULATION:
      self.events.add(EventName.cameraMalfunction)

    if self.sm['model'].frameDropPerc > 20 and not SIMULATION:
      self.events.add(EventName.modeldLagging)
    print("STARTING if you see this message then all the events for malfunction detection should be working?", flush=True)

    # Only allow engagement with brake pressed when stopped behind another stopped car
    print("STARTING # Only allow engagement with brake pressed when stopped behind another stopped car", flush=True)
    
    print("STARTING if CS.brakePressed and self.sm['plan'].vTargetFuture >= STARTING_TARGET_SPEED", flush=True)
    if CS.brakePressed and self.sm['plan'].vTargetFuture >= STARTING_TARGET_SPEED \
      and self.CP.openpilotLongitudinalControl and CS.vEgo < 0.3:
      self.events.add(EventName.noTarget)
      print("STARTING self.events.add(EventName.noTarget)", flush=True)

  print("STARTING def data_sample(self):", flush=True)
  def data_sample(self):
    """Receive data from sockets and update carState"""

    # Update carState from CAN
    print("STARTING # Update carState from CAN", flush=True)
    print("STARTING can_strs, CS", flush=True)
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    CS = self.CI.update(self.CC, can_strs)

    self.sm.update(0)
    print("STARTING can_strs, CS DUMP", can_strs, CS, flush=True)

    # Check for CAN timeout
    print("STARTING # Check for CAN timeout", flush=True)
    print("STARTING if not can_strs:", flush=True)
    if not can_strs:
      self.can_error_counter += 1
      self.can_rcv_error = True
      print("STARTING self.can_rcv_error TRUE", self.can_rcv_error, flush=True)
    else:
      self.can_rcv_error = False
      print("STARTING self.can_rcv_error FALSE", self.can_rcv_error, flush=True)

    # When the panda and controlsd do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0
      print("STARTING self.mismatch_counter = 0", flush=True)

    if not self.sm['health'].controlsAllowed and self.enabled:
      self.mismatch_counter += 1
      print("STARTING if not self.sm['health'].controlsAllowed and self.enabled:", flush=True)

    self.distance_traveled += CS.vEgo * DT_CTRL

    return CS

  print("STARTING def state_transition(self, CS):", flush=True)
  def state_transition(self, CS):
    """Compute conditional state transitions and execute actions on state transitions"""

    print("STARTING self.v_cruise_kph_last", flush=True)
    self.v_cruise_kph_last = self.v_cruise_kph
    print("STARTING self.v_cruise_kph_last DUMP", self.v_cruise_kph_last, flush=True)

    # if stock cruise is completely disabled, then we can use our own set speed logic
    if not self.CP.enableCruise:
      self.v_cruise_kph = update_v_cruise(self.v_cruise_kph, CS.buttonEvents, self.enabled)
      print("STARTING self.v_cruise_kph = update_v_cruise(self.v_cruise_kph, CS.buttonEvents, self.enabled) DUMP", self.v_cruise_kph, flush=True)
    elif self.CP.enableCruise and CS.cruiseState.enabled:
      self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
      print("STARTING self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH DUMP", self.v_cruise_kph, flush=True)

    # decrease the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)
    print("STARTING self.soft_disable_timer = max(0, self.soft_disable_timer - 1)", self.soft_disable_timer, flush=True)

    self.current_alert_types = [ET.PERMANENT]
    print("STARTING self.current_alert_types = [ET.PERMANENT]", self.current_alert_types, flush=True)

    # ENABLED, PRE ENABLING, SOFT DISABLING
    print("STARTING if self.state != State.disabled:", flush=True)
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      print("STARTING if self.events.any(ET.USER_DISABLE):", flush=True)
      if self.events.any(ET.USER_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)
        print("STARTING self.state = State.disabled DUMP", self.state, flush=True)

      print("STARTING elif self.events.any(ET.IMMEDIATE_DISABLE):", flush=True)
      elif self.events.any(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.IMMEDIATE_DISABLE)
        print("STARTING self.state = State.disabled DUMP", self.state, flush=True)

      else:
        # ENABLED
        print("STARTING if self.state == State.enabled:", flush=True)
        if self.state == State.enabled:
          print("STARTING if self.events.any(ET.SOFT_DISABLE):", flush=True)
          if self.events.any(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = 300   # 3s
            self.current_alert_types.append(ET.SOFT_DISABLE)
            print("STARTING self.state = State.softDisabling", self.state, flush=True)

        # SOFT DISABLING
        print("STARTING elif self.state == State.softDisabling:", flush=True)
        elif self.state == State.softDisabling:
          print("STARTING if not self.events.any(ET.SOFT_DISABLE):", flush=True)
          if not self.events.any(ET.SOFT_DISABLE):
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled
            print("STARTING self.state = State.enabled", self.state, flush=True)

          print("STARTING elif self.events.any(ET.SOFT_DISABLE) and self.soft_disable_timer > 0:", flush=True)
          elif self.events.any(ET.SOFT_DISABLE) and self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)
            print("STARTING self.current_alert_types.append(ET.SOFT_DISABLE)", flush=True)

          print("STARTING elif self.soft_disable_timer <= 0:", flush=True)
          elif self.soft_disable_timer <= 0:
            self.state = State.disabled
            print("STARTING self.state = State.disabled", flush=True)

        # PRE ENABLING
        print("STARTING elif self.state == State.preEnabled:", flush=True)
        elif self.state == State.preEnabled:
          print("STARTING if not self.events.any(ET.PRE_ENABLE):", flush=True)
          if not self.events.any(ET.PRE_ENABLE):
            self.state = State.enabled
            print("STARTING self.state = State.enabled", self.state, flush=True)
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)
            print("STARTING self.current_alert_types.append(ET.PRE_ENABLE):", flush=True)

    # DISABLED
    print("STARTING elif self.state == State.disabled:", flush=True)
    elif self.state == State.disabled:
      print("STARTING if self.events.any(ET.ENABLE):", flush=True)
      if self.events.any(ET.ENABLE):
        print("STARTING if self.events.any(ET.NO_ENTRY):", flush=True)
        if self.events.any(ET.NO_ENTRY):
          self.current_alert_types.append(ET.NO_ENTRY)
          print("STARTING self.current_alert_types.append(ET.NO_ENTRY)", flush=True)

        else:
          print("STARTING if self.events.any(ET.PRE_ENABLE):", flush=True)
          if self.events.any(ET.PRE_ENABLE):
            self.state = State.preEnabled
            print("STARTING self.state = State.preEnabled", self.state, flush=True)
          else:
            self.state = State.enabled
            print("STARTING self.state = State.enabled", self.state, flush=True)
          self.current_alert_types.append(ET.ENABLE)
          print("STARTING self.current_alert_types.append(ET.ENABLE)", flush=True)
          self.v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, self.v_cruise_kph_last)
          print("STARTING self.v_cruise_kph = initialize_v_cruise(CS.vEgo, CS.buttonEvents, self.v_cruise_kph_last)", self.v_cruise_kph, flush=True)

    # Check if actuators are enabled
    print("STARTING self.active = self.state == State.enabled or self.state == State.softDisabling", flush=True)
    self.active = self.state == State.enabled or self.state == State.softDisabling
    print("STARTING self.active = self.state == State.enabled or self.state == State.softDisabling DUMP", self.active, flush=True)
    print("STARTING if self.active:", flush=True)
    if self.active:
      self.current_alert_types.append(ET.WARNING)
      print("STARTING self.current_alert_types.append(ET.WARNING)", flush=True)

    # Check if openpilot is engaged
    print("STARTING self.enabled", flush=True)
    self.enabled = self.active or self.state == State.preEnabled
    print("STARTING self.enabled DUMP", self.enabled, flush=True)

  print("STARTING def state_control(self, CS):", flush=True)
  def state_control(self, CS):
    """Given the state, this function returns an actuators packet"""

    print("STARTING plan, path_plan, and actuators", flush=True)
    plan = self.sm['plan']
    path_plan = self.sm['pathPlan']

    actuators = car.CarControl.Actuators.new_message()
    print("STARTING plan, path_plan, and actuators DUMP", plan, path_plan, actuators, flush=True)

    print("STARTING if CS.leftBlinker or CS.rightBlinker:", flush=True)
    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = self.sm.frame
      print("STARTING self.last_blinker_frame = self.sm.frame", flush=True)

    # State specific actions

    print("STARTING if not self.active:", flush=True)
    if not self.active:
      self.LaC.reset()
      self.LoC.reset(v_pid=CS.vEgo)
      print("STARTING self.LaC.reset()", flush=True)
      print("STARTING self.LoC.reset(v_pid=CS.vEgo)", flush=True)

    print("STARTING plan_age, a_acc_sol, v_acc_sol, actuators.gas, actuators.brake, actuators.steer, actuators.steerAngle, and angle_control_saturated", flush=True)
    
    plan_age = DT_CTRL * (self.sm.frame - self.sm.rcv_frame['plan'])
    # no greater than dt mpc + dt, to prevent too high extraps
    dt = min(plan_age, LON_MPC_STEP + DT_CTRL) + DT_CTRL

    a_acc_sol = plan.aStart + (dt / LON_MPC_STEP) * (plan.aTarget - plan.aStart)
    v_acc_sol = plan.vStart + dt * (a_acc_sol + plan.aStart) / 2.0

    # Gas/Brake PID loop
    actuators.gas, actuators.brake = self.LoC.update(self.active, CS, v_acc_sol, plan.vTargetFuture, a_acc_sol, self.CP)
    # Steering PID loop and lateral MPC
    actuators.steer, actuators.steerAngle, lac_log = self.LaC.update(self.active, CS, self.CP, path_plan)

    # Check for difference between desired angle and angle for angle based control
    angle_control_saturated = self.CP.steerControlType == car.CarParams.SteerControlType.angle and \
      abs(actuators.steerAngle - CS.steeringAngle) > STEER_ANGLE_SATURATION_THRESHOLD
      
    print("STARTING plan_age, a_acc_sol, v_acc_sol, actuators.gas, actuators.brake, actuators.steer, actuators.steerAngle, and angle_control_saturated DUMP", plan_age, a_acc_sol, v_acc_sol, actuators.gas, actuators.brake, actuators.steer, actuators.steerAngle, angle_control_saturated flush=True)

    print("STARTING if angle_control_saturated and not CS.steeringPressed and self.active:", flush=True)
    if angle_control_saturated and not CS.steeringPressed and self.active:
      self.saturated_count += 1
      print("STARTING self.saturated_count += 1", flush=True)
    else:
      self.saturated_count = 0
      print("STARTING self.saturated_count = 0", flush=True)

    # Send a "steering required alert" if saturation count has reached the limit
    print("STARTING if (lac_log.saturated and not CS.steeringPressed) or", flush=True)
    if (lac_log.saturated and not CS.steeringPressed) or \
       (self.saturated_count > STEER_ANGLE_SATURATION_TIMEOUT):
       print("STARTING (self.saturated_count > STEER_ANGLE_SATURATION_TIMEOUT):", flush=True)
      # Check if we deviated from the path
      print("STARTING left_deviation, right_deviation", flush=True)
      left_deviation = actuators.steer > 0 and path_plan.dPoly[3] > 0.1
      right_deviation = actuators.steer < 0 and path_plan.dPoly[3] < -0.1
      print("STARTING left_deviation, right_deviation DUMP", left_deviation, right_deviation, flush=True)

      print("STARTING if left_deviation or right_deviation:", flush=True)
      if left_deviation or right_deviation:
        self.events.add(EventName.steerSaturated)
        print("STARTING self.events.add(EventName.steerSaturated)", flush=True)

    return actuators, v_acc_sol, a_acc_sol, lac_log

  print("STARTING def publish_logs(self, CS, start_time, actuators, v_acc, a_acc, lac_log):", flush=True)
  def publish_logs(self, CS, start_time, actuators, v_acc, a_acc, lac_log):
    """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

    print("STARTING CC, CC.enabled, CC.actuators, CC.cruiseControl.override, CC.cruiseControl.cancel", flush=True)
    CC = car.CarControl.new_message()
    CC.enabled = self.enabled
    CC.actuators = actuators

    CC.cruiseControl.override = True
    CC.cruiseControl.cancel = not self.CP.enableCruise or (not self.enabled and CS.cruiseState.enabled)
    print("STARTING CC, CC.enabled, CC.actuators, CC.cruiseControl.override, CC.cruiseControl.cancel DUMP", CC, CC.enabled, CC.actuators, CC.cruiseControl.override, CC.cruiseControl.cancel, flush=True)

    # Some override values for Honda
    # brake discount removes a sharp nonlinearity
    print("STARTING brake_discount, speed_override, CC.cruiseControl.speedOverride, CC.cruiseControl.accelOverride, CC.hudControl.setSpeed, CC.hudControl.speedVisible, CC.hudControl.lanesVisible, CC.hudControl.leadVisible, right_lane_visible, left_lane_visible, CC.hudControl.rightLaneVisible, CC.hudControl.leftLaneVisible, recent_blinker, ldw_allowed", flush=True)
    brake_discount = (1.0 - clip(actuators.brake * 3., 0.0, 1.0))
    speed_override = max(0.0, (self.LoC.v_pid + CS.cruiseState.speedOffset) * brake_discount)
    CC.cruiseControl.speedOverride = float(speed_override if self.CP.enableCruise else 0.0)
    CC.cruiseControl.accelOverride = self.CI.calc_accel_override(CS.aEgo, self.sm['plan'].aTarget, CS.vEgo, self.sm['plan'].vTarget)

    CC.hudControl.setSpeed = float(self.v_cruise_kph * CV.KPH_TO_MS)
    CC.hudControl.speedVisible = self.enabled
    CC.hudControl.lanesVisible = self.enabled
    CC.hudControl.leadVisible = self.sm['plan'].hasLead

    right_lane_visible = self.sm['pathPlan'].rProb > 0.5
    left_lane_visible = self.sm['pathPlan'].lProb > 0.5
    CC.hudControl.rightLaneVisible = bool(right_lane_visible)
    CC.hudControl.leftLaneVisible = bool(left_lane_visible)

    recent_blinker = (self.sm.frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
    ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker \
                    and not self.active and self.sm['liveCalibration'].calStatus == Calibration.CALIBRATED
    print("STARTING brake_discount, speed_override, CC.cruiseControl.speedOverride, CC.cruiseControl.accelOverride, CC.hudControl.setSpeed, CC.hudControl.speedVisible, CC.hudControl.lanesVisible, CC.hudControl.leadVisible, right_lane_visible, left_lane_visible, CC.hudControl.rightLaneVisible, CC.hudControl.leftLaneVisible, recent_blinker, ldw_allowed DUMP", brake_discount, speed_override, CC.cruiseControl.speedOverride, CC.cruiseControl.accelOverride, CC.hudControl.setSpeed, CC.hudControl.speedVisible, CC.hudControl.lanesVisible, CC.hudControl.leadVisible, right_lane_visible, left_lane_visible, CC.hudControl.rightLaneVisible, CC.hudControl.leftLaneVisible, recent_blinker, ldw_allowed, flush=True)

    print("STARTING meta = self.sm['model'].meta", flush=True)
    meta = self.sm['model'].meta
    print("STARTING meta = self.sm['model'].meta DUMP", meta, flush=True)
    print("STARTING if len(meta.desirePrediction) and ldw_allowed:", flush=True)
    if len(meta.desirePrediction) and ldw_allowed:
      print("STARTING l_lane_change_prob, r_lane_change_prob, l_lane_close, r_lane_close, CC.hudControl.leftLaneDepart, CC.hudControl.rightLaneDepart", flush=True)
      l_lane_change_prob = meta.desirePrediction[Desire.laneChangeLeft - 1]
      r_lane_change_prob = meta.desirePrediction[Desire.laneChangeRight - 1]
      l_lane_close = left_lane_visible and (self.sm['pathPlan'].lPoly[3] < (1.08 - CAMERA_OFFSET))
      r_lane_close = right_lane_visible and (self.sm['pathPlan'].rPoly[3] > -(1.08 + CAMERA_OFFSET))

      CC.hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      CC.hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)
      print("STARTING l_lane_change_prob, r_lane_change_prob, l_lane_close, r_lane_close, CC.hudControl.leftLaneDepart, CC.hudControl.rightLaneDepart DUMP", l_lane_change_prob, r_lane_change_prob, l_lane_close, r_lane_close, CC.hudControl.leftLaneDepart, CC.hudControl.rightLaneDepart, flush=True)

    print("STARTING if CC.hudControl.rightLaneDepart or CC.hudControl.leftLaneDepart:", flush=True)
    if CC.hudControl.rightLaneDepart or CC.hudControl.leftLaneDepart:
      self.events.add(EventName.ldw)
      print("STARTING self.events.add(EventName.ldw)", flush=True)

    print("STARTING clear_event, alerts, CC.hudControl.visualAlert", flush=True)
    clear_event = ET.WARNING if ET.WARNING not in self.current_alert_types else None
    alerts = self.events.create_alerts(self.current_alert_types, [self.CP, self.sm, self.is_metric])
    self.AM.add_many(self.sm.frame, alerts, self.enabled)
    self.AM.process_alerts(self.sm.frame, clear_event)
    CC.hudControl.visualAlert = self.AM.visual_alert
    print("STARTING clear_event, alerts, CC.hudControl.visualAlert DUMP", clear_event, alerts, CC.hudControl.visualAlert, flush=True)

    print("STARTING if not self.read_only:", flush=True)
    if not self.read_only:
      # send car controls over can
      print("STARTING can_sends = self.CI.apply(CC)", flush=True)
      can_sends = self.CI.apply(CC)
      print("STARTING can_sends = self.CI.apply(CC) DUMP", can_sends, flush=True)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

    print("STARTING force_decel", flush=True)
    force_decel = (self.sm['dMonitoringState'].awarenessStatus < 0.) or \
                  (self.state == State.softDisabling)
    print("STARTING force_decel", force_decel, flush=True)

    print("STARTING steer_angle_rad", flush=True)
    steer_angle_rad = (CS.steeringAngle - self.sm['pathPlan'].angleOffset) * CV.DEG_TO_RAD
    print("STARTING steer_angle_rad DUMP", steer_angle_rad, flush=True)

    # controlsState
    dat = messaging.new_message('controlsState')
    print("STARTING dat DUMP", dat, flush=True)
    dat.valid = CS.canValid
    print("STARTING dat.valid DUMP", dat.valid, flush=True)
    controlsState = dat.controlsState
    print("STARTING controlsState DUMP", controlsState, flush=True)
    controlsState.alertText1 = self.AM.alert_text_1
    print("STARTING controlsState.alertText1 DUMP", controlsState.alertText1, flush=True)
    controlsState.alertText2 = self.AM.alert_text_2
    print("STARTING controlsState.alertText2 DUMP", controlsState.alertText2, flush=True)
    controlsState.alertSize = self.AM.alert_size
    print("STARTING controlsState.alertSize DUMP", controlsState.alertSize, flush=True)
    controlsState.alertStatus = self.AM.alert_status
    print("STARTING controlsState.alertStatus DUMP", controlsState.alertStatus, flush=True)
    controlsState.alertBlinkingRate = self.AM.alert_rate
    print("STARTING controlsState.alertBlinkingRate DUMP", controlsState.alertBlinkingRate, flush=True)
    controlsState.alertType = self.AM.alert_type
    print("STARTING controlsState.alertType DUMP", controlsState.alertType, flush=True)
    controlsState.alertSound = self.AM.audible_alert
    print("STARTING controlsState.alertSound DUMP", controlsState.alertSound, flush=True)
    controlsState.driverMonitoringOn = self.sm['dMonitoringState'].faceDetected
    print("STARTING controlsState.driverMonitoringOn DUMP", controlsState.driverMonitoringOn, flush=True)
    controlsState.canMonoTimes = list(CS.canMonoTimes)
    print("STARTING controlsState.canMonoTimes DUMP", controlsState.canMonoTimes, flush=True)
    controlsState.planMonoTime = self.sm.logMonoTime['plan']
    print("STARTING controlsState.planMonoTime DUMP", controlsState.planMonoTime, flush=True)
    controlsState.pathPlanMonoTime = self.sm.logMonoTime['pathPlan']
    print("STARTING controlsState.pathPlanMonoTime DUMP", controlsState.pathPlanMonoTime, flush=True)
    controlsState.enabled = self.enabled
    print("STARTING controlsState.enabled DUMP", controlsState.enabled, flush=True)
    controlsState.active = self.active
    print("STARTING controlsState.active DUMP", controlsState.active, flush=True)
    controlsState.vEgo = CS.vEgo
    print("STARTING controlsState.vEgo DUMP", controlsState.vEgo, flush=True)
    controlsState.vEgoRaw = CS.vEgoRaw
    print("STARTING controlsState.vEgoRaw DUMP", controlsState.vEgoRaw, flush=True)
    controlsState.angleSteers = CS.steeringAngle
    print("STARTING controlsState.angleSteers DUMP", controlsState.angleSteers, flush=True)
    controlsState.curvature = self.VM.calc_curvature(steer_angle_rad, CS.vEgo)
    print("STARTING controlsState.curvature DUMP", controlsState.curvature, flush=True)
    controlsState.steerOverride = CS.steeringPressed
    print("STARTING controlsState.steerOverride DUMP", controlsState.steerOverride, flush=True)
    controlsState.state = self.state
    print("STARTING controlsState.state DUMP", controlsState.state, flush=True)
    controlsState.engageable = not self.events.any(ET.NO_ENTRY)
    print("STARTING controlsState.engageable DUMP", controlsState.engageable, flush=True)
    controlsState.longControlState = self.LoC.long_control_state
    print("STARTING controlsState.longControlState DUMP", controlsState.longControlState, flush=True)
    controlsState.vPid = float(self.LoC.v_pid)
    print("STARTING controlsState.vPid DUMP", controlsState.vPid, flush=True)
    controlsState.vCruise = float(self.v_cruise_kph)
    print("STARTING controlsState.vCruise DUMP", controlsState.vCruise, flush=True)
    controlsState.upAccelCmd = float(self.LoC.pid.p)
    print("STARTING controlsState.upAccelCmd DUMP", controlsState.upAccelCmd, flush=True)
    controlsState.uiAccelCmd = float(self.LoC.pid.i)
    print("STARTING controlsState.uiAccelCmd DUMP", controlsState.uiAccelCmd, flush=True)
    controlsState.ufAccelCmd = float(self.LoC.pid.f)
    print("STARTING controlsState.ufAccelCmd DUMP", controlsState.ufAccelCmd, flush=True)
    controlsState.angleSteersDes = float(self.LaC.angle_steers_des)
    print("STARTING controlsState.angleSteersDes DUMP", controlsState.angleSteersDes, flush=True)
    controlsState.vTargetLead = float(v_acc)
    print("STARTING controlsState.vTargetLead DUMP", controlsState.vTargetLead, flush=True)
    controlsState.aTarget = float(a_acc)
    print("STARTING controlsState.aTarget DUMP", controlsState.aTarget, flush=True)
    controlsState.jerkFactor = float(self.sm['plan'].jerkFactor)
    print("STARTING controlsState.jerkFactor DUMP", controlsState.jerkFactor, flush=True)
    controlsState.gpsPlannerActive = self.sm['plan'].gpsPlannerActive
    print("STARTING controlsState.gpsPlannerActive DUMP", controlsState.gpsPlannerActive, flush=True)
    controlsState.vCurvature = self.sm['plan'].vCurvature
    print("STARTING controlsState.vCurvature DUMP", controlsState.vCurvature, flush=True)
    controlsState.decelForModel = self.sm['plan'].longitudinalPlanSource == LongitudinalPlanSource.model
    print("STARTING controlsState.decelForModel DUMP", controlsState.decelForModel, flush=True)
    controlsState.cumLagMs = -self.rk.remaining * 1000.
    print("STARTING controlsState.cumLagMs DUMP", controlsState.cumLagMs, flush=True)
    controlsState.startMonoTime = int(start_time * 1e9)
    print("STARTING controlsState.startMonoTime DUMP", controlsState.startMonoTime, flush=True)
    controlsState.mapValid = self.sm['plan'].mapValid
    print("STARTING controlsState.mapValid DUMP", controlsState.mapValid, flush=True)
    controlsState.forceDecel = bool(force_decel)
    print("STARTING controlsState.forceDecel DUMP", controlsState.forceDecel, flush=True)
    controlsState.canErrorCounter = self.can_error_counter
    print("STARTING controlsState.canErrorCounter DUMP", controlsState.canErrorCounter, flush=True)

    print("STARTING if self.CP.lateralTuning.which() == 'pid':", flush=True)
    if self.CP.lateralTuning.which() == 'pid':
      controlsState.lateralControlState.pidState = lac_log
      print("STARTING controlsState.lateralControlState.pidState = lac_log", flush=True)
    print("STARTING elif self.CP.lateralTuning.which() == 'lqr':", flush=True)
    elif self.CP.lateralTuning.which() == 'lqr':
      controlsState.lateralControlState.lqrState = lac_log
      print("STARTING controlsState.lateralControlState.lqrState = lac_log", flush=True)
    print("STARTING elif self.CP.lateralTuning.which() == 'indi':", flush=True)
    elif self.CP.lateralTuning.which() == 'indi':
      controlsState.lateralControlState.indiState = lac_log
      print("STARTING controlsState.lateralControlState.indiState = lac_log", flush=True)
    self.pm.send('controlsState', dat)
    print("STARTING self.pm.send('controlsState', dat)", flush=True)

    # carState
    print("STARTING # carState", flush=True)
    car_events = self.events.to_msg()
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.events = car_events
    self.pm.send('carState', cs_send)
    print("STARTING # carState", car_events, cs_send, cs_send.valid, cs_send.carState, cs_send.carState.events, flush=True)

    # carEvents - logged every second or on change
    print("STARTING # carEvents - logged every second or on change", flush=True)
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('carEvents', len(self.events))
      ce_send.carEvents = car_events
      self.pm.send('carEvents', ce_send)
      print("STARTING # carEvents - logged every second or on change DUMP", ce_send, ce_send.carEvents, flush=True)
    self.events_prev = self.events.names.copy()
    print("STARTING self.events_prev DUMP", self.events_prev, flush=True)

    # carParams - logged every 50 seconds (> 1 per segment)
    print("STARTING if (self.sm.frame % int(50. / DT_CTRL) == 0):", flush=True)
    if (self.sm.frame % int(50. / DT_CTRL) == 0):
      cp_send = messaging.new_message('carParams')
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)
      print("STARTING if (self.sm.frame % int(50. / DT_CTRL) == 0): DUMP", cp_send, cp_send.carParams, flush=True)

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)
    print("STARTING cc_send, cc_send.valid, cc_send.carControl DUMP", cc_send, cc_send.valid, cc_send.carControl, flush=True)

    # copy CarControl to pass to CarInterface on the next iteration
    self.CC = CC
    print("STARTING self.CC DUMP", self.CC, flush=True)

  print("STARTING def step(self):", flush=True)
  def step(self):
    start_time = sec_since_boot()
    self.prof.checkpoint("Ratekeeper", ignore=True)
    print("STARTING start_time DUMP", start_time, flush=True)

    # Sample data from sockets and get a carState
    CS = self.data_sample()
    self.prof.checkpoint("Sample")
    print("STARTING CS DUMP", CS, flush=True)

    self.update_events(CS)

    print("STARTING if not self.read_only:", flush=True)
    if not self.read_only:
      # Update control state
      self.state_transition(CS)
      self.prof.checkpoint("State transition")
      print("STARTING if not self.read_only: if command works", flush=True)

    # Compute actuators (runs PID loops and lateral MPC)
    actuators, v_acc, a_acc, lac_log = self.state_control(CS)
    print("STARTING actuators, v_acc, a_acc, lac_log = self.state_control(CS) DUMP", actuators, v_acc, a_acc, lac_log, flush=True)

    self.prof.checkpoint("State Control")

    # Publish data
    self.publish_logs(CS, start_time, actuators, v_acc, a_acc, lac_log)
    self.prof.checkpoint("Sent")

  print("STARTING def controlsd_thread(self):", flush=True)
  def controlsd_thread(self):
    print("STARTING while True", flush=True)
    while True:
      self.step()
      self.rk.monitor_time()
      self.prof.display()
      print("STARTING while True works", flush=True)

def main(sm=None, pm=None, logcan=None):
  controls = Controls(sm, pm, logcan)
  controls.controlsd_thread()
  print("STARTING controls DUMP", controls, flush=True)


if __name__ == "__main__":
  main()
