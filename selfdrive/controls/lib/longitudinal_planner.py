#!/usr/bin/env python3
import math
import numpy as np
from common.numpy_fast import clip, interp

import cereal.messaging as messaging
from common.conversions import Conversions as CV
from common.filter_simple import FirstOrderFilter
from common.realtime import DT_MDL
from selfdrive.modeld.constants import T_IDXS
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, N
from selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from selfdrive.swaglog import cloudlog
from selfdrive.controls.lib.vision_turn_controller import VisionTurnController
from common.params import Params
from selfdrive.controls.lib.events import Events

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -1.2
A_CRUISE_MIN_VALS = [-0.75, -0.77, -0.84, -0.95, -0.80, -0.70]
A_CRUISE_MIN_BP = [0., 30 * CV.KPH_TO_MS, 50 * CV.KPH_TO_MS, 70 * CV.KPH_TO_MS, 110 * CV.KPH_TO_MS, 130 * CV.KPH_TO_MS]
A_CRUISE_MAX_VALS = [2.2, 2.0, 1.5, 1.1, .65, .5,  .4,  0.3, 0.25, 0.09]
A_CRUISE_MAX_BP = [0., 10 * CV.KPH_TO_MS, 20 * CV.KPH_TO_MS, 30 * CV.KPH_TO_MS, 40 * CV.KPH_TO_MS, 50 * CV.KPH_TO_MS, 70 * CV.KPH_TO_MS, 90 * CV.KPH_TO_MS, 110 * CV.KPH_TO_MS, 130 * CV.KPH_TO_MS]


# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_min_accel(v_ego):
  return interp(v_ego, A_CRUISE_MIN_BP, A_CRUISE_MIN_VALS)

def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class Planner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.use_cluster_speed = Params().get_bool('UseClusterSpeed')
    self.cruise_source = 'cruise'
    self.vision_turn_controller = VisionTurnController(CP)
    self.events = Events()
    
    self.params_count = 0
    self.cruiseMaxVals1 = float(int(Params().get("CruiseMaxVals1", encoding="utf8"))) / 100.
    self.cruiseMaxVals2 = float(int(Params().get("CruiseMaxVals2", encoding="utf8"))) / 100.
    self.cruiseMaxVals3 = float(int(Params().get("CruiseMaxVals3", encoding="utf8"))) / 100.
    self.cruiseMaxVals4 = float(int(Params().get("CruiseMaxVals4", encoding="utf8"))) / 100.
    self.cruiseMaxVals5 = float(int(Params().get("CruiseMaxVals5", encoding="utf8"))) / 100.
    self.cruiseMaxVals6 = float(int(Params().get("CruiseMaxVals6", encoding="utf8"))) / 100.
    self.cruiseMaxVals7 = float(int(Params().get("CruiseMaxVals1", encoding="utf8"))) / 100.
    self.cruiseMaxVals8 = float(int(Params().get("CruiseMaxVals2", encoding="utf8"))) / 100.
    self.cruiseMaxVals9 = float(int(Params().get("CruiseMaxVals3", encoding="utf8"))) / 100.
    self.cruiseMaxVals10 = float(int(Params().get("CruiseMaxVals4", encoding="utf8"))) / 100.

  def update_params(self):
    self.params_count = (self.params_count + 1) % 200
    if self.params_count == 50:
      self.cruiseMaxVals1 = float(int(Params().get("CruiseMaxVals1", encoding="utf8"))) / 100.
      self.cruiseMaxVals2 = float(int(Params().get("CruiseMaxVals2", encoding="utf8"))) / 100.
    elif self.params_count == 100:
      self.cruiseMaxVals3 = float(int(Params().get("CruiseMaxVals1", encoding="utf8"))) / 100.
      self.cruiseMaxVals4 = float(int(Params().get("CruiseMaxVals2", encoding="utf8"))) / 100.
    elif self.params_count == 130:
      self.cruiseMaxVals5 = float(int(Params().get("CruiseMaxVals3", encoding="utf8"))) / 100.
      self.cruiseMaxVals6 = float(int(Params().get("CruiseMaxVals4", encoding="utf8"))) / 100.
      self.cruiseMaxVals7 = float(int(Params().get("CruiseMaxVals1", encoding="utf8"))) / 100.
    elif self.params_count == 150:
      self.cruiseMaxVals8 = float(int(Params().get("CruiseMaxVals5", encoding="utf8"))) / 100.
      self.cruiseMaxVals9 = float(int(Params().get("CruiseMaxVals6", encoding="utf8"))) / 100.
      self.cruiseMaxVals10 = float(int(Params().get("CruiseMaxVals1", encoding="utf8"))) / 100.

  def get_max_accel(self, v_ego):
    cruiseMaxVals = [self.cruiseMaxVals1, self.cruiseMaxVals2, self.cruiseMaxVals3, self.cruiseMaxVals4, self.cruiseMaxVals5, self.cruiseMaxVals6, self.cruiseMaxVals7, self.cruiseMaxVals8, self.cruiseMaxVals9, self.cruiseMaxVals10]
    return interp(v_ego, A_CRUISE_MAX_BP, cruiseMaxVals)
    
  def update(self, sm):
    self.update_params()
    
    v_ego = sm['carState'].vEgo
    v_cruise_kph = sm['controlsState'].vCruise
    v_cruise_kph = min(v_cruise_kph, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    # neokii
    if not self.use_cluster_speed:
      vCluRatio = sm['carState'].vCluRatio
      if vCluRatio > 0.5:
        v_cruise *= vCluRatio
        v_cruise = int(v_cruise * CV.MS_TO_KPH + 0.25) * CV.KPH_TO_MS

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not sm['carState'].standstill
    
    accel_limits = [get_min_accel(v_ego), self.get_max_accel(v_ego)]
    accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = clip(sm['carState'].aEgo, *accel_limits)
      self.mpc.prev_a = np.full(N+1, self.a_desired)

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    
    # Get acceleration and active solutions for custom long mpc.
    self.cruise_source, a_min_sol, v_cruise_sol = self.cruise_solutions(not reset_state, self.v_desired_filter.x,
                                                                        self.a_desired, v_cruise, sm)

    if force_slow_decel:
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05, a_min_sol)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['carState'], sm['radarState'], v_cruise_sol, prev_accel_constraint)


    self.v_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 5 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(DT_MDL, T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source if self.mpc.source != 'cruise' else self.cruise_source
    longitudinalPlan.visionTurnControllerState = self.vision_turn_controller.state
    longitudinalPlan.visionTurnSpeed = float(self.vision_turn_controller.v_turn)
    longitudinalPlan.eventsDEPRECATED = self.events.to_msg()
    
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    pm.send('longitudinalPlan', plan_send)
    
  def cruise_solutions(self, enabled, v_ego, a_ego, v_cruise, sm):
    # Update controllers
    self.vision_turn_controller.update(enabled, v_ego, a_ego, v_cruise, sm)
    self.events = Events()

    # Pick solution with lowest velocity target.
    a_solutions = {'cruise': float("inf")}
    v_solutions = {'cruise': v_cruise}

    if self.vision_turn_controller.is_active:
      a_solutions['turn'] = self.vision_turn_controller.a_target
      v_solutions['turn'] = self.vision_turn_controller.v_turn

    source = min(v_solutions, key=v_solutions.get)

    return source, a_solutions[source], v_solutions[source]
