"""
Physics engine for EV CAN simulation.

Contains four model classes and the PhysicsEngine coroutine:
  - PhysicsState      : simulation state dataclass
  - BatteryModel1RC   : 1RC Thevenin ECM
  - MotorModel        : motor/inverter thermal + torque follower
  - VehicleModel      : longitudinal vehicle dynamics
  - PhysicsEngine     : async coroutine that ties everything together
"""

import asyncio
import dataclasses
import math
import queue

import numpy as np

from src.config_loader import AppConfig

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
T_AMB = 25.0          # ambient temperature, °C (hard-coded per spec)
NOMINAL_STEP_S = 0.010  # 10 ms nominal integration step


# ---------------------------------------------------------------------------
# PhysicsState
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class PhysicsState:
    soc:              float = 0.80   # 0.0–1.0 (fraction)
    v_terminal:       float = 37.0   # V
    i_pack:           float = 0.0    # A (positive = discharge)
    t_battery:        float = 25.0   # °C
    v_rc:             float = 0.0    # RC branch voltage V
    motor_rpm:        float = 0.0    # RPM (can be negative)
    motor_temp:       float = 25.0   # °C
    inverter_temp:    float = 25.0   # °C
    vehicle_speed_ms: float = 0.0    # m/s
    torque_cmd:       float = 0.0    # Nm (written by MCU)
    throttle_pct:     float = 0.0    # 0–100
    brake_pct:        float = 0.0    # 0–100
    # DC link contactor state — written by BMS FSM
    v_dc_link:        float = 0.0    # capacitor voltage (V), rises during pre-charge
    precharge_relay:  bool  = False  # pre-charge relay (series resistor path)
    main_contactor:   bool  = False  # main positive contactor (direct path)


# ---------------------------------------------------------------------------
# BatteryModel1RC  — 1st-order Thevenin ECM
# ---------------------------------------------------------------------------

class BatteryModel1RC:
    """1RC Thevenin Equivalent Circuit Model for the battery pack."""

    def __init__(self, cfg: AppConfig):
        pack   = cfg.battery.pack
        ecm    = cfg.battery.ecm_1rc
        therm  = cfg.battery.thermal

        # Pack topology
        self._cells_series   = pack.cells_series
        self._cells_parallel = pack.cells_parallel
        self._q_pack_ah      = pack.cell_capacity_ah * pack.cells_parallel  # total Ah

        # ECM parameters
        self._r0_ref     = ecm.r0_ohm
        self._r1_ref     = ecm.r1_ohm
        self._c1         = ecm.c1_farad
        self._temp_coeff = ecm.temp_coeff_per_c

        # Thermal parameters
        self._thermal_mass      = therm.thermal_mass_j_per_k
        self._h_passive         = therm.passive_convection_w_per_k
        self._h_active          = therm.active_cooling_w_per_k
        self._active_threshold  = therm.active_cooling_threshold_c

        # OCV-SOC lookup table (cell voltage)
        table = np.array(cfg.battery.ocv_soc_table, dtype=float)
        self._ocv_soc_pts = table[:, 0]   # SOC fractions
        self._ocv_v_pts   = table[:, 1]   # cell voltages

        # State
        self._soc = 0.80    # fraction
        self._v_rc = 0.0    # RC branch voltage
        self._t   = T_AMB   # battery temperature °C

        # Startup assertion — Blocker 9 mitigation:
        # ensure nominal step << RC time constant / 10
        assert NOMINAL_STEP_S < (ecm.r1_ohm * ecm.c1_farad) / 10, (
            f"Nominal step {NOMINAL_STEP_S}s must be < RC/10 = "
            f"{ecm.r1_ohm * ecm.c1_farad / 10:.3f}s for numerical stability"
        )

    # ------------------------------------------------------------------
    def _ocv(self) -> float:
        """Open-circuit voltage of the full pack at current SOC."""
        cell_ocv = float(np.interp(self._soc, self._ocv_soc_pts, self._ocv_v_pts))
        return cell_ocv * self._cells_series

    def _r0(self) -> float:
        return self._r0_ref * math.exp(-self._temp_coeff * (self._t - T_AMB))

    def _r1(self) -> float:
        return self._r1_ref * math.exp(-self._temp_coeff * (self._t - T_AMB))

    # ------------------------------------------------------------------
    def step(self, i_pack: float, dt: float) -> dict:
        """
        Advance ECM by dt seconds with pack current i_pack (A, positive=discharge).

        Returns dict: {"soc": <0-100 %>, "voltage": V_terminal, "temperature": °C, "v_rc": V}
        """
        r0 = self._r0()
        r1 = self._r1()
        c1 = self._c1

        # --- RC branch update (exact exponential decay, unconditionally stable) ---
        # Forward Euler becomes unstable when dt >= 2*R1*C1 (happens at high T as R1→0).
        # Exact solution: V_RC(t+dt) = V_RC*exp(-dt/τ) + I*R1*(1 - exp(-dt/τ))
        tau = r1 * c1 if r1 * c1 > 1e-12 else 1e-12
        decay = math.exp(-dt / tau)
        v_rc_new = self._v_rc * decay + i_pack * r1 * (1.0 - decay)

        # --- Terminal voltage ---
        v_ocv      = self._ocv()
        v_terminal = v_ocv - i_pack * r0 - v_rc_new
        v_terminal = max(0.0, v_terminal)

        # --- SOC update ---
        soc_new = self._soc + dt * (-i_pack / (3600.0 * self._q_pack_ah))
        soc_new = max(0.0, min(1.0, soc_new))

        # --- Thermal model ---
        q_heat    = i_pack ** 2 * r0 + (v_rc_new ** 2 / r1 if r1 > 0 else 0.0)
        h_cooling = self._h_active if self._t > self._active_threshold else self._h_passive
        dt_temp   = dt * (q_heat - h_cooling * (self._t - T_AMB)) / self._thermal_mass

        # --- Commit state ---
        self._v_rc = v_rc_new
        self._soc  = soc_new
        self._t    = min(self._t + dt_temp, 600.0)  # cap at 600°C (well past any protection)

        return {
            "soc":         self._soc * 100.0,   # percentage (0-100)
            "voltage":     v_terminal,
            "temperature": self._t,
            "v_rc":        self._v_rc,
        }


# ---------------------------------------------------------------------------
# MotorModel  — torque follower + thermal model
# ---------------------------------------------------------------------------

class MotorModel:
    """Electric motor and inverter model: torque limiter + thermal dynamics."""

    def __init__(self, cfg: AppConfig):
        m = cfg.vehicle.motor
        self._peak_torque    = m.peak_torque_nm
        self._peak_power     = m.peak_power_w
        self._efficiency     = m.efficiency
        self._derating_start = m.derating_start_c
        self._derating_end   = m.derating_end_c

        # Motor thermal RC
        self._motor_R = m.motor_thermal_r_k_per_w
        self._motor_C = m.motor_thermal_c_j_per_k

        # Inverter thermal RC
        self._inv_R = m.inverter_thermal_r_k_per_w
        self._inv_C = m.inverter_thermal_c_j_per_k

        # State
        self.motor_temp    = T_AMB
        self.inverter_temp = T_AMB

    @property
    def eff(self) -> float:
        return self._efficiency

    def step(self, torque_cmd: float, rpm: float, dt: float) -> float:
        """
        Advance motor model by dt seconds.

        Returns actual torque executed (Nm, same sign as cmd).
        """
        omega = abs(rpm) * math.pi / 30.0   # rad/s

        # --- Power-limited max torque ---
        t_max = min(self._peak_torque, self._peak_power / max(omega, 1.0))

        # --- Thermal derating ---
        if self.motor_temp >= self._derating_end:
            derate = 0.0
        elif self.motor_temp >= self._derating_start:
            derate = max(
                0.0,
                1.0 - (self.motor_temp - self._derating_start)
                    / (self._derating_end - self._derating_start),
            )
        else:
            derate = 1.0
        t_max *= derate

        t_actual = max(-t_max, min(t_max, torque_cmd))

        # --- Loss calculations ---
        if t_actual > 0:
            p_loss_motor = t_actual * omega * (1.0 - self._efficiency)
        elif t_actual < 0:
            p_loss_motor = abs(t_actual) * omega * (1.0 - self._efficiency)
        else:
            p_loss_motor = 0.0
        p_loss_inverter = p_loss_motor * 0.3

        # --- Thermal update ---
        self.motor_temp += dt * (
            p_loss_motor / self._motor_C
            - (self.motor_temp - T_AMB) / (self._motor_R * self._motor_C)
        )
        self.inverter_temp += dt * (
            p_loss_inverter / self._inv_C
            - (self.inverter_temp - T_AMB) / (self._inv_R * self._inv_C)
        )

        return t_actual


# ---------------------------------------------------------------------------
# VehicleModel  — longitudinal dynamics
# ---------------------------------------------------------------------------

class VehicleModel:
    """Single-track longitudinal vehicle dynamics model."""

    def __init__(self, cfg: AppConfig):
        c = cfg.vehicle.chassis
        self._mass         = c.mass_kg
        self._wheel_radius = c.wheel_radius_m
        self._gear_ratio   = c.gear_ratio
        self._Cd           = c.drag_coefficient
        self._A            = c.frontal_area_m2
        self._Crr          = c.rolling_resistance
        self._rho          = c.air_density_kg_m3

        # State
        self.speed     = 0.0   # m/s
        self.motor_rpm = 0.0

    def step(self, t_actual: float, brake_pct: float, dt: float) -> None:
        """Update self.speed (m/s) and self.motor_rpm."""
        # Wheel torque → drive force
        t_wheel = t_actual * self._gear_ratio
        F_drive  = t_wheel / self._wheel_radius

        # Resistive forces
        F_brake = (brake_pct / 100.0) * self._mass * 9.81 * 0.3
        F_drag  = 0.5 * self._rho * self._Cd * self._A * self.speed ** 2
        F_roll  = (
            self._mass * 9.81 * self._Crr
            if self.speed > 0.01
            else 0.0
        )

        F_net = F_drive - F_brake - F_drag - F_roll
        a     = F_net / self._mass

        self.speed     = max(0.0, self.speed + dt * a)
        self.motor_rpm = (
            self.speed / self._wheel_radius * self._gear_ratio * 30.0 / math.pi
        )


# ---------------------------------------------------------------------------
# PhysicsEngine  — async coroutine that ties all models together
# ---------------------------------------------------------------------------

class PhysicsEngine:
    """
    Drift-free 10 ms physics coroutine.

    The public state is exposed via self.state (PhysicsState).
    Commands are read from a stdlib queue.Queue (thread-safe, no await).
    """

    NOMINAL_STEP_S = NOMINAL_STEP_S

    def __init__(self, cfg: AppConfig):
        self.state    = PhysicsState()
        self._battery = BatteryModel1RC(cfg)
        self._motor   = MotorModel(cfg)
        self._vehicle = VehicleModel(cfg)
        self._cfg     = cfg
        self._bus              = None   # set by run.py after bus is created
        self._thermal_runaway  = False  # set by drain_commands via fault injection

    # ------------------------------------------------------------------
    async def run(self, command_q: queue.Queue) -> None:
        """Drift-free 10 ms coroutine. Reads commands from stdlib queue (thread-safe)."""
        loop      = asyncio.get_event_loop()
        next_wake = loop.time()
        last_time = loop.time()
        while True:
            next_wake += self.NOMINAL_STEP_S
            now        = loop.time()
            dt         = now - last_time       # actual elapsed (correct on Windows)
            last_time  = now
            await asyncio.sleep(max(0, next_wake - now))
            self.drain_commands(command_q)
            self._step(dt if dt > 0 else self.NOMINAL_STEP_S)

    # ------------------------------------------------------------------
    def drain_commands(self, command_q: queue.Queue) -> None:
        """No await — GIL + cooperative scheduling makes writes atomic."""
        while True:
            try:
                cmd = command_q.get_nowait()
                match cmd.get("type"):
                    case "throttle":
                        self.state.throttle_pct = float(cmd["value"])
                    case "brake":
                        self.state.brake_pct = float(cmd["value"])
                    case "fault":
                        if cmd["fault"] == "wire_cut":
                            if self._bus is not None:
                                self._bus.fault_wire_cut = bool(cmd.get("active", True))
                        elif cmd["fault"] == "crc_corrupt":
                            if self._bus is not None:
                                self._bus.fault_crc_corrupt = bool(cmd.get("active", True))
                        elif cmd["fault"] == "thermal_runaway":
                            self._thermal_runaway = bool(cmd.get("active", True))
                        elif cmd["fault"] == "overspeed":
                            self.state.motor_rpm = (
                                self._cfg.vehicle.motor.max_rpm * 1.1
                            )
            except queue.Empty:
                break

    # ------------------------------------------------------------------
    def _step(self, dt: float) -> None:
        """No await — see Blocker 7 invariant."""
        s = self.state

        # Thermal runaway fault: inject 10°C/s continuously while active
        if self._thermal_runaway:
            self._battery._t = min(self._battery._t + 0.1, 600.0)

        # --- DC link pre-charge RC circuit ---
        # BMS FSM writes precharge_relay / main_contactor; physics integrates V_dc.
        # τ = R_precharge × C_dclink ≈ 0.5 s → 90% in ~1.15 s at 10 ms step.
        if s.main_contactor:
            s.v_dc_link = s.v_terminal                          # hard-connected
        elif s.precharge_relay:
            _prot = self._cfg.battery.protection
            tau = max(_prot.precharge_resistor_ohm * _prot.dc_link_capacitance_f, 1e-6)
            s.v_dc_link += dt * (s.v_terminal - s.v_dc_link) / tau
            s.v_dc_link = max(0.0, s.v_dc_link)
        else:
            s.v_dc_link = 0.0                                   # both relays open

        # --- Motor torque → vehicle dynamics ---
        t_actual            = self._motor.step(s.torque_cmd, s.motor_rpm, dt)
        self._vehicle.step(t_actual, s.brake_pct, dt)
        s.motor_rpm         = self._vehicle.motor_rpm
        s.vehicle_speed_ms  = self._vehicle.speed
        s.motor_temp        = self._motor.motor_temp
        s.inverter_temp     = self._motor.inverter_temp

        # --- Electrical demand from motor → battery current ---
        omega = abs(s.motor_rpm) * math.pi / 30.0
        if t_actual > 0:
            i_demand = (
                (t_actual * omega) / (s.v_terminal * self._motor.eff)
                if s.v_terminal > 0
                else 0.0
            )
        elif t_actual < 0:
            i_demand = (
                (t_actual * omega * self._motor.eff) / s.v_terminal
                if s.v_terminal > 0
                else 0.0
            )
        else:
            i_demand = 0.5   # quiescent draw

        # Hard clamp to battery protection limits — mirrors BMS hardware current limiter.
        # MCU already enforces this via torque scaling but physics runs at 10ms vs MCU 50ms,
        # so voltage sag between MCU ticks can still produce over-limit currents without this.
        _prot = self._cfg.battery.protection
        i_demand = max(-_prot.max_charge_current_a,
                       min(_prot.max_discharge_current_a, i_demand))

        # --- Battery ECM step ---
        result       = self._battery.step(i_demand, dt)
        s.soc        = result["soc"] / 100.0   # convert % → fraction
        s.v_terminal = result["voltage"]
        s.i_pack     = i_demand
        s.t_battery  = result["temperature"]
        s.v_rc       = result["v_rc"]
