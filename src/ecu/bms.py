"""
Battery Management System (BMS) ECU.

Publishes battery state (SOC, voltage, current, temperature) and enforces
protection limits with dynamic current derating based on SOC.

Pre-charge FSM:
  INIT → PRECHARGE: close pre-charge relay (series resistor path)
  PRECHARGE → ACTIVE: V_dc ≥ 90% V_pack within timeout → close main contactor
  PRECHARGE → FAULT: timeout exceeded → DTC P0AA6
"""

import time

from src.ecu.base_ecu import BaseECU, ECUState


class BMS(BaseECU):
    """Battery Management System ECU."""

    ECU_NAME  = "BMS"
    CYCLE_TIME = 0.100   # 100ms

    def __init__(self, can_bus, physics_state, cfg):
        """
        Initialize BMS ECU.

        Args:
            can_bus: CANBus instance for publishing messages
            physics_state: PhysicsState instance for reading battery state
            cfg: AppConfig instance for accessing battery protection limits
        """
        super().__init__(can_bus, physics_state, cfg)

        # Store config shortcuts
        self._prot = cfg.battery.protection
        self._pack = cfg.battery.pack

        # Pre-charge FSM tracking
        self._precharge_start: float = 0.0

        # Instance variables for telemetry
        self._last_soc_pct = 0.0
        self._last_v_terminal = 0.0
        self._last_i_pack = 0.0
        self._last_t_battery = 0.0
        self._last_fault_flags = 0
        self._last_drive_permission = 0
        self._last_max_discharge = 0.0
        self._last_max_charge = 0.0
        self._last_v_dc_link = 0.0

    async def _tick(self) -> None:
        """
        Execute one BMS cycle.

        1. INIT → PRECHARGE: close pre-charge relay, record start time
        2. PRECHARGE: poll V_dc vs V_pack; complete or timeout
        3. ACTIVE / FAULT / SAFE_STATE: normal protection logic
        """
        # ── Pre-charge state machine ───────────────────────────────────────
        if self.fsm_state == ECUState.INIT:
            self.fsm_state = ECUState.PRECHARGE
            self._physics.precharge_relay = True
            self._physics.main_contactor  = False
            self._precharge_start = time.monotonic()

        if self.fsm_state == ECUState.PRECHARGE:
            v_dc    = self._physics.v_dc_link
            v_pack  = self._physics.v_terminal
            elapsed = time.monotonic() - self._precharge_start

            if v_dc >= 0.90 * v_pack:
                # Success: open pre-charge relay, close main contactor
                self._physics.precharge_relay = False
                self._physics.main_contactor  = True
                self.fsm_state = ECUState.ACTIVE
                # Fall through to normal protection logic on the same tick
            elif elapsed > self._prot.precharge_timeout_s:
                # Timeout: open everything, raise fault DTC P0AA6
                self._physics.precharge_relay = False
                self._physics.main_contactor  = False
                self._dtcs.add("P0AA6")
                self.fsm_state = ECUState.FAULT
            else:
                # Still charging: publish drive-inhibited status and return
                soc = self._physics.soc * 100.0
                v   = v_pack
                t   = self._physics.t_battery
                self._last_soc_pct         = soc
                self._last_v_terminal      = v
                self._last_i_pack          = 0.0
                self._last_t_battery       = t
                self._last_fault_flags     = 0
                self._last_drive_permission = 0
                self._last_max_discharge   = self._prot.max_discharge_current_a
                self._last_max_charge      = self._prot.max_charge_current_a
                self._last_v_dc_link       = v_dc
                self._bus.publish("BMS_STATUS", {
                    "BMS_SOC": soc, "BMS_PackVoltage": v, "BMS_PackCurrent": 0.0,
                    "BMS_PackTemp": t, "BMS_FaultFlags": 0.0, "BMS_DrivePermission": 0.0,
                })
                self._bus.publish("BMS_LIMITS", {
                    "BMS_MaxDischargeCurrent": self._prot.max_discharge_current_a,
                    "BMS_MaxChargeCurrent":    self._prot.max_charge_current_a,
                })
                return

        # ── Normal protection logic (ACTIVE / FAULT / SAFE_STATE) ─────────

        # Read from physics state
        s = self._physics
        soc_pct    = s.soc * 100.0
        v_terminal = s.v_terminal
        i_pack     = s.i_pack
        t_battery  = s.t_battery

        # Store for telemetry
        self._last_soc_pct    = soc_pct
        self._last_v_terminal = v_terminal
        self._last_i_pack     = i_pack
        self._last_t_battery  = t_battery
        self._last_v_dc_link  = s.v_dc_link

        # Compute cell voltage
        cells_series = self._pack.cells_series
        v_cell = v_terminal / cells_series if cells_series > 0 else 0.0

        # Compute FaultFlags bitmask (4 bits)
        # Bit 0 (LSB) = OVP: v_cell > max_cell_voltage_v
        # Bit 1       = OTP: t_battery > max_temp_c
        # Bit 2       = UVP: v_cell < min_cell_voltage_v
        # Bit 3       = OCP: i_pack > max_discharge_current_a
        fault_flags = 0

        if v_cell > self._prot.max_cell_voltage_v:
            fault_flags |= 0x01  # OVP

        if t_battery > self._prot.max_temp_c:
            fault_flags |= 0x02  # OTP
            self._dtcs.add("P0A1F")

        if v_cell < self._prot.min_cell_voltage_v:
            fault_flags |= 0x04  # UVP
            self._dtcs.add("P0A7F")

        if i_pack > self._prot.max_discharge_current_a:
            fault_flags |= 0x08  # OCP
            self._dtcs.add("P0A0D")

        self._last_fault_flags = fault_flags

        # FSM transition: FAULT if any fault flag set, else ACTIVE
        # (unless already in SAFE_STATE from watchdog)
        if self.fsm_state != ECUState.SAFE_STATE:
            self.fsm_state = ECUState.FAULT if fault_flags > 0 else ECUState.ACTIVE

        # DrivePermission: 1 only when fault_flags == 0
        drive_permission = 1 if fault_flags == 0 else 0
        self._last_drive_permission = drive_permission

        # Dynamic current limits (derating)
        # MaxDischargeCurrent: linear ramp from 200A (SOC≥20%) to 0A (SOC≤5%)
        if soc_pct >= 20.0:
            max_discharge = self._prot.max_discharge_current_a
        elif soc_pct <= 5.0:
            max_discharge = 0.0
        else:
            max_discharge = (
                self._prot.max_discharge_current_a * (soc_pct - 5.0) / 15.0
            )

        # MaxChargeCurrent: linear ramp from 30A (SOC≤80%) to 0A (SOC≥98%)
        if soc_pct <= 80.0:
            max_charge = self._prot.max_charge_current_a
        elif soc_pct >= 98.0:
            max_charge = 0.0
        else:
            max_charge = (
                self._prot.max_charge_current_a * (98.0 - soc_pct) / 18.0
            )

        self._last_max_discharge = max_discharge
        self._last_max_charge = max_charge

        # Publish BMS_STATUS (0x110)
        self._bus.publish("BMS_STATUS", {
            "BMS_SOC":              soc_pct,
            "BMS_PackVoltage":      v_terminal,
            "BMS_PackCurrent":      i_pack,
            "BMS_PackTemp":         t_battery,
            "BMS_FaultFlags":       float(fault_flags),
            "BMS_DrivePermission":  float(drive_permission),
        })

        # Publish BMS_LIMITS (0x111)
        self._bus.publish("BMS_LIMITS", {
            "BMS_MaxDischargeCurrent": max_discharge,
            "BMS_MaxChargeCurrent":    max_charge,
        })

    def get_telemetry(self) -> dict:
        """Return last computed telemetry values."""
        return {
            "soc":              self._last_soc_pct,
            "voltage":          self._last_v_terminal,
            "current":          self._last_i_pack,
            "temp":             self._last_t_battery,
            "fault_flags":      self._last_fault_flags,
            "drive_permission": bool(self._last_drive_permission),
            "max_discharge_a":  self._last_max_discharge,
            "max_charge_a":     self._last_max_charge,
            "state":            self.fsm_state.name,
            "v_dc_link":        self._last_v_dc_link,
            "precharge_relay":  self._physics.precharge_relay,
            "main_contactor":   self._physics.main_contactor,
        }
