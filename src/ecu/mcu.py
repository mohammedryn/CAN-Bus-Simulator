"""
Motor Control Unit (MCU) ECU.

Executes motor torque commands with safety limits: overspeed protection,
thermal derating, power-limited torque, and regenerative braking current
clamping to BMS charge current limits.
"""

import math

from src.ecu.base_ecu import BaseECU, ECUState, WatchdogTimer


class MCU(BaseECU):
    """Motor Control Unit ECU."""

    ECU_NAME = "MCU"
    CYCLE_TIME = 0.050   # 50ms

    def __init__(self, can_bus, physics_state, cfg):
        """
        Initialize MCU ECU.

        Args:
            can_bus: CANBus instance for publishing messages
            physics_state: PhysicsState instance for reading/writing motor state
            cfg: AppConfig instance for accessing motor configuration
        """
        super().__init__(can_bus, physics_state, cfg)

        # Subscribe to VCU_COMMAND (0x120) and BMS_LIMITS (0x111)
        self.subscribe_to(0x120, 0x111)

        # Register watchdog for VCU_COMMAND
        self._watchdogs = {
            0x120: WatchdogTimer(
                cfg.can.watchdogs["mcu_vcu_command"].timeout_ms / 1000.0,
                cfg.can.watchdogs["mcu_vcu_command"].dtc,
            ),
        }

        # Cached state from received frames
        self._vcu_torque_request = 0.0   # Nm, from VCU_COMMAND
        self._vcu_brake_request = 0.0    # %, from VCU_COMMAND
        self._bms_max_charge_current    = cfg.battery.protection.max_charge_current_a     # A
        self._bms_max_discharge_current = cfg.battery.protection.max_discharge_current_a  # A

        # Config shortcuts
        self._motor_cfg = cfg.vehicle.motor

    def _on_frame(self, frame) -> None:
        """
        Handle received CAN frames.

        Updates cached state from VCU_COMMAND and BMS_LIMITS.
        """
        if frame.message_name == "VCU_COMMAND":
            self._vcu_torque_request = frame.parsed.get("VCU_TorqueRequest", 0.0)
            self._vcu_brake_request = frame.parsed.get("VCU_BrakeRequest", 0.0)
        elif frame.message_name == "BMS_LIMITS":
            self._bms_max_charge_current    = frame.parsed.get("BMS_MaxChargeCurrent", 0.0)
            self._bms_max_discharge_current = frame.parsed.get("BMS_MaxDischargeCurrent", self._bms_max_discharge_current)

    async def _tick(self) -> None:
        """
        Execute one MCU cycle.

        1. Transition INIT → ACTIVE on first tick
        2. Guard: if SAFE_STATE, publish zero torque and return
        3. Read physics state (motor_rpm, motor_temp, v_terminal)
        4. Overspeed protection (> 1.05 * max_rpm → P0C70, SAFE_STATE)
        5. Thermal derating (120°C to 200°C ramp)
        6. Add DTC P0C41 if motor_temp > 160°C
        7. Compute torque-speed power limit
        8. CRITICAL: Regen current clamp (Blocker/Architecture Rule 8)
        9. Compute actual torque (limited by t_max)
        10. Write torque_cmd to physics state (MCU is the ONLY writer)
        11. Publish MCU_STATUS message
        """
        # FSM transition INIT → ACTIVE
        if self.fsm_state == ECUState.INIT:
            self.fsm_state = ECUState.ACTIVE

        # SAFE_STATE guard: publish zero torque and return
        if self.fsm_state == ECUState.SAFE_STATE:
            self._physics.torque_cmd = 0.0
            self._bus.publish("MCU_STATUS", {
                "MCU_ActualTorque": 0.0,
                "MCU_MotorRPM": self._physics.motor_rpm,
                "MCU_MotorTemp": self._physics.motor_temp,
                "MCU_InverterTemp": self._physics.inverter_temp,
                "MCU_ActiveState": float(self.fsm_state),
            })
            return

        # Read physics state
        motor_rpm = self._physics.motor_rpm
        motor_temp = self._physics.motor_temp
        v_terminal = self._physics.v_terminal

        # Overspeed protection: > 1.05 * max_rpm
        max_rpm = self._motor_cfg.max_rpm
        if abs(motor_rpm) > max_rpm * 1.05:
            self._dtcs.add("P0C70")
            self.fsm_state = ECUState.SAFE_STATE
            self._physics.torque_cmd = 0.0
            self._bus.publish("MCU_STATUS", {
                "MCU_ActualTorque": 0.0,
                "MCU_MotorRPM": motor_rpm,
                "MCU_MotorTemp": motor_temp,
                "MCU_InverterTemp": self._physics.inverter_temp,
                "MCU_ActiveState": float(self.fsm_state),
            })
            return

        # Thermal derating factor (120°C to 200°C)
        derating_start = self._motor_cfg.derating_start_c  # 120°C
        derating_end = self._motor_cfg.derating_end_c      # 200°C
        if motor_temp >= derating_end:
            derate = 0.0
        elif motor_temp >= derating_start:
            derate = max(0.0, 1.0 - (motor_temp - derating_start) / (derating_end - derating_start))
        else:
            derate = 1.0

        # Add DTC if overtemp (160°C threshold)
        if motor_temp > 160.0:
            self._dtcs.add("P0C41")

        # Torque-speed power limit
        omega = max(abs(motor_rpm) * math.pi / 30.0, 1.0)   # rad/s, avoid /0
        t_max = min(self._motor_cfg.peak_torque_nm, self._motor_cfg.peak_power_w / omega) * derate

        torque_req = self._vcu_torque_request   # may be negative (regen)
        eff = self._motor_cfg.efficiency  # 0.92

        if v_terminal > 0:
            if torque_req < 0:
                # Regen: clamp charge current to BMS_MaxChargeCurrent
                i_regen = abs(torque_req) * omega * eff / v_terminal
                if i_regen > self._bms_max_charge_current:
                    torque_req = -(self._bms_max_charge_current * v_terminal) / (omega * eff)
            elif torque_req > 0:
                # Drive: clamp discharge current to BMS_MaxDischargeCurrent
                i_drive = torque_req * omega / (eff * v_terminal)
                if i_drive > self._bms_max_discharge_current:
                    torque_req = (self._bms_max_discharge_current * v_terminal * eff) / omega

        # Actual torque (clipped by t_max)
        actual_torque = max(-t_max, min(t_max, torque_req))

        # Write to physics state (MCU is the ONLY writer of torque_cmd)
        self._physics.torque_cmd = actual_torque

        # Publish MCU_STATUS (0x130)
        self._bus.publish("MCU_STATUS", {
            "MCU_ActualTorque": actual_torque,
            "MCU_MotorRPM": motor_rpm,
            "MCU_MotorTemp": motor_temp,
            "MCU_InverterTemp": self._physics.inverter_temp,
            "MCU_ActiveState": float(self.fsm_state),
        })

    def get_telemetry(self) -> dict:
        """
        Return telemetry data for debugging/monitoring.

        Returns:
            dict with keys: actual_torque, motor_rpm, motor_temp, inverter_temp, state
        """
        return {
            "actual_torque": self._physics.torque_cmd,
            "motor_rpm": self._physics.motor_rpm,
            "motor_temp": self._physics.motor_temp,
            "inverter_temp": self._physics.inverter_temp,
            "state": self.fsm_state.name,
        }
