"""
Vehicle Control Unit (VCU) ECU.

Processes throttle and brake inputs, enforces plausibility checks per ISO 26262 ASIL-B requirement,
applies torque slewing, and generates motor control commands with fault detection.
"""

from src.ecu.base_ecu import BaseECU, ECUState, WatchdogTimer


class VCU(BaseECU):
    """Vehicle Control Unit ECU."""

    ECU_NAME = "VCU"
    CYCLE_TIME = 0.050   # 50ms
    REGEN_MAX_NM = 100.0  # max regen torque magnitude (Nm)

    def __init__(self, can_bus, physics_state, cfg):
        """
        Initialize VCU ECU.

        Args:
            can_bus: CANBus instance for publishing messages
            physics_state: PhysicsState instance for reading throttle/brake
            cfg: AppConfig instance for accessing VCU and motor configuration
        """
        super().__init__(can_bus, physics_state, cfg)

        # Subscribe to BMS_STATUS (0x110) and MCU_STATUS (0x130)
        self.subscribe_to(0x110, 0x130)

        # Register watchdogs for BMS and MCU status
        self._watchdogs = {
            0x110: WatchdogTimer(
                cfg.can.watchdogs["vcu_bms_status"].timeout_ms / 1000.0,
                cfg.can.watchdogs["vcu_bms_status"].dtc,
            ),
            0x130: WatchdogTimer(
                cfg.can.watchdogs["vcu_mcu_status"].timeout_ms / 1000.0,
                cfg.can.watchdogs["vcu_mcu_status"].dtc,
            ),
        }

        # Cached state from received frames.
        # Default to 1.0 so VCU stays ACTIVE at startup before the first BMS frame
        # arrives (BMS cycle is 100ms, VCU cycle is 50ms — the first VCU tick fires
        # before BMS has published anything).
        self._drive_permission = 1.0
        self._fault_flags = 0.0
        self._bms_frame_received = False

        # Torque slew state
        self._torque_filtered = 0.0

        # Config shortcuts
        self._vcu_cfg = cfg.vehicle.vcu
        self._motor_cfg = cfg.vehicle.motor

    def _on_frame(self, frame) -> None:
        """
        Handle received CAN frames.

        Updates cached state from BMS_STATUS. MCU_STATUS frames only reset
        the watchdog (handled in base class).
        """
        if frame.message_name == "BMS_STATUS":
            self._drive_permission = frame.parsed.get("BMS_DrivePermission", 0.0)
            self._fault_flags = frame.parsed.get("BMS_FaultFlags", 0.0)
            self._bms_frame_received = True

    async def _tick(self) -> None:
        """
        Execute one VCU cycle.

        1. Transition INIT → ACTIVE on first tick
        2. Guard: if SAFE_STATE, publish zero torque and return
        3. Check drive permission and fault flags → SAFE_STATE if violated
        4. Read throttle and brake inputs
        5. Apply plausibility check (ISO 26262 ASIL-B requirement)
        6. Apply torque slew rate limiting
        7. Publish VCU_COMMAND message
        """
        # FSM transition INIT → ACTIVE
        if self.fsm_state == ECUState.INIT:
            self.fsm_state = ECUState.ACTIVE

        # Guard: if SAFE_STATE, publish zero torque and return
        if self.fsm_state == ECUState.SAFE_STATE:
            self._torque_filtered = 0.0
            self._bus.publish("VCU_COMMAND", {
                "VCU_TorqueRequest": 0.0,
                "VCU_BrakeRequest": self._physics.brake_pct,
                "VCU_ActiveState": float(self.fsm_state),
            })
            return

        # Check drive permission and fault flags.
        # Use FAULT (recoverable) so VCU returns to ACTIVE when BMS clears the fault.
        # SAFE_STATE is reserved for watchdog timeouts (set by _check_watchdogs).
        if self._bms_frame_received and (self._fault_flags > 0 or self._drive_permission == 0.0):
            self.fsm_state = ECUState.FAULT
            self._torque_filtered = 0.0
            self._bus.publish("VCU_COMMAND", {
                "VCU_TorqueRequest": 0.0,
                "VCU_BrakeRequest": self._physics.brake_pct,
                "VCU_ActiveState": float(self.fsm_state),
            })
            return

        # Recover from FAULT once BMS clears the condition
        if self.fsm_state == ECUState.FAULT:
            self.fsm_state = ECUState.ACTIVE

        # Read inputs
        throttle_pct = self._physics.throttle_pct   # 0–100
        brake_pct = self._physics.brake_pct          # 0–100

        # Plausibility check (ISO 26262 ASIL-B requirement)
        thresh_t = self._vcu_cfg.throttle_plausibility_threshold_pct  # 5.0
        thresh_b = self._vcu_cfg.brake_plausibility_threshold_pct     # 5.0

        if throttle_pct > thresh_t and brake_pct > thresh_b:
            # Both throttle and brake active — torque command = 0
            torque_target = 0.0
        else:
            # Compute torque based on brake or throttle
            if brake_pct > thresh_b:
                # Braking: negative torque (regenerative braking)
                torque_target = -(brake_pct / 100.0) * self.REGEN_MAX_NM
            else:
                # Acceleration: positive torque
                torque_target = (throttle_pct / 100.0) * self._motor_cfg.peak_torque_nm

        # Slew rate limiter
        dt = self.CYCLE_TIME   # 50ms
        max_delta = self._vcu_cfg.torque_slew_rate_nm_per_s * dt
        delta = torque_target - self._torque_filtered
        self._torque_filtered += max(-max_delta, min(max_delta, delta))

        # Publish VCU_COMMAND (0x120)
        self._bus.publish("VCU_COMMAND", {
            "VCU_TorqueRequest": self._torque_filtered,
            "VCU_BrakeRequest": brake_pct,
            "VCU_ActiveState": float(self.fsm_state),
        })

    def get_telemetry(self) -> dict:
        """Return current telemetry values."""
        return {
            "throttle_pct": self._physics.throttle_pct,
            "brake_pct": self._physics.brake_pct,
            "torque_request": self._torque_filtered,
            "state": self.fsm_state.name,
        }
