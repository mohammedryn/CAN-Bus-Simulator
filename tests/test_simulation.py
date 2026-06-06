"""
Simulation tests for EV CAN project.
Exactly 6 tests covering DBC round-trip, SOC energy balance, FSM transitions,
watchdog expiry, throttle/brake plausibility, and regen current clamping.
"""

import asyncio
import math
from pathlib import Path

import pytest

from src.config_loader import load_configs
from src.can_bus import CANBus
from src.physics import PhysicsEngine, PhysicsState
from src.ecu.base_ecu import ECUState
from src.ecu.bms import BMS
from src.ecu.vcu import VCU
from src.ecu.mcu import MCU

CONFIG_DIR = Path("D:/arys_garage/config")
DBC_PATH = Path("D:/arys_garage/can_bus.dbc")


def make_bus_and_cfg():
    cfg = load_configs(CONFIG_DIR)
    bus = CANBus(DBC_PATH, cfg.can)
    return bus, cfg


# ---------------------------------------------------------------------------
# Test 1 — DBC bit-packing round-trip (all four messages)
# ---------------------------------------------------------------------------

async def test_dbc_roundtrip():
    bus, cfg = make_bus_and_cfg()

    # --- BMS_STATUS ---
    q_bms = asyncio.Queue()
    bus.subscribe(0x110, q_bms)
    bus.publish("BMS_STATUS", {
        "BMS_SOC": 75.0,
        "BMS_PackVoltage": 37.5,
        "BMS_PackCurrent": 50.0,
        "BMS_PackTemp": 30.0,
        "BMS_FaultFlags": 0.0,
        "BMS_DrivePermission": 1.0,
    })
    frame = q_bms.get_nowait()
    assert abs(frame.parsed["BMS_SOC"] - 75.0) <= 0.1 / 2
    assert abs(frame.parsed["BMS_PackVoltage"] - 37.5) <= 0.01 / 2
    assert abs(frame.parsed["BMS_PackCurrent"] - 50.0) <= 0.1 / 2
    assert abs(frame.parsed["BMS_PackTemp"] - 30.0) <= 1.0 / 2
    assert frame.parsed["BMS_FaultFlags"] == 0.0
    assert frame.parsed["BMS_DrivePermission"] == 1.0

    # --- BMS_LIMITS ---
    q_lim = asyncio.Queue()
    bus.subscribe(0x111, q_lim)
    bus.publish("BMS_LIMITS", {
        "BMS_MaxDischargeCurrent": 150.0,
        "BMS_MaxChargeCurrent": 20.0,
    })
    frame = q_lim.get_nowait()
    assert abs(frame.parsed["BMS_MaxDischargeCurrent"] - 150.0) <= 0.1 / 2
    assert abs(frame.parsed["BMS_MaxChargeCurrent"] - 20.0) <= 0.1 / 2

    # --- VCU_COMMAND ---
    q_vcu = asyncio.Queue()
    bus.subscribe(0x120, q_vcu)
    bus.publish("VCU_COMMAND", {
        "VCU_TorqueRequest": 100.0,
        "VCU_BrakeRequest": 0.0,
        "VCU_ActiveState": 1.0,
    })
    frame = q_vcu.get_nowait()
    assert abs(frame.parsed["VCU_TorqueRequest"] - 100.0) <= 0.1 / 2
    assert frame.parsed["VCU_BrakeRequest"] == 0.0
    assert frame.parsed["VCU_ActiveState"] == 1.0

    # --- MCU_STATUS (with Motorola encoding assertion) ---
    q_mcu = asyncio.Queue()
    bus.subscribe(0x130, q_mcu)
    frame = bus.publish("MCU_STATUS", {
        "MCU_ActualTorque": 0.0,
        "MCU_MotorRPM": 4000,
        "MCU_MotorTemp": 0.0,
        "MCU_InverterTemp": 0.0,
        "MCU_ActiveState": 0,
    })
    # raw = 4000 + 15000 = 19000 = 0x4A38
    assert frame.data[2] == 0x4A, f"Expected 0x4A at byte 2, got 0x{frame.data[2]:02X}"
    assert frame.data[3] == 0x38, f"Expected 0x38 at byte 3, got 0x{frame.data[3]:02X}"
    # Also verify round-trip from queue
    frame2 = q_mcu.get_nowait()
    assert abs(frame2.parsed["MCU_MotorRPM"] - 4000.0) <= 1.0 / 2


# ---------------------------------------------------------------------------
# Test 2 — SOC energy balance (1RC ECM)
# ---------------------------------------------------------------------------

async def test_soc_energy_balance():
    cfg = load_configs(CONFIG_DIR)
    engine = PhysicsEngine(cfg)

    # _soc is fraction (0-1), initialized to 0.80
    initial_soc_fraction = engine._battery._soc   # 0.80

    # Discharge for 360s at 100A (36000 steps × 0.01s)
    for _ in range(36000):
        engine._battery.step(100.0, 0.01)

    # Read final state with a tiny step
    result = engine._battery.step(0.0, 0.001)
    final_soc_pct = result["soc"]   # percent 0-100

    # Convert initial fraction to percent for consistent subtraction
    initial_soc_pct = initial_soc_fraction * 100.0
    delta_soc = (initial_soc_pct - final_soc_pct) / 100.0   # back to fraction

    # Q_pack = cell_capacity_ah * cells_parallel = 5.0 * 10 = 50 Ah
    expected_delta = (100.0 * 360.0) / (3600.0 * 50.0)   # = 0.200

    assert abs(delta_soc - expected_delta) < 0.005, (
        f"Expected ΔSOC ≈ {expected_delta:.4f}, got {delta_soc:.4f}"
    )


# ---------------------------------------------------------------------------
# Test 3 — FSM state transitions
# ---------------------------------------------------------------------------

async def test_fsm_transitions():
    cfg = load_configs(CONFIG_DIR)
    bus, _ = make_bus_and_cfg()
    state = PhysicsState()

    # 1a. BMS: first _tick() → PRECHARGE (closes pre-charge relay)
    bms = BMS(bus, state, cfg)
    assert bms.fsm_state == ECUState.INIT
    await bms._tick()
    assert bms.fsm_state == ECUState.PRECHARGE, (
        f"Expected BMS PRECHARGE after first tick, got {bms.fsm_state}"
    )
    assert state.precharge_relay is True
    assert state.main_contactor  is False

    # 1b. BMS: simulate V_dc reaching 95% of V_pack → ACTIVE (closes main contactor)
    state.v_dc_link = state.v_terminal * 0.95
    await bms._tick()
    assert bms.fsm_state == ECUState.ACTIVE, (
        f"Expected BMS ACTIVE after pre-charge, got {bms.fsm_state}"
    )
    assert state.precharge_relay is False
    assert state.main_contactor  is True

    # 2. VCU: fault_flags > 0, drive_permission = 0 → FAULT after _tick()
    # (FAULT is recoverable; SAFE_STATE is reserved for watchdog timeouts)
    vcu = VCU(bus, state, cfg)
    vcu.fsm_state = ECUState.ACTIVE   # skip INIT transition
    vcu._fault_flags = 1.0
    vcu._drive_permission = 0.0
    vcu._bms_frame_received = True    # simulate having received at least one BMS frame
    await vcu._tick()
    assert vcu.fsm_state == ECUState.FAULT, (
        f"Expected VCU FAULT after drive-permission denial, got {vcu.fsm_state}"
    )

    # 3. MCU: SAFE_STATE → torque_cmd must be 0.0
    mcu = MCU(bus, state, cfg)
    mcu.fsm_state = ECUState.SAFE_STATE
    state.torque_cmd = 99.0   # set a non-zero value first
    await mcu._tick()
    assert state.torque_cmd == 0.0, (
        f"Expected torque_cmd == 0.0 in SAFE_STATE, got {state.torque_cmd}"
    )


# ---------------------------------------------------------------------------
# Test 4 — Watchdog expiry and DTC generation
# ---------------------------------------------------------------------------

async def test_watchdog_expiry():
    cfg = load_configs(CONFIG_DIR)
    bus, _ = make_bus_and_cfg()
    state = PhysicsState()

    vcu = VCU(bus, state, cfg)
    # Override watchdog timeout to 100ms for fast test
    vcu._watchdogs[0x110].timeout = 0.10

    # Publish one BMS_STATUS frame to reset watchdog
    bus.publish("BMS_STATUS", {
        "BMS_SOC": 80.0,
        "BMS_PackVoltage": 37.0,
        "BMS_PackCurrent": 0.0,
        "BMS_PackTemp": 25.0,
        "BMS_FaultFlags": 0.0,
        "BMS_DrivePermission": 1.0,
    })
    vcu._drain_inbox()
    assert not vcu._watchdogs[0x110].expired, "Watchdog should not be expired right after reset"

    # Wait 150ms without publishing any new BMS_STATUS
    await asyncio.sleep(0.15)

    # Check watchdogs — should trigger SAFE_STATE
    vcu._check_watchdogs()

    assert vcu.fsm_state == ECUState.SAFE_STATE, (
        f"Expected SAFE_STATE after watchdog expiry, got {vcu.fsm_state}"
    )
    assert "U0100" in vcu._dtcs, (
        f"Expected DTC U0100, got {vcu._dtcs}"
    )


# ---------------------------------------------------------------------------
# Test 5 — Throttle/brake plausibility cut
# ---------------------------------------------------------------------------

async def test_plausibility():
    cfg = load_configs(CONFIG_DIR)
    bus, _ = make_bus_and_cfg()
    state = PhysicsState()

    vcu = VCU(bus, state, cfg)
    vcu.fsm_state = ECUState.ACTIVE
    vcu._drive_permission = 1.0
    vcu._fault_flags = 0.0
    vcu._torque_filtered = 0.0

    # Set simultaneous throttle and brake (above plausibility thresholds of 5%)
    state.throttle_pct = 80.0
    state.brake_pct = 20.0

    q = asyncio.Queue()
    bus.subscribe(0x120, q)

    await vcu._tick()

    frame = q.get_nowait()
    # Both throttle and brake active → plausibility cut → torque = 0
    assert frame.parsed["VCU_TorqueRequest"] == 0.0, (
        f"Expected 0.0 torque on plausibility cut, got {frame.parsed['VCU_TorqueRequest']}"
    )

    # Now only throttle, no brake
    state.throttle_pct = 80.0
    state.brake_pct = 0.0
    await vcu._tick()
    frame = q.get_nowait()
    assert frame.parsed["VCU_TorqueRequest"] > 0.0, (
        f"Expected positive torque with throttle only, got {frame.parsed['VCU_TorqueRequest']}"
    )


# ---------------------------------------------------------------------------
# Test 6 — Regen current clamping against BMS_MaxChargeCurrent
# ---------------------------------------------------------------------------

async def test_regen_clamp():
    cfg = load_configs(CONFIG_DIR)
    bus, _ = make_bus_and_cfg()
    state = PhysicsState()

    mcu = MCU(bus, state, cfg)
    mcu.fsm_state = ECUState.ACTIVE

    # Inject BMS limit: MaxChargeCurrent = 10A
    mcu._bms_max_charge_current = 10.0

    # Set physics state for regen scenario
    state.motor_rpm = 3000.0
    state.v_terminal = 37.0
    state.motor_temp = 25.0
    state.inverter_temp = 25.0

    # Large regen demand
    mcu._vcu_torque_request = -200.0

    await mcu._tick()

    # torque_cmd should be negative (regen) and clamped
    assert state.torque_cmd < 0, (
        f"Expected negative torque_cmd (regen), got {state.torque_cmd}"
    )

    # Verify charge current from actual torque <= 10A (+small tolerance)
    omega = 3000.0 * math.pi / 30.0
    eff = cfg.vehicle.motor.efficiency   # 0.92
    i_actual = abs(state.torque_cmd) * omega * eff / state.v_terminal

    assert i_actual <= 10.1, (
        f"Expected regen current <= 10.1 A, got {i_actual:.3f} A"
    )
