"""
EV CAN Simulation — Entry Point

Thread 1: asyncio event loop — ECUs (BMS, VCU, MCU) + PhysicsEngine
Thread 2: uvicorn daemon — FastAPI/WebSocket dashboard server

Cross-thread data:
  telemetry_q: queue.Queue — Thread 1 writes, Thread 2 reads
  command_q:   queue.Queue — Thread 2 writes, Thread 1 reads
"""

import asyncio
import queue
import signal
import threading
from pathlib import Path

from src.config_loader import load_configs
from src.can_bus import CANBus
from src.physics import PhysicsEngine
from src.ecu.bms import BMS
from src.ecu.vcu import VCU
from src.ecu.mcu import MCU
from src.web.server import start_web_server

CONFIG_DIR = Path(__file__).parent / "config"
DBC_PATH   = Path(__file__).parent / "can_bus.dbc"


def build_telemetry(bms, vcu, mcu, bus, physics_state) -> dict:
    """Build the telemetry snapshot pushed to WebSocket clients."""
    bms_telem = bms.get_telemetry()
    vcu_telem = vcu.get_telemetry()
    mcu_telem = mcu.get_telemetry()

    return {
        "bms": {
            "soc":              bms_telem["soc"],
            "voltage":          bms_telem["voltage"],
            "current":          bms_telem["current"],
            "temp":             bms_telem["temp"],
            "fault_flags":      bms_telem["fault_flags"],
            "drive_permission": bms_telem["drive_permission"],
            "max_discharge_a":  bms_telem["max_discharge_a"],
            "max_charge_a":     bms_telem["max_charge_a"],
            "state":            bms_telem["state"],
            "v_dc_link":        bms_telem["v_dc_link"],
            "precharge_relay":  bms_telem["precharge_relay"],
            "main_contactor":   bms_telem["main_contactor"],
        },
        "vcu": {
            "throttle_pct":       vcu_telem["throttle_pct"],
            "brake_pct":          vcu_telem["brake_pct"],
            "torque_request":     vcu_telem["torque_request"],
            "state":              vcu_telem["state"],
        },
        "mcu": {
            "actual_torque":      mcu_telem["actual_torque"],
            "motor_rpm":          mcu_telem["motor_rpm"],
            "motor_temp":         mcu_telem["motor_temp"],
            "inverter_temp":      mcu_telem["inverter_temp"],
            "state":              mcu_telem["state"],
        },
        "vehicle": {
            "speed_kmh":          physics_state.vehicle_speed_ms * 3.6,
        },
        "can_log":               bus.get_log(15),
        "bus_load_pct":          bus.bus_load_pct,
        "active_dtcs":           sorted(bms.active_dtcs + vcu.active_dtcs + mcu.active_dtcs),
    }


async def sim_loop(
    bms: BMS, vcu: VCU, mcu: MCU,
    physics: PhysicsEngine,
    bus: CANBus,
    telemetry_q: queue.Queue,
    command_q:   queue.Queue,
    shutdown_event: threading.Event,
) -> None:
    """Thread 1: Runs all ECU coroutines + physics engine + telemetry push."""

    # Start all ECU tasks and physics
    tasks = [
        asyncio.create_task(bms.run()),
        asyncio.create_task(vcu.run()),
        asyncio.create_task(mcu.run()),
        asyncio.create_task(physics.run(command_q)),
    ]

    # Telemetry push task: every 100ms collect snapshot and push to telemetry_q
    async def push_telemetry():
        while True:
            await asyncio.sleep(0.100)
            snapshot = build_telemetry(bms, vcu, mcu, bus, physics.state)
            bus.push_telemetry(snapshot, telemetry_q)

    tasks.append(asyncio.create_task(push_telemetry()))

    # Shutdown watcher: check shutdown event every 200ms
    async def shutdown_watcher():
        while True:
            await asyncio.sleep(0.2)
            if shutdown_event.is_set():
                for t in tasks:
                    t.cancel()
                return

    tasks.append(asyncio.create_task(shutdown_watcher()))

    try:
        await asyncio.gather(*tasks, return_exceptions=True)
    except asyncio.CancelledError:
        pass


def main() -> None:
    # Load config and validate at startup
    cfg = load_configs(CONFIG_DIR)

    # Shared CAN bus (read-only after init — cantools DBC thread-safe for reads)
    bus = CANBus(DBC_PATH, cfg.can)

    # Physics engine (owns PhysicsState, runs in Thread 1)
    physics = PhysicsEngine(cfg)
    physics._bus = bus  # give physics access to bus for fault injection (wire_cut, crc_corrupt)

    # ECUs (run in Thread 1)
    bms = BMS(bus, physics.state, cfg)
    vcu = VCU(bus, physics.state, cfg)
    mcu = MCU(bus, physics.state, cfg)

    # Cross-thread queues (stdlib, thread-safe)
    telemetry_q: queue.Queue = queue.Queue(maxsize=10)
    command_q:   queue.Queue = queue.Queue(maxsize=50)

    # Shutdown coordination
    shutdown_event = threading.Event()

    def signal_handler(signum, frame):
        print("\nShutdown requested...")
        shutdown_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, signal_handler)

    # Thread 2: web server (daemon thread)
    web_thread = threading.Thread(
        target=start_web_server,
        args=(telemetry_q, command_q, bms, vcu, mcu, shutdown_event, bus),
        daemon=True,
        name="WebServer",
    )
    web_thread.start()
    print("Web server started at http://localhost:8000")

    # Thread 1: simulation asyncio loop (blocks main thread)
    try:
        asyncio.run(sim_loop(bms, vcu, mcu, physics, bus, telemetry_q, command_q, shutdown_event))
    except (KeyboardInterrupt, SystemExit):
        shutdown_event.set()

    print("Simulation stopped.")


if __name__ == "__main__":
    main()
