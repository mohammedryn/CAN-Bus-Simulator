import asyncio
import dataclasses
import datetime
import queue
import time
from collections import deque
from pathlib import Path
from typing import Optional

import cantools

from src.config_loader import CANConfig


@dataclasses.dataclass
class CANFrame:
    timestamp:    float
    can_id:       int
    data:         bytes
    message_name: str
    parsed:       dict[str, float]


class CANBus:
    def __init__(self, dbc_path: Path, cfg: CANConfig):
        self.db = cantools.database.load_file(str(dbc_path))
        assert self.db is not None, "Failed to load DBC file"
        self._subs: dict[int, list[asyncio.Queue]] = {}
        self._log:  deque[CANFrame] = deque(maxlen=100)
        # Separate chronological log for .asc export (50k frames ≈ 10 min at full rate)
        self._export_log: deque[CANFrame] = deque(maxlen=50_000)
        self._session_start: float = time.monotonic()
        self._bit_window: deque[tuple[float, int]] = deque()
        self._baudrate = cfg.bus.baudrate_bps
        self.fault_wire_cut    = False
        self.fault_crc_corrupt = False

    def subscribe(self, can_id: int, queue: asyncio.Queue) -> None:
        self._subs.setdefault(can_id, []).append(queue)

    def publish(self, message_name: str, signals: dict) -> Optional[CANFrame]:
        if self.fault_wire_cut:
            return None
        msg = self.db.get_message_by_name(message_name)
        # CRITICAL: Clamp all values to DBC [min, max] before encode
        # cantools raises ValueError on out-of-range — this happens during fault injection
        safe = {
            sig.name: max(sig.minimum, min(sig.maximum, signals[sig.name]))
            for sig in msg.signals if sig.name in signals
        }
        data = msg.encode(safe)
        if self.fault_crc_corrupt:
            data = data[:-1] + bytes([data[-1] ^ 0xFF])
        parsed = {k: float(v) for k, v in msg.decode(data).items()}
        frame  = CANFrame(time.monotonic(), msg.frame_id, data, message_name, parsed)
        self._log.appendleft(frame)
        self._export_log.append(frame)   # chronological; used by generate_asc()
        # CAN 2.0A (11-bit ID) frame bits: 47 fixed overhead + data + bit-stuffing estimate.
        # Stuffed region covers SOF+arb+ctrl+data+CRC = (34 + data_bits); 1 stuff bit per 5.
        _data_bits = len(data) * 8
        _frame_bits = 47 + _data_bits + (34 + _data_bits) // 5
        self._bit_window.append((frame.timestamp, _frame_bits))
        for q in self._subs.get(msg.frame_id, []):
            try:
                q.put_nowait(frame)
            except asyncio.QueueFull:
                pass   # drop if ECU is behind — real CAN also drops under overload
        return frame

    @property
    def bus_load_pct(self) -> float:
        now  = time.monotonic()
        bits = sum(b for t, b in self._bit_window if now - t < 1.0)
        return min(100.0, bits / self._baudrate * 100.0)

    def get_log(self, n: int = 20) -> list[dict]:
        return [
            {"t": f"{f.timestamp:.3f}", "id": f"0x{f.can_id:03X}",
             "name": f.message_name, "hex": f.data.hex().upper(), "parsed": f.parsed}
            for f in list(self._log)[:n]
        ]

    def generate_asc(self) -> str:
        """Return the session CAN log as a Vector CANalyzer .asc string.

        Compatible with CANalyzer, PEAK PCAN-Explorer, and cantools CLI.
        Timestamps are relative to session start (monotonic).
        """
        now_str = datetime.datetime.now().strftime("%a %b %d %H:%M:%S.000 %Y")
        lines = [
            f"date {now_str}",
            "base hex  timestamps absolute",
            "no internal events logged",
        ]
        for frame in self._export_log:
            ts       = frame.timestamp - self._session_start
            dlc      = len(frame.data)
            data_hex = " ".join(f"{b:02X}" for b in frame.data)
            # CAN 2.0A standard frame: ID without 'x' suffix, channel 1, direction Rx
            lines.append(
                f"   {ts:10.6f} 1  {frame.can_id:03X}             Rx   d {dlc} {data_hex}"
            )
        lines.append("End TriggerBlock")
        return "\n".join(lines) + "\n"

    def push_telemetry(self, snapshot: dict, bridge: queue.Queue) -> None:
        """Thread 1 only. Writes telemetry snapshot to stdlib queue for Thread 2."""
        try:
            bridge.put_nowait(snapshot)
        except queue.Full:
            pass
