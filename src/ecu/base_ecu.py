import asyncio
import abc
import time
from enum import IntEnum
from typing import Any

# ISO 14229-1 DTC category prefix → 2-bit field (bits 15:14 of the 16-bit DTC word)
_DTC_CATEGORY: dict[str, int] = {'P': 0b00, 'C': 0b01, 'B': 0b10, 'U': 0b11}


def _encode_dtc(dtc: str) -> bytes:
    """Encode a DTC string to ISO 14229 2-byte DTC word + 1-byte status mask.

    ISO 14229-1 / SAE J2012DA encoding:
      bits 15:14 — category  (P=00, C=01, B=10, U=11)
      bits 13:12 — first hex digit of numeric code
      bits 11:0  — remaining three hex digits

    Examples:
      U0100 → 0xC100 + 0x08  (11_00_0001_0000_0000)
      P0A1F → 0x0A1F + 0x08  (00_00_1010_0001_1111)
    """
    prefix  = dtc[0].upper()
    numeric = dtc[1:]                          # e.g. "0A1F" or "0100"
    category    = _DTC_CATEGORY.get(prefix, 0b00)
    first_nibble = int(numeric[0], 16)         # e.g. 0 from "0A1F"
    rest         = int(numeric[1:], 16)        # e.g. 0xA1F from "A1F"
    word = (category << 14) | (first_nibble << 12) | rest
    return word.to_bytes(2, 'big') + b'\x08'  # 2-byte DTC + confirmedDTC status


class ECUState(IntEnum):
    """FSM states for ECU operation."""
    INIT       = 0
    ACTIVE     = 1
    FAULT      = 2
    SAFE_STATE = 3
    PRECHARGE  = 4   # BMS-only: pre-charge relay closed, waiting for V_dc ≥ 90% V_pack


class WatchdogTimer:
    """Monitors frame reception timeout. Triggers DTC on expiry."""
    def __init__(self, timeout_s: float, dtc: str):
        self.timeout  = timeout_s
        self.dtc      = dtc
        self._last_rx = time.monotonic()
        self.expired  = False

    def reset(self) -> None:
        """Reset the watchdog timer."""
        self._last_rx = time.monotonic()
        self.expired  = False

    def check(self) -> bool:
        """Returns True if watchdog has expired (timed out)."""
        if not self.expired and (time.monotonic() - self._last_rx > self.timeout):
            self.expired = True
        return self.expired


class BaseECU(abc.ABC):
    """Base class for all ECUs (BMS, VCU, MCU). Provides FSM, watchdog, and UDS handler."""
    ECU_NAME: str = "BASE"
    CYCLE_TIME: float = 0.050  # subclasses override

    def __init__(self, can_bus: Any, physics_state: Any, cfg: Any):
        self._bus          = can_bus
        self._physics      = physics_state
        self._cfg          = cfg
        self._inbox        = asyncio.Queue(maxsize=64)
        self._watchdogs: dict[int, WatchdogTimer] = {}
        self._dtcs: set[str] = set()
        self.fsm_state     = ECUState.INIT
        self._running      = True

    def subscribe_to(self, *can_ids: int) -> None:
        """Register this ECU's inbox for the given CAN IDs."""
        for can_id in can_ids:
            self._bus.subscribe(can_id, self._inbox)

    async def run(self) -> None:
        """Drift-free scheduler. Overriding is not recommended — override _tick instead."""
        loop      = asyncio.get_event_loop()
        next_wake = loop.time()
        while self._running:
            next_wake += self.CYCLE_TIME
            await asyncio.sleep(max(0, next_wake - loop.time()))
            self._check_watchdogs()
            await self._tick()
            self._drain_inbox()

    def stop(self) -> None:
        """Stop the ECU task."""
        self._running = False

    def _check_watchdogs(self) -> None:
        """Check all registered watchdogs; transition to SAFE_STATE on expiry."""
        for wdog in self._watchdogs.values():
            if wdog.check() and self.fsm_state != ECUState.SAFE_STATE:
                self._dtcs.add(wdog.dtc)
                self.fsm_state = ECUState.SAFE_STATE

    def _drain_inbox(self) -> None:
        """Drain inbox queue and reset watchdogs for received frame IDs."""
        while True:
            try:
                frame = self._inbox.get_nowait()
                self._on_frame(frame)
                if frame.can_id in self._watchdogs:
                    self._watchdogs[frame.can_id].reset()
            except asyncio.QueueEmpty:
                break

    @abc.abstractmethod
    async def _tick(self) -> None:
        """Called every CYCLE_TIME. Subclasses implement their logic here."""
        ...

    def _on_frame(self, frame) -> None:
        """Called for each received frame. Subclasses override to update cached state."""
        pass

    def handle_uds(self, data: bytes) -> bytes:
        """ISO 14229-1 UDS handler. NO OBD-II (0x09) — UDS only."""
        if len(data) < 1:
            return bytes([0x7F, 0x00, 0x11])
        sid = data[0]
        match sid:
            case 0x22:                                      # ReadDataByIdentifier
                if len(data) < 3:
                    return bytes([0x7F, 0x22, 0x13])        # incorrectMessageLength
                did = (data[1] << 8) | data[2]
                if did == 0xF190:                           # VIN
                    return bytes([0x62, 0xF1, 0x90]) + b"AG1MOTO00000001"
                if did == 0xF18C:                           # ECU serial number
                    return bytes([0x62, 0xF1, 0x8C]) + self.ECU_NAME.ljust(8).encode()[:8]
                return bytes([0x7F, 0x22, 0x31])            # requestOutOfRange
            case 0x19:                                      # ReadDTCInformation
                if len(data) < 2 or data[1] != 0x02:
                    return bytes([0x7F, 0x19, 0x12])        # subFunctionNotSupported
                payload = b"".join(_encode_dtc(d) for d in sorted(self._dtcs))
                # 0xFF = DTCStatusAvailabilityMask (all status bits supported)
                return bytes([0x59, 0x02, 0xFF]) + payload
            case 0x14:                                      # ClearDiagnosticInformation
                self._dtcs.clear()
                return bytes([0x54])
            case _:
                return bytes([0x7F, sid, 0x11])             # serviceNotSupported

    @property
    def active_dtcs(self) -> list[str]:
        """Return sorted list of active DTCs."""
        return sorted(self._dtcs)
