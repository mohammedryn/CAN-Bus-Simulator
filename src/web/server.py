"""
Thread 2: FastAPI/WebSocket web server.

Cross-thread data flow (NEVER use asyncio.Queue across threads):
  Thread 1 → Thread 2: stdlib queue.Queue (telemetry snapshots)
  Thread 2 → Thread 1: stdlib queue.Queue (user commands: throttle/brake/fault)
"""

import asyncio
import json
import queue
from pathlib import Path
from typing import Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse, Response
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware


_clients: Set[WebSocket] = set()


def _build_app(
    telemetry_q: queue.Queue,   # Thread 1 → Thread 2
    command_q:   queue.Queue,   # Thread 2 → Thread 1
    bms_ecu,                    # for UDS requests
    vcu_ecu,
    mcu_ecu,
    bus,                        # for .asc export
) -> FastAPI:
    app = FastAPI(title="EV CAN Simulation Dashboard")

    app.add_middleware(CORSMiddleware, allow_origins=["*"],
                       allow_methods=["*"], allow_headers=["*"])

    static_dir = Path(__file__).parent / "static"
    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

    @app.get("/")
    async def root():
        return FileResponse(str(static_dir / "index.html"))

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        _clients.add(ws)
        try:
            async for msg in ws.iter_text():
                try:
                    cmd = json.loads(msg)
                    command_q.put_nowait(cmd)
                except (json.JSONDecodeError, queue.Full):
                    pass
        except WebSocketDisconnect:
            pass
        finally:
            _clients.discard(ws)

    @app.post("/control")
    async def control(body: dict):
        """POST { "type": "throttle"|"brake", "value": 0–100 }"""
        try:
            command_q.put_nowait(body)
        except queue.Full:
            pass
        return {"ok": True}

    @app.post("/fault")
    async def fault(body: dict):
        """POST { "fault": "wire_cut|thermal_runaway|overspeed|throttle_sensor", "active": true|false }"""
        try:
            body["type"] = "fault"
            command_q.put_nowait(body)
        except queue.Full:
            pass
        return {"ok": True}

    @app.get("/api/export/asc")
    async def export_asc():
        """Download the full session CAN log as a CANalyzer-compatible .asc file."""
        content = bus.generate_asc()
        return Response(
            content=content,
            media_type="text/plain",
            headers={"Content-Disposition": 'attachment; filename="session.asc"'},
        )

    @app.post("/uds")
    async def uds(body: dict):
        """POST { "ecu": "bms"|"vcu"|"mcu", "payload": "22F190" }"""
        ecu_map = {"bms": bms_ecu, "vcu": vcu_ecu, "mcu": mcu_ecu}
        ecu_name = body.get("ecu", "").lower()
        ecu = ecu_map.get(ecu_name)
        if not ecu:
            return {"error": "unknown ecu"}
        payload_hex = body.get("payload", "")
        try:
            data = bytes.fromhex(payload_hex)
            resp = ecu.handle_uds(data)
            return {"response": resp.hex().upper()}
        except Exception as e:
            return {"error": str(e)}

    async def _broadcast_loop():
        """Background task: drain telemetry_q every 100ms, push to all WS clients."""
        while True:
            await asyncio.sleep(0.100)
            snapshots = []
            while True:
                try:
                    snapshots.append(telemetry_q.get_nowait())
                except queue.Empty:
                    break
            if not snapshots or not _clients:
                continue
            payload = json.dumps(snapshots[-1])   # send only the latest snapshot
            dead = set()
            for ws in _clients:
                try:
                    await ws.send_text(payload)
                except Exception:
                    dead.add(ws)
            _clients.difference_update(dead)

    @app.on_event("startup")
    async def startup():
        asyncio.create_task(_broadcast_loop())

    return app


def start_web_server(
    telemetry_q: queue.Queue,
    command_q:   queue.Queue,
    bms_ecu,
    vcu_ecu,
    mcu_ecu,
    shutdown_event,           # threading.Event — signals graceful shutdown
    bus,                      # CANBus — for .asc export endpoint
) -> None:
    """
    Blocking. Run as daemon thread from run.py.

    IMPORTANT: This is Thread 2. Do NOT use any asyncio.Queue from Thread 1 here.
    The only cross-thread communication is via stdlib queue.Queue objects.
    """
    app = _build_app(telemetry_q, command_q, bms_ecu, vcu_ecu, mcu_ecu, bus)

    # Use uvicorn.Server for graceful shutdown support
    config = uvicorn.Config(app, host="0.0.0.0", port=8000, log_level="warning")
    server = uvicorn.Server(config)

    # Monitor shutdown event in a thread-safe way
    import threading
    def watch_shutdown():
        shutdown_event.wait()
        server.should_exit = True

    t = threading.Thread(target=watch_shutdown, daemon=True)
    t.start()

    server.run()
