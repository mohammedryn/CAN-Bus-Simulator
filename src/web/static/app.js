/**
 * EV CAN Dashboard — app.js
 * Ary's Garage Real-time WebSocket Dashboard
 */

'use strict';

// ── Constants ─────────────────────────────────────────
const WS_URL        = `ws://${location.host}/ws`;
const UDS_URL       = `/uds`;
const RECONNECT_MS  = 2000;
const CHART_POINTS  = 600;   // 60s at 100ms interval

// Speedometer geometry (SVG viewBox 220x140)
const SPD_CX = 110, SPD_CY = 125;
const SPD_R  = 92;
const SPD_START_DEG = 210;   // degrees clockwise from 3-o'clock (SVG)
const SPD_SWEEP_DEG = 120;   // 210 → 90 deg sweep across 150 km/h
const SPD_MAX = 150;

// ── State ─────────────────────────────────────────────
let ws            = null;
let wsConnected   = false;
let reconnectTimer = null;
let telemetryChart = null;
let canFrameCount = 0;
const MAX_CAN_ROWS = 15;

// ── Utilities ─────────────────────────────────────────
/**
 * Polar → Cartesian (SVG coordinate system, angle from top = 0)
 * @param {number} cx
 * @param {number} cy
 * @param {number} r
 * @param {number} angleDeg  degrees, 0 = right (+x), increases clockwise
 */
function polarToCartesian(cx, cy, r, angleDeg) {
  const rad = (angleDeg - 90) * Math.PI / 180;
  return {
    x: cx + r * Math.cos(rad),
    y: cy + r * Math.sin(rad),
  };
}

/**
 * SVG arc path for speedometer.
 * Angles are in standard SVG/math conventions (0 = east, clockwise).
 */
function arcPath(cx, cy, r, startAngleDeg, endAngleDeg) {
  // Convert "speedometer" degrees (0 = east, CW) to SVG
  const start = polarToCartesianSVG(cx, cy, r, startAngleDeg);
  const end   = polarToCartesianSVG(cx, cy, r, endAngleDeg);
  const large = (endAngleDeg - startAngleDeg + 360) % 360 > 180 ? 1 : 0;
  return `M ${start.x} ${start.y} A ${r} ${r} 0 ${large} 1 ${end.x} ${end.y}`;
}

function polarToCartesianSVG(cx, cy, r, angleDeg) {
  const rad = angleDeg * Math.PI / 180;
  return { x: cx + r * Math.cos(rad), y: cy + r * Math.sin(rad) };
}

function clamp(val, min, max) {
  return Math.max(min, Math.min(max, val));
}

function fmt1(n) {
  return (typeof n === 'number' ? n.toFixed(1) : '--');
}

function fmt0(n) {
  return (typeof n === 'number' ? Math.round(n).toString() : '--');
}

// ── WebSocket ─────────────────────────────────────────
function connectWebSocket() {
  if (ws) {
    ws.onclose = null;
    ws.onerror = null;
    try { ws.close(); } catch(_) {}
    ws = null;
  }

  setWSStatus(false);

  try {
    ws = new WebSocket(WS_URL);
  } catch (err) {
    scheduleReconnect();
    return;
  }

  ws.onopen = () => {
    wsConnected = true;
    setWSStatus(true);
    if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
  };

  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      updateUI(data);
    } catch (err) {
      console.warn('Failed to parse WS message:', err);
    }
  };

  ws.onclose = () => {
    wsConnected = false;
    setWSStatus(false);
    scheduleReconnect();
  };

  ws.onerror = () => {
    wsConnected = false;
    setWSStatus(false);
  };
}

function scheduleReconnect() {
  if (reconnectTimer) return;
  reconnectTimer = setTimeout(() => {
    reconnectTimer = null;
    connectWebSocket();
  }, RECONNECT_MS);
}

function setWSStatus(connected) {
  const dot    = document.getElementById('ws-dot');
  const label  = document.getElementById('ws-status');
  if (connected) {
    dot.classList.add('connected');
    label.textContent = 'CONNECTED';
    label.style.color = 'var(--color-success)';
  } else {
    dot.classList.remove('connected');
    label.textContent = 'DISCONNECTED';
    label.style.color = 'var(--color-fault)';
  }
}

function sendWS(obj) {
  if (ws && wsConnected && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

// ── Main UI update ────────────────────────────────────
function updateUI(data) {
  const bms     = data.bms     || {};
  const vcu     = data.vcu     || {};
  const mcu     = data.mcu     || {};
  const vehicle = data.vehicle || {};

  // Speedometer
  updateSpeedometer(vehicle.speed_kmh ?? 0);

  // Power (P = I × V)
  const current = bms.current ?? 0;
  const voltage = bms.voltage ?? 0;
  const powerKW = (current * voltage) / 1000;
  setEl('power-value', fmt1(powerKW));
  setEl('bms-current', fmt1(current));
  setEl('bms-voltage', fmt1(voltage));
  setEl('bms-state',   `BMS: ${bms.state || '--'}`);

  // SOC
  updateSOCBar(bms.soc ?? 0);
  setEl('bms-max-discharge', fmt0(bms.max_discharge_a));
  setEl('bms-max-charge',    fmt0(bms.max_charge_a));
  const drivePerm = bms.drive_permission;
  const driveEl   = document.getElementById('bms-drive-perm');
  if (driveEl) {
    driveEl.textContent = drivePerm ? 'DRIVE: ALLOWED' : 'DRIVE: INHIBITED';
    driveEl.style.color = drivePerm ? 'var(--color-success)' : 'var(--color-fault)';
    driveEl.style.borderColor = drivePerm
      ? 'rgba(0,255,136,0.3)' : 'rgba(255,45,85,0.3)';
  }

  // Battery temp
  const battTemp = bms.temp ?? 0;
  setEl('batt-temp-value', fmt1(battTemp));
  updateTempIndicator('temp-indicator', battTemp, 40, 55);

  // Motor temps
  setEl('motor-temp',    fmt1(mcu.motor_temp));
  setEl('inverter-temp', fmt1(mcu.inverter_temp));
  setEl('mcu-state',     `MCU: ${mcu.state || '--'}`);

  // VCU state
  setEl('vehicle-state', `VCU: ${vcu.state || '--'}`);

  // Torque / RPM
  setEl('torque-request', fmt1(vcu.torque_request));
  setEl('actual-torque',  fmt1(mcu.actual_torque));
  setEl('motor-rpm',      fmt0(mcu.motor_rpm));
  setEl('vcu-throttle',   fmt1(vcu.throttle_pct));

  // DC link / contactor status
  setEl('dc-link-voltage', fmt1(bms.v_dc_link ?? 0));
  const contactorEl = document.getElementById('contactor-state');
  if (contactorEl) {
    if (bms.main_contactor) {
      contactorEl.textContent = 'MAIN ON';
      contactorEl.style.color = 'var(--color-success)';
    } else if (bms.precharge_relay) {
      contactorEl.textContent = 'PRE-CHG';
      contactorEl.style.color = 'var(--color-battery)';
    } else {
      contactorEl.textContent = 'OPEN';
      contactorEl.style.color = 'rgba(255,255,255,0.3)';
    }
  }

  // CAN sniffer
  if (data.can_log) {
    updateCANSniffer(data.can_log, data.bus_load_pct ?? 0);
  }

  // Chart
  addChartPoint(data);

  // DTCs
  updateDTCList(data.active_dtcs || []);

  // Fault alert
  checkFaultAlert(bms, vcu, data.active_dtcs);

  // Footer timestamp
  const now = new Date();
  setEl('last-update', `Updated: ${now.toLocaleTimeString()}`);
}

// ── Speedometer ───────────────────────────────────────
/**
 * Updates the SVG speedometer arc for 0–150 km/h.
 * The arc sweeps from 210° to 330° in standard math angles
 * (measured from positive-x axis, counter-clockwise in math,
 *  but SVG Y-axis is inverted so it looks clockwise on screen).
 *
 * Visually: 0 km/h = bottom-left, 150 km/h = bottom-right.
 */
function updateSpeedometer(speedKmh) {
  const speed = clamp(speedKmh, 0, SPD_MAX);
  const trackEl = document.getElementById('speed-track');
  const arcEl   = document.getElementById('speed-arc');
  const valEl   = document.getElementById('speed-value');

  if (!trackEl || !arcEl) return;

  // Using SVG angles where 0° = east (right), clockwise positive
  // Arc start: 210° (bottom-left), end: 330° (bottom-right, equivalent to -30°)
  const startAngle = 210;
  const endAngle   = 330;  // 210 + 120
  const fillAngle  = startAngle + (speed / SPD_MAX) * 120;

  trackEl.setAttribute('d', arcPath(SPD_CX, SPD_CY, SPD_R, startAngle, endAngle));

  if (speed <= 0) {
    arcEl.setAttribute('d', '');
  } else {
    arcEl.setAttribute('d', arcPath(SPD_CX, SPD_CY, SPD_R, startAngle, fillAngle));
  }

  if (valEl) valEl.textContent = Math.round(speed);

  // Color: green < 80, amber 80–120, red > 120
  let arcColor = 'var(--color-success)';
  if (speed > 120) arcColor = 'var(--color-fault)';
  else if (speed > 80) arcColor = 'var(--color-battery)';
  if (arcEl) arcEl.setAttribute('stroke', arcColor);
}

// ── SOC Bar ───────────────────────────────────────────
function updateSOCBar(socPct) {
  const fill    = document.getElementById('soc-bar-fill');
  const pctLabel = document.getElementById('soc-pct-value');

  const pct = clamp(socPct, 0, 100);

  if (fill) {
    fill.style.width = `${pct}%`;
    let color = 'var(--color-success)';
    if (pct < 20) color = 'var(--color-fault)';
    else if (pct < 50) color = 'var(--color-battery)';
    fill.style.background = color;
    fill.style.boxShadow  = `0 0 8px ${color}`;
  }
  if (pctLabel) {
    pctLabel.textContent = `${Math.round(pct)}%`;
    let color = 'var(--color-success)';
    if (pct < 20) color = 'var(--color-fault)';
    else if (pct < 50) color = 'var(--color-battery)';
    pctLabel.style.color = color;
  }
}

// ── Temp indicator ────────────────────────────────────
function updateTempIndicator(elId, temp, warnThresh, hotThresh) {
  const el = document.getElementById(elId);
  if (!el) return;
  if (temp >= hotThresh) {
    el.className = 'temp-indicator temp-hot';
    el.textContent = 'CRITICAL';
  } else if (temp >= warnThresh) {
    el.className = 'temp-indicator temp-warm';
    el.textContent = 'ELEVATED';
  } else {
    el.className = 'temp-indicator temp-ok';
    el.textContent = 'NOMINAL';
  }

  const tempEl = document.getElementById('batt-temp-value');
  if (tempEl) {
    let color = 'var(--color-success)';
    if (temp >= hotThresh) color = 'var(--color-fault)';
    else if (temp >= warnThresh) color = 'var(--color-battery)';
    tempEl.style.color = color;
  }
}

// ── CAN Sniffer ───────────────────────────────────────
function updateCANSniffer(canLog, busLoad) {
  const framesEl  = document.getElementById('can-frames');
  const busLoadEl = document.getElementById('bus-load-pct');

  if (busLoadEl) {
    busLoadEl.textContent = typeof busLoad === 'number' ? busLoad.toFixed(1) : '0.0';
    busLoadEl.style.color = busLoad > 70
      ? 'var(--color-fault)'
      : busLoad > 40
        ? 'var(--color-battery)'
        : 'var(--color-accent)';
  }

  if (!framesEl || !canLog || !canLog.length) return;

  // Prepend new frames (newest on top via column-reverse CSS)
  canLog.forEach(frame => {
    const row = document.createElement('div');
    row.className = 'can-frame';

    const parsedStr = frame.parsed
      ? Object.entries(frame.parsed)
          .map(([k, v]) => `${k}:${typeof v === 'number' ? v.toFixed(2) : v}`)
          .join(' | ')
      : '';

    row.innerHTML =
      `<span class="can-col-ts">${frame.t || ''}</span>` +
      `<span class="can-col-id">${frame.id || ''}</span>` +
      `<span class="can-col-name">${frame.name || ''}</span>` +
      `<span class="can-col-hex">${frame.hex || ''}</span>` +
      `<span class="can-col-parsed">{${parsedStr}}</span>`;

    framesEl.insertBefore(row, framesEl.firstChild);
    canFrameCount++;
  });

  // Trim to MAX_CAN_ROWS
  while (framesEl.children.length > MAX_CAN_ROWS) {
    framesEl.removeChild(framesEl.lastChild);
  }
}

// ── Chart ─────────────────────────────────────────────
function initChart() {
  const ctx = document.getElementById('telemetry-chart');
  if (!ctx) return;

  const emptyData = () => Array(CHART_POINTS).fill(null);

  telemetryChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: emptyData(),
      datasets: [
        {
          label: 'Speed (km/h)',
          data: emptyData(),
          borderColor: '#00f5ff',
          backgroundColor: 'rgba(0,245,255,0.04)',
          borderWidth: 1.5,
          pointRadius: 0,
          tension: 0.3,
          fill: false,
          yAxisID: 'y',
        },
        {
          label: 'Torque/2 (Nm÷2)',
          data: emptyData(),
          borderColor: '#ffa500',
          backgroundColor: 'rgba(255,165,0,0.04)',
          borderWidth: 1.5,
          pointRadius: 0,
          tension: 0.3,
          fill: false,
          yAxisID: 'y',
        },
        {
          label: 'Motor Temp (°C)',
          data: emptyData(),
          borderColor: '#ff2d55',
          backgroundColor: 'rgba(255,45,85,0.04)',
          borderWidth: 1.5,
          pointRadius: 0,
          tension: 0.3,
          fill: false,
          yAxisID: 'y2',
        },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: {
        mode: 'index',
        intersect: false,
      },
      plugins: {
        legend: {
          position: 'top',
          labels: {
            color: 'rgba(255,255,255,0.6)',
            font: { family: "'Courier New', monospace", size: 11 },
            boxWidth: 16,
            padding: 14,
          },
        },
        tooltip: {
          backgroundColor: 'rgba(10,14,26,0.9)',
          borderColor: 'rgba(0,245,255,0.3)',
          borderWidth: 1,
          titleColor: '#00f5ff',
          bodyColor: 'rgba(255,255,255,0.8)',
          titleFont: { family: "'Courier New', monospace", size: 11 },
          bodyFont: { family: "'Courier New', monospace", size: 11 },
        },
      },
      scales: {
        x: {
          display: false,
          grid: { color: 'rgba(255,255,255,0.04)' },
        },
        y: {
          position: 'left',
          grid: { color: 'rgba(255,255,255,0.06)' },
          ticks: {
            color: 'rgba(255,255,255,0.45)',
            font: { family: "'Courier New', monospace", size: 10 },
            maxTicksLimit: 6,
          },
          title: {
            display: true,
            text: 'km/h  |  Nm÷2',
            color: 'rgba(255,255,255,0.35)',
            font: { family: "'Courier New', monospace", size: 10 },
          },
        },
        y2: {
          position: 'right',
          grid: { drawOnChartArea: false },
          ticks: {
            color: 'rgba(255,45,85,0.6)',
            font: { family: "'Courier New', monospace", size: 10 },
            maxTicksLimit: 5,
          },
          title: {
            display: true,
            text: 'Temp (°C)',
            color: 'rgba(255,45,85,0.5)',
            font: { family: "'Courier New', monospace", size: 10 },
          },
        },
      },
    },
  });
}

function addChartPoint(data) {
  if (!telemetryChart) return;

  const speed      = data.vehicle?.speed_kmh ?? 0;
  const torqueHalf = (data.vcu?.torque_request ?? 0) / 2;
  const motorTemp  = data.mcu?.motor_temp ?? 0;
  const ts         = new Date().toLocaleTimeString('en-GB', { hour12: false });

  const ds = telemetryChart.data.datasets;
  const lb = telemetryChart.data.labels;

  lb.push(ts);
  ds[0].data.push(speed);
  ds[1].data.push(torqueHalf);
  ds[2].data.push(motorTemp);

  // Rolling window
  if (lb.length > CHART_POINTS) {
    lb.shift();
    ds[0].data.shift();
    ds[1].data.shift();
    ds[2].data.shift();
  }

  telemetryChart.update('none');
}

// ── DTC List ──────────────────────────────────────────
function updateDTCList(dtcs) {
  const listEl = document.getElementById('dtc-list');
  if (!listEl) return;

  listEl.innerHTML = '';
  if (!dtcs || dtcs.length === 0) {
    listEl.innerHTML = '<div class="dtc-empty">No active DTCs</div>';
    return;
  }

  dtcs.forEach(code => {
    const badge = document.createElement('div');
    badge.className = 'dtc-badge';
    badge.textContent = typeof code === 'object'
      ? (code.code || JSON.stringify(code))
      : String(code);
    listEl.appendChild(badge);
  });
}

// ── Fault alert ───────────────────────────────────────
const FAULT_FLAG_NAMES = {
  1:   'Overvoltage',
  2:   'Overtemperature',
  4:   'Undervoltage',
  8:   'Overcurrent',
  16:  'Cell Imbalance',
  32:  'Communication Fault',
  64:  'Isolation Fault',
  128: 'Hardware Fault',
};

function checkFaultAlert(bms, vcu, dtcs) {
  const alertEl   = document.getElementById('fault-alert');
  const alertText = document.getElementById('fault-alert-text');
  if (!alertEl) return;

  const faultFlags  = bms.fault_flags || 0;
  const driveOk     = bms.drive_permission !== false;
  const hasDTCs     = dtcs && dtcs.length > 0;
  const hasInjected = [...document.querySelectorAll('.fault-toggle input:checked')].length > 0;

  if (faultFlags > 0 || !driveOk || hasDTCs) {
    const msgs = [];
    if (faultFlags > 0) {
      for (const [bit, name] of Object.entries(FAULT_FLAG_NAMES)) {
        if (faultFlags & Number(bit)) msgs.push(name);
      }
    }
    if (!driveOk)  msgs.push('Drive Permission Denied');
    if (hasDTCs)   msgs.push(`${dtcs.length} Active DTC(s)`);

    alertText.textContent = msgs.length > 0
      ? `FAULT: ${msgs.join(' | ')}`
      : 'FAULT DETECTED';
    alertEl.classList.remove('hidden');
  } else if (!hasInjected) {
    alertEl.classList.add('hidden');
  }
}

// ── Controls ──────────────────────────────────────────
function onThrottle(value) {
  const v = Number(value);
  setEl('throttle-display', v);
  updateSliderFill('throttle-slider', v, '--throttle-pct');
  sendWS({ type: 'throttle', value: v });
}

function onBrake(value) {
  const v = Number(value);
  setEl('brake-display', v);
  updateSliderFill('brake-slider', v, '--brake-pct');
  sendWS({ type: 'brake', value: v });
}

function updateSliderFill(sliderId, value, cssVar) {
  const el = document.getElementById(sliderId);
  if (el) el.style.setProperty(cssVar, `${value}%`);
}

function onFaultToggle(faultName, active) {
  const payload = { type: 'fault', fault: faultName, active };
  sendWS(payload);

  // Update toggle visual
  const checkboxId = {
    wire_cut:         'fault-wire-cut',
    thermal_runaway:  'fault-thermal',
    overspeed:        'fault-overspeed',
    throttle_sensor:  'fault-throttle-sensor',
  }[faultName];

  if (checkboxId) {
    const label = document.getElementById(checkboxId)?.closest('.fault-toggle');
    if (label) {
      label.classList.toggle('active', active);
    }
  }

  // Show/hide global fault alert for injected faults
  if (active) {
    const alertEl   = document.getElementById('fault-alert');
    const alertText = document.getElementById('fault-alert-text');
    if (alertEl && alertText) {
      alertText.textContent = `FAULT INJECTED: ${faultName.replace(/_/g, ' ').toUpperCase()}`;
      alertEl.classList.remove('hidden');
    }
  }
}

// ── DTC clear ─────────────────────────────────────────
async function clearDTCs() {
  // Send UDS 0x14 (ClearDiagnosticInformation) to all three ECUs
  try {
    await Promise.all(['bms', 'vcu', 'mcu'].map(ecu =>
      fetch('/uds', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ecu, payload: '14FFFFFF' }),
      })
    ));
    showUDSResponse('Clear DTCs: OK — all ECUs cleared');
    updateDTCList([]);
  } catch (err) {
    showUDSResponse(`Error: ${err.message}`);
  }
}

// Maps button command strings → { ecu, payload } for the /uds endpoint
const UDS_COMMANDS = {
  'read_vin_bms':  { ecu: 'bms', payload: '22F190' },
  'read_dtc_vcu':  { ecu: 'vcu', payload: '19020F' },
  'clear_dtc_mcu': { ecu: 'mcu', payload: '14FFFFFF' },
};

// ── UDS ───────────────────────────────────────────────
async function sendUDS(command) {
  const btn = event?.target;
  if (btn) { btn.disabled = true; btn.textContent = '...'; }

  const udsCmd = UDS_COMMANDS[command];
  if (!udsCmd) {
    showUDSResponse(`Unknown UDS command: ${command}`);
    if (btn) { btn.disabled = false; btn.textContent = command; }
    return;
  }

  try {
    const res = await fetch(UDS_URL, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(udsCmd),
    });
    const data = await res.json().catch(() => ({ result: res.statusText }));
    showUDSResponse(JSON.stringify(data, null, 2));
  } catch (err) {
    showUDSResponse(`Error: ${err.message}`);
  } finally {
    if (btn) {
      btn.disabled = false;
      btn.textContent = {
        read_vin_bms:  'Read VIN (BMS)',
        read_dtc_vcu:  'Read DTC (VCU)',
        clear_dtc_mcu: 'Clear DTC (MCU)',
      }[command] || command;
    }
  }
}

function showUDSResponse(text) {
  const el = document.getElementById('uds-response');
  if (!el) return;
  el.textContent = text;
  el.classList.add('visible');
  setTimeout(() => el.classList.remove('visible'), 8000);
}

// ── ASC Export ───────────────────────────────────────
function downloadASC() {
  const a = document.createElement('a');
  a.href = '/api/export/asc';
  a.download = 'session.asc';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
}

// ── Helpers ───────────────────────────────────────────
function setEl(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

// ── Init ──────────────────────────────────────────────
document.addEventListener('DOMContentLoaded', () => {
  initChart();
  connectWebSocket();

  // Initialize slider fills to 0
  updateSliderFill('throttle-slider', 0, '--throttle-pct');
  updateSliderFill('brake-slider',    0, '--brake-pct');

  // Draw empty speedometer track once
  const trackEl = document.getElementById('speed-track');
  if (trackEl) {
    trackEl.setAttribute('d', arcPath(SPD_CX, SPD_CY, SPD_R, 210, 330));
  }
  // Draw initial (empty) arc
  updateSpeedometer(0);
});
