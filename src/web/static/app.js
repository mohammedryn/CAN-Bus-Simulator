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
const SPD_START_DEG = 150;   // degrees clockwise from 3-o'clock (SVG)
const SPD_SWEEP_DEG = 240;   // 150 → 390 deg sweep across 150 km/h (automotive gauge)
const SPD_MAX = 150;

// ── State ─────────────────────────────────────────────
let ws            = null;
let wsConnected   = false;
let reconnectTimer = null;
let telemetryChart = null;
let canFrameCount = 0;
const MAX_CAN_ROWS = 20;

// ── Utilities ─────────────────────────────────────────
/**
 * Polar → Cartesian (SVG coordinate system)
 * @param {number} cx
 * @param {number} cy
 * @param {number} r
 * @param {number} angleDeg  degrees, 0 = right (+x), increases clockwise
 */
function polarToCartesianSVG(cx, cy, r, angleDeg) {
  const rad = angleDeg * Math.PI / 180;
  return { x: cx + r * Math.cos(rad), y: cy + r * Math.sin(rad) };
}

/**
 * SVG arc path for speedometer.
 */
function arcPath(cx, cy, r, startAngleDeg, endAngleDeg) {
  const start = polarToCartesianSVG(cx, cy, r, startAngleDeg);
  const end   = polarToCartesianSVG(cx, cy, r, endAngleDeg);
  const large = (endAngleDeg - startAngleDeg + 360) % 360 > 180 ? 1 : 0;
  return `M ${start.x} ${start.y} A ${r} ${r} 0 ${large} 1 ${end.x} ${end.y}`;
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
  const overallStateEl = document.getElementById('sys-overall-state');
  if (overallStateEl) {
    overallStateEl.textContent = vcu.state || 'INIT';
    if (vcu.state === 'ACTIVE') {
      overallStateEl.style.color = 'var(--color-success)';
      overallStateEl.style.borderColor = 'rgba(0,255,136,0.3)';
    } else if (vcu.state === 'FAULT' || vcu.state === 'SAFE_STATE') {
      overallStateEl.style.color = 'var(--color-fault)';
      overallStateEl.style.borderColor = 'rgba(255,45,85,0.3)';
    } else {
      overallStateEl.style.color = 'var(--color-battery)';
      overallStateEl.style.borderColor = 'rgba(255,165,0,0.3)';
    }
  }

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

  // Visual Chassis and Circuit components
  updatePowertrainChassis(bms, mcu, current);
  updateCircuitSchematic(bms);

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
function updateSpeedometer(speedKmh) {
  const speed = clamp(speedKmh, 0, SPD_MAX);
  const trackEl = document.getElementById('speed-track');
  const arcEl   = document.getElementById('speed-arc');
  const valEl   = document.getElementById('speed-value');

  if (!trackEl || !arcEl) return;

  const startAngle = SPD_START_DEG;
  const endAngle   = SPD_START_DEG + SPD_SWEEP_DEG;
  const fillAngle  = startAngle + (speed / SPD_MAX) * SPD_SWEEP_DEG;

  trackEl.setAttribute('d', arcPath(SPD_CX, SPD_CY, SPD_R, startAngle, endAngle));

  if (speed <= 0) {
    arcEl.setAttribute('d', '');
  } else {
    arcEl.setAttribute('d', arcPath(SPD_CX, SPD_CY, SPD_R, startAngle, fillAngle));
  }

  if (valEl) valEl.textContent = Math.round(speed);
}

// ── SOC Bar & Liquid Battery ──────────────────────────
function updateSOCBar(socPct) {
  const pctLabel = document.getElementById('soc-pct-value');
  const pct = clamp(socPct, 0, 100);

  if (pctLabel) {
    pctLabel.textContent = `${Math.round(pct)}%`;
    let color = 'var(--color-success)';
    if (pct < 20) color = 'var(--color-fault)';
    else if (pct < 50) color = 'var(--color-battery)';
    pctLabel.style.color = color;
  }

  // Update battery visual level
  const liquidEl = document.getElementById('soc-battery-liquid');
  if (liquidEl) {
    liquidEl.style.height = `${pct}%`;
    if (pct < 20) {
      liquidEl.style.background = 'linear-gradient(to top, var(--color-fault), #ef4444)';
      liquidEl.style.boxShadow = '0 0 15px rgba(255, 45, 85, 0.4)';
    } else if (pct < 50) {
      liquidEl.style.background = 'linear-gradient(to top, var(--color-battery), #f59e0b)';
      liquidEl.style.boxShadow = '0 0 15px rgba(255, 165, 0, 0.4)';
    } else {
      liquidEl.style.background = 'linear-gradient(to top, var(--color-success), #10b981)';
      liquidEl.style.boxShadow = '0 0 15px rgba(0, 255, 136, 0.4)';
    }
  }
}

// ── Temp indicator ────────────────────────────────────
function updateTempIndicator(elId, temp, warnThresh, hotThresh) {
  const el = document.getElementById(elId);
  if (!el) return;
  if (temp >= hotThresh) {
    el.className = 'thermal-indicator-badge temp-hot';
    el.textContent = 'CRITICAL';
  } else if (temp >= warnThresh) {
    el.className = 'thermal-indicator-badge temp-warm';
    el.textContent = 'ELEVATED';
  } else {
    el.className = 'thermal-indicator-badge temp-ok';
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

// ── Powertrain Vector Chassis Animation ───────────────
function updatePowertrainChassis(bms, mcu, current) {
  // 1. Color cells based on temperatures
  const cells = document.querySelectorAll('.bms-cell');
  const temp = bms.temp ?? 25;
  cells.forEach(cell => {
    cell.classList.remove('cell-ok', 'cell-warm', 'cell-runaway');
    if (temp >= 55) {
      cell.classList.add('cell-runaway');
    } else if (temp >= 40) {
      cell.classList.add('cell-warm');
    } else {
      cell.classList.add('cell-ok');
    }
  });

  // 2. Animate power flow paths based on current flow
  const pathBatInv = document.getElementById('svg-path-bat-inv');
  const directionText = document.getElementById('flow-direction-text');
  
  if (pathBatInv) {
    pathBatInv.classList.remove('discharge', 'regen');
    if (current > 1.0) {
      pathBatInv.classList.add('discharge');
      if (directionText) directionText.textContent = `DISCHARGING (${fmt1(current)}A)`;
    } else if (current < -1.0) {
      pathBatInv.classList.add('regen');
      if (directionText) directionText.textContent = `REGENERATIVE BRAKING (${fmt1(Math.abs(current))}A)`;
    } else {
      if (directionText) {
        directionText.textContent = bms.precharge_relay ? 'PRE-CHARGING' : 'IDLE';
      }
    }
  }

  // 3. Motor spinner speed depending on actual RPM
  const rotor = document.getElementById('svg-motor-rotor');
  if (rotor) {
    rotor.removeAttribute('class');
    const rpm = mcu.motor_rpm ?? 0;
    if (rpm > 4000) {
      rotor.setAttribute('class', 'rotor-spin-fast');
    } else if (rpm > 1000) {
      rotor.setAttribute('class', 'rotor-spin-medium');
    } else if (rpm > 10) {
      rotor.setAttribute('class', 'rotor-spin-slow');
    }
  }
}

// ── Pre-Charge circuit visual board schematic ─────────
function updateCircuitSchematic(bms) {
  const prechgRelay = document.getElementById('node-prechg-relay');
  const mainPos = document.getElementById('node-main-pos');
  const mainNeg = document.getElementById('node-main-neg');
  
  const wirePos1 = document.getElementById('wire-pos-1');
  const wirePrechg1 = document.getElementById('wire-prechg-1');
  const wireResistor = document.getElementById('wire-resistor-body');
  const wirePrechg2 = document.getElementById('wire-prechg-2');
  
  const wirePos2 = document.getElementById('wire-pos-2');
  const wireNeg1 = document.getElementById('wire-neg-1');
  const wireNeg2 = document.getElementById('wire-neg-2');
  const wireMcuTop = document.getElementById('wire-mcu-top');
  const wireMcuBot = document.getElementById('wire-mcu-bot');
  
  const capPlates = document.querySelectorAll('.cap-plate');
  const schemVDCEl = document.getElementById('schematic-vdc');
  const stageEl = document.getElementById('precharge-stage');

  // Reset energized classes
  const allWires = [wirePos1, wirePrechg1, wireResistor, wirePrechg2, wirePos2, wireNeg1, wireNeg2, wireMcuTop, wireMcuBot];
  allWires.forEach(w => {
    if (w) {
      w.classList.remove('energized-bms', 'energized-active', 'energized');
    }
  });
  capPlates.forEach(p => p.classList.remove('energized'));

  if (schemVDCEl) {
    schemVDCEl.textContent = `${fmt1(bms.v_dc_link ?? 0)} V`;
  }

  if (bms.main_contactor) {
    if (prechgRelay) prechgRelay.classList.remove('closed');
    if (mainPos) mainPos.classList.add('closed');
    if (mainNeg) mainNeg.classList.add('closed');

    // Energize full active circuit
    [wirePos1, wirePos2, wireNeg1, wireNeg2, wireMcuTop, wireMcuBot].forEach(w => {
      if (w) w.classList.add('energized-active');
    });
    capPlates.forEach(p => p.classList.add('energized'));
    
    if (stageEl) {
      stageEl.textContent = 'ACTIVE';
      stageEl.className = 'state-badge precharge-status-badge temp-ok';
    }
  } else if (bms.precharge_relay) {
    if (prechgRelay) prechgRelay.classList.add('closed');
    if (mainPos) mainPos.classList.remove('closed');
    if (mainNeg) mainNeg.classList.remove('closed');

    // Energize precharge branch
    [wirePos1, wirePrechg1, wireResistor, wirePrechg2].forEach(w => {
      if (w) w.classList.add('energized-bms');
    });
    if (wireResistor) wireResistor.classList.add('energized');
    capPlates.forEach(p => p.classList.add('energized'));
    
    if (stageEl) {
      stageEl.textContent = 'PRECHARGE';
      stageEl.className = 'state-badge precharge-status-badge temp-warm';
    }
  } else {
    if (prechgRelay) prechgRelay.classList.remove('closed');
    if (mainPos) mainPos.classList.remove('closed');
    if (mainNeg) mainNeg.classList.remove('closed');
    
    if (stageEl) {
      if (bms.state === 'FAULT' || bms.state === 'SAFE_STATE') {
        stageEl.textContent = 'FAULT';
        stageEl.className = 'state-badge precharge-status-badge temp-hot';
      } else {
        stageEl.textContent = 'INIT';
        stageEl.className = 'state-badge precharge-status-badge';
      }
    }
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

  // Clear initial message
  const initialMsg = framesEl.querySelector('.can-initial-msg');
  if (initialMsg) {
    framesEl.removeChild(initialMsg);
  }

  // Prepend new frames
  canLog.forEach(frame => {
    const row = document.createElement('div');
    row.className = 'can-frame';

    const parsedStr = frame.parsed
      ? Object.entries(frame.parsed)
          .map(([k, v]) => `${k}:${typeof v === 'number' ? v.toFixed(1) : v}`)
          .join(' | ')
      : '';

    // Color code CAN IDs for readability
    let idColorClass = '';
    if (frame.id === '0x110') idColorClass = 'style="color: var(--color-success)"';
    else if (frame.id === '0x111') idColorClass = 'style="color: var(--color-accent)"';
    else if (frame.id === '0x120') idColorClass = 'style="color: #60a5fa"';
    else if (frame.id === '0x130') idColorClass = 'style="color: var(--color-battery)"';

    row.innerHTML =
      `<span class="can-col-ts">${frame.t || ''}</span>` +
      `<span class="can-col-id" ${idColorClass}>${frame.id || ''}</span>` +
      `<span class="can-col-name">${frame.name || ''}</span>` +
      `<span class="can-col-hex">${frame.hex || ''}</span>` +
      `<span class="can-col-parsed">{ ${parsedStr} }</span>`;

    framesEl.insertBefore(row, framesEl.firstChild);
    canFrameCount++;
  });

  // Trim rows
  while (framesEl.children.length > MAX_CAN_ROWS) {
    framesEl.removeChild(framesEl.lastChild);
  }
}

// ── Chart ─────────────────────────────────────────────
function initChart() {
  const canvas = document.getElementById('telemetry-chart');
  if (!canvas) return;

  const ctx = canvas.getContext('2d');
  
  // Create cyber gradients for chart area fills
  const speedGrad = ctx.createLinearGradient(0, 0, 0, 180);
  speedGrad.addColorStop(0, 'rgba(0, 245, 255, 0.18)');
  speedGrad.addColorStop(1, 'rgba(0, 245, 255, 0.0)');

  const torqueGrad = ctx.createLinearGradient(0, 0, 0, 180);
  torqueGrad.addColorStop(0, 'rgba(255, 165, 0, 0.15)');
  torqueGrad.addColorStop(1, 'rgba(255, 165, 0, 0.0)');

  const tempGrad = ctx.createLinearGradient(0, 0, 0, 180);
  tempGrad.addColorStop(0, 'rgba(255, 45, 85, 0.12)');
  tempGrad.addColorStop(1, 'rgba(255, 45, 85, 0.0)');

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
          backgroundColor: speedGrad,
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.3,
          fill: true,
          yAxisID: 'y',
        },
        {
          label: 'Torque/2 (Nm÷2)',
          data: emptyData(),
          borderColor: '#ffa500',
          backgroundColor: torqueGrad,
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.3,
          fill: true,
          yAxisID: 'y',
        },
        {
          label: 'Motor Temp (°C)',
          data: emptyData(),
          borderColor: '#ff2d55',
          backgroundColor: tempGrad,
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.3,
          fill: true,
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
          display: false, // Custom legends configured in HTML
        },
        tooltip: {
          backgroundColor: 'rgba(6, 9, 19, 0.95)',
          borderColor: 'rgba(0, 245, 255, 0.25)',
          borderWidth: 1,
          titleColor: '#00f5ff',
          bodyColor: '#e2e8f0',
          titleFont: { family: "'Orbitron', sans-serif", size: 10, weight: 'bold' },
          bodyFont: { family: "'Inter', sans-serif", size: 10 },
          padding: 8,
          cornerRadius: 6,
        },
      },
      scales: {
        x: {
          display: false,
          grid: { display: false },
        },
        y: {
          position: 'left',
          grid: { color: 'rgba(255, 255, 255, 0.03)' },
          ticks: {
            color: 'rgba(255, 255, 255, 0.4)',
            font: { family: "'Orbitron', sans-serif", size: 9 },
            maxTicksLimit: 6,
          },
          title: {
            display: true,
            text: 'SPEED / TORQUE',
            color: 'rgba(255, 255, 255, 0.25)',
            font: { family: "'Inter', sans-serif", size: 9, weight: 'bold' },
          },
        },
        y2: {
          position: 'right',
          grid: { drawOnChartArea: false },
          ticks: {
            color: 'rgba(255, 45, 85, 0.5)',
            font: { family: "'Orbitron', sans-serif", size: 9 },
            maxTicksLimit: 5,
          },
          title: {
            display: true,
            text: 'TEMP (°C)',
            color: 'rgba(255, 45, 85, 0.35)',
            font: { family: "'Inter', sans-serif", size: 9, weight: 'bold' },
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
    listEl.innerHTML = '<div class="dtc-empty">No active DTCs in registry</div>';
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
  const hasInjected = [...document.querySelectorAll('.fault-switch-card input:checked')].length > 0;

  // Counter badge on faults deck
  const counterEl = document.getElementById('active-faults-counter');
  const checkCount = document.querySelectorAll('.fault-switch-card input:checked').length;
  if (counterEl) {
    counterEl.textContent = `${checkCount} Active`;
    if (checkCount > 0) {
      counterEl.classList.add('has-active');
    } else {
      counterEl.classList.remove('has-active');
    }
  }

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
      ? `SYSTEM FAULT: ${msgs.join(' | ')}`
      : 'SYSTEM FAULT DETECTED';
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

  // Update switch deck glow styles
  const checkboxId = {
    wire_cut:         'fault-wire-cut',
    thermal_runaway:  'fault-thermal',
    overspeed:        'fault-overspeed',
    throttle_sensor:  'fault-throttle-sensor',
  }[faultName];

  if (checkboxId) {
    const cardEl = document.getElementById(checkboxId)?.closest('.fault-switch-card');
    if (cardEl) {
      if (active) {
        cardEl.classList.add('active');
      } else {
        cardEl.classList.remove('active');
      }
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
  try {
    for (const ecu of ['bms', 'vcu', 'mcu']) {
      await fetch('/uds', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ecu, payload: '14FFFFFF' }),
      });
    }
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
  const btn = event?.currentTarget || event?.target;
  let origText = '';
  if (btn) { 
    origText = btn.innerHTML;
    btn.disabled = true; 
    btn.innerHTML = '<i class="fa-solid fa-spinner fa-spin"></i> QUERYING...'; 
  }

  const udsCmd = UDS_COMMANDS[command];
  if (!udsCmd) {
    showUDSResponse(`Unknown UDS command: ${command}`);
    if (btn) { btn.disabled = false; btn.innerHTML = origText; }
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
      btn.innerHTML = origText;
    }
  }
}

function showUDSResponse(text) {
  const el = document.getElementById('uds-response');
  if (!el) return;
  el.textContent = text;
  el.classList.add('visible');
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
    trackEl.setAttribute('d', arcPath(SPD_CX, SPD_CY, SPD_R, SPD_START_DEG, SPD_START_DEG + SPD_SWEEP_DEG));
  }
  updateSpeedometer(0);
});
