/**
 * Tactical Cyber HUD - Client Application JavaScript
 * drone_turret_v2 (Milestone M5)
 */

(function () {
  "use strict";

  // State
  const state = {
    audioEnabled: true,
    safetyArmed: false,
    wsConnected: false,
    lastLockState: false,
    config: {},
  };

  // Web Audio Context for synthesized sound cues
  let audioCtx = null;

  function initAudio() {
    if (!audioCtx) {
      const AudioContext = window.AudioContext || window.webkitAudioContext;
      if (AudioContext) {
        audioCtx = new AudioContext();
      }
    }
  }

  function playTone(freq, type, duration, gainVal = 0.1) {
    if (!state.audioEnabled) return;
    try {
      initAudio();
      if (!audioCtx) return;
      if (audioCtx.state === "suspended") {
        audioCtx.resume();
      }
      const osc = audioCtx.createOscillator();
      const gain = audioCtx.createGain();

      osc.type = type || "sine";
      osc.frequency.setValueAtTime(freq, audioCtx.currentTime);

      gain.gain.setValueAtTime(gainVal, audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.0001, audioCtx.currentTime + duration);

      osc.connect(gain);
      gain.connect(audioCtx.destination);

      osc.start();
      osc.stop(audioCtx.currentTime + duration);
    } catch (e) {
      console.debug("Audio play error:", e);
    }
  }

  function playLockBeep() {
    playTone(880, "sawtooth", 0.15, 0.12);
    setTimeout(() => playTone(1320, "sine", 0.20, 0.15), 100);
  }

  function playLockLostTone() {
    playTone(440, "triangle", 0.25, 0.1);
  }

  function playFireBlast() {
    playTone(150, "square", 0.35, 0.25);
    setTimeout(() => playTone(80, "sawtooth", 0.40, 0.20), 80);
  }

  // DOM Elements
  const el = {
    lockPill: document.getElementById("global-lock-pill"),
    lockStatusText: document.getElementById("lock-status-text"),
    hardwareModePill: document.getElementById("hardware-mode-pill"),
    hardwareModeText: document.getElementById("hardware-mode-text"),
    headerFps: document.getElementById("header-fps"),
    headerTime: document.getElementById("header-time"),
    soundToggle: document.getElementById("btn-sound-toggle"),
    soundIcon: document.getElementById("sound-icon"),

    // Telemetry
    valSpeed: document.getElementById("val-speed"),
    barSpeed: document.getElementById("bar-speed"),
    valMach: document.getElementById("val-mach"),
    valSpeedMps: document.getElementById("val-speed-mps"),
    valRange: document.getElementById("val-range"),
    barRange: document.getElementById("bar-range"),
    valRangeSrc: document.getElementById("val-range-src"),
    valTrackId: document.getElementById("val-track-id"),
    valConf: document.getElementById("val-conf"),
    valClass: document.getElementById("val-class"),
    valFilterState: document.getElementById("val-filter-state"),

    // Ballistics
    valTint: document.getElementById("val-tint"),
    valDrop: document.getElementById("val-drop"),
    valLeadPan: document.getElementById("val-lead-pan"),
    valLeadTilt: document.getElementById("val-lead-tilt"),
    needlePan: document.getElementById("needle-pan"),
    needleTilt: document.getElementById("needle-tilt"),
    readoutPan: document.getElementById("readout-pan"),
    readoutTilt: document.getElementById("readout-tilt"),

    // Actions
    safetySwitch: document.getElementById("safety-arm-switch"),
    safetyLabelText: document.getElementById("safety-label-text"),
    btnFire: document.getElementById("btn-fire-solenoid"),
    btnHome: document.getElementById("btn-home-turret"),
    logStream: document.getElementById("event-log-stream"),
    btnClearLog: document.getElementById("btn-clear-log"),

    // Video
    liveImg: document.getElementById("live-stream-img"),
    btnFullscreen: document.getElementById("btn-fullscreen"),
    btnSnapshot: document.getElementById("btn-snapshot"),
    btnReconnectStream: document.getElementById("btn-reconnect-stream"),
    streamResLabel: document.getElementById("stream-res-label"),
    streamModelLabel: document.getElementById("stream-model-label"),

    // Controls
    selectCamera: document.getElementById("select-camera"),
    btnRefreshCameras: document.getElementById("btn-refresh-cameras"),
    selectModel: document.getElementById("select-model"),
    selectDronePreset: document.getElementById("select-drone-preset"),
    checkSimMode: document.getElementById("check-sim-mode"),
    selectArduinoPort: document.getElementById("select-arduino-port"),
    btnRefreshPorts: document.getElementById("btn-refresh-ports"),

    // Sliders
    sliderConf: document.getElementById("slider-conf"),
    valSliderConf: document.getElementById("val-slider-conf"),
    sliderV0: document.getElementById("slider-v0"),
    valSliderV0: document.getElementById("val-slider-v0"),
    sliderMass: document.getElementById("slider-mass"),
    valSliderMass: document.getElementById("val-slider-mass"),
    sliderCd: document.getElementById("slider-cd"),
    valSliderCd: document.getElementById("val-slider-cd"),
    sliderKp: document.getElementById("slider-kp"),
    valSliderKp: document.getElementById("val-slider-kp"),
    sliderKd: document.getElementById("slider-kd"),
    valSliderKd: document.getElementById("val-slider-kd"),
    btnSaveParams: document.getElementById("btn-save-params"),
    btnResetParams: document.getElementById("btn-reset-params"),
  };

  // Helper: append event log
  function logEvent(msg, type = "sys") {
    if (!el.logStream) return;
    const now = new Date();
    const ts = now.toISOString().substring(11, 19);
    const div = document.createElement("div");
    div.className = `log-entry log-${type}`;
    div.textContent = `[${ts}] ${msg}`;
    el.logStream.appendChild(div);
    el.logStream.scrollTop = el.logStream.scrollHeight;
  }

  // Update Clock
  setInterval(() => {
    if (el.headerTime) {
      const now = new Date();
      el.headerTime.textContent = now.toISOString().substring(11, 19) + " UTC";
    }
  }, 1000);

  // ===========================================================================
  // WebSocket Telemetry Connection
  // ===========================================================================
  let ws = null;

  function connectTelemetryWS() {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const wsUrl = `${protocol}//${window.location.host}/ws/telemetry`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
      state.wsConnected = true;
      logEvent("Telemetry WebSocket connected at 30Hz stream.", "sys");
    };

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        updateTelemetryUI(data);
      } catch (err) {
        console.debug("WS parse error:", err);
      }
    };

    ws.onclose = () => {
      state.wsConnected = false;
      setTimeout(connectTelemetryWS, 2000);
    };

    ws.onerror = (err) => {
      console.debug("WS error:", err);
      ws.close();
    };
  }

  // Telemetry UI Updater
  function updateTelemetryUI(data) {
    if (!data) return;

    // FPS
    if (el.headerFps && data.fps !== undefined) {
      el.headerFps.textContent = Number(data.fps).toFixed(1);
    }

    // Lock Status & Badge
    const isLocked = Boolean(data.target_locked);
    const trackingState = (data.tracking_state || "SEARCHING").toUpperCase();

    if (isLocked && trackingState === "LOCKED") {
      el.lockPill.className = "status-pill status-locked";
      el.lockStatusText.textContent = `● TARGET LOCKED (TRK #${data.track_id || 1})`;
      if (!state.lastLockState) {
        logEvent(`TARGET ACQUIRED: ${data.target_class || "Drone"} #${data.track_id || 1} [Range: ${data.distance_m || 0}m]`, "lock");
        playLockBeep();
      }
      state.lastLockState = true;
    } else if (trackingState === "COASTING") {
      el.lockPill.className = "status-pill status-coasting";
      el.lockStatusText.textContent = "▲ TARGET COASTING (OCCLUDED)";
    } else {
      el.lockPill.className = "status-pill status-searching";
      el.lockStatusText.textContent = "◌ SCANNING AIRSPACE";
      if (state.lastLockState) {
        logEvent("TARGET LOST: Resuming optical radar search.", "warn");
        playLockLostTone();
      }
      state.lastLockState = false;
    }

    // Speed
    const speedKmh = Number(data.speed_kmh || 0);
    if (el.valSpeed) el.valSpeed.textContent = speedKmh.toFixed(1);
    if (el.barSpeed) {
      const pct = Math.min(100, Math.max(0, (speedKmh / 200.0) * 100));
      el.barSpeed.style.width = pct + "%";
    }
    const speedMps = speedKmh / 3.6;
    if (el.valSpeedMps) el.valSpeedMps.textContent = speedMps.toFixed(1);
    if (el.valMach) el.valMach.textContent = (speedMps / 343.0).toFixed(2);

    // Range
    const distM = Number(data.distance_m || 0);
    if (el.valRange) el.valRange.textContent = distM.toFixed(1);
    if (el.barRange) {
      const rPct = Math.min(100, Math.max(0, (distM / 60.0) * 100));
      el.barRange.style.width = rPct + "%";
    }
    if (el.valRangeSrc) el.valRangeSrc.textContent = (data.distance_source || "OPTICAL").toUpperCase();

    // Target Details
    if (el.valTrackId) el.valTrackId.textContent = data.track_id ? `#${data.track_id}` : "NONE";
    if (el.valConf) el.valConf.textContent = data.confidence ? Math.round(data.confidence * 100) + "%" : "--%";
    if (el.valClass) el.valClass.textContent = data.target_class || "--";
    if (el.valFilterState) el.valFilterState.textContent = trackingState;

    // Ballistics
    if (el.valTint) el.valTint.textContent = data.intercept_time_s ? `${Number(data.intercept_time_s).toFixed(2)} s` : "-- s";
    if (el.valDrop) el.valDrop.textContent = data.drop_m ? `${Number(data.drop_m).toFixed(2)} m` : "-- m";
    if (el.valLeadPan) el.valLeadPan.textContent = `${Number(data.lead_pan_angle || 90.0).toFixed(1)}°`;
    if (el.valLeadTilt) el.valLeadTilt.textContent = `${Number(data.lead_tilt_angle || 90.0).toFixed(1)}°`;

    // Gimbal attitude needles
    const panAngle = Number(data.pan_angle || 90.0);
    const tiltAngle = Number(data.tilt_angle || 90.0);
    if (el.needlePan) el.needlePan.style.left = `${(panAngle / 180.0) * 100}%`;
    if (el.needleTilt) el.needleTilt.style.left = `${(tiltAngle / 180.0) * 100}%`;
    if (el.readoutPan) el.readoutPan.textContent = `${panAngle.toFixed(1)}°`;
    if (el.readoutTilt) el.readoutTilt.textContent = `${tiltAngle.toFixed(1)}°`;

    // Hardware status
    if (el.hardwareModeText) {
      if (data.simulation_mode) {
        el.hardwareModeText.textContent = "SIMULATION MODE";
      } else {
        el.hardwareModeText.textContent = data.arduino_connected ? "ARDUINO ONLINE" : "HARDWARE OFFLINE";
      }
    }
  }

  // ===========================================================================
  // API Calls & Dynamic Config
  // ===========================================================================

  async function fetchConfig() {
    try {
      const res = await fetch("/api/config");
      if (res.ok) {
        const cfg = await res.json();
        state.config = cfg;
        populateConfigUI(cfg);
      }
    } catch (e) {
      console.debug("Failed to fetch config:", e);
    }
  }

  function populateConfigUI(cfg) {
    if (!cfg) return;

    if (cfg.confidence !== undefined) {
      el.sliderConf.value = cfg.confidence;
      el.valSliderConf.textContent = Math.round(cfg.confidence * 100) + "%";
    }
    if (cfg.muzzle_velocity !== undefined) {
      el.sliderV0.value = cfg.muzzle_velocity;
      el.valSliderV0.textContent = cfg.muzzle_velocity + " m/s";
    }
    if (cfg.net_mass !== undefined) {
      el.sliderMass.value = cfg.net_mass;
      el.valSliderMass.textContent = Number(cfg.net_mass).toFixed(2) + " kg";
    }
    if (cfg.net_cd !== undefined) {
      el.sliderCd.value = cfg.net_cd;
      el.valSliderCd.textContent = Number(cfg.net_cd).toFixed(2);
    }
    if (cfg.pid_kp !== undefined) {
      el.sliderKp.value = cfg.pid_kp;
      el.valSliderKp.textContent = Number(cfg.pid_kp).toFixed(2);
    }
    if (cfg.pid_kd !== undefined) {
      el.sliderKd.value = cfg.pid_kd;
      el.valSliderKd.textContent = Number(cfg.pid_kd).toFixed(3);
    }
    if (cfg.model_name && el.selectModel) {
      el.selectModel.value = cfg.model_name;
      if (el.streamModelLabel) el.streamModelLabel.textContent = `MODEL: ${cfg.model_name}`;
    }
    if (cfg.simulation_mode !== undefined && el.checkSimMode) {
      el.checkSimMode.checked = cfg.simulation_mode;
    }
  }

  async function postConfig(payload) {
    try {
      const res = await fetch("/api/config", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      if (res.ok) {
        const json = await res.json();
        state.config = json.config;
        logEvent(`Config updated: ${Object.keys(payload).join(", ")}`, "sys");
      }
    } catch (e) {
      logEvent(`Failed to update config: ${e.message}`, "warn");
    }
  }

  async function fetchCameras() {
    try {
      const res = await fetch("/api/cameras");
      if (res.ok) {
        const list = await res.json();
        el.selectCamera.innerHTML = "";
        list.forEach((cam) => {
          const opt = document.createElement("option");
          opt.value = cam.id;
          opt.textContent = `${cam.name} (${cam.width}x${cam.height})`;
          el.selectCamera.appendChild(opt);
        });
        logEvent(`Scanned ${list.length} video capture device(s).`, "sys");
      }
    } catch (e) {
      console.debug("Failed to list cameras:", e);
    }
  }

  async function fetchSerialPorts() {
    try {
      const res = await fetch("/api/hardware/ports");
      if (res.ok) {
        const data = await res.json();
        el.selectArduinoPort.innerHTML = "";
        (data.ports || []).forEach((p) => {
          const opt = document.createElement("option");
          opt.value = p.port;
          opt.textContent = `${p.port} (${p.description})`;
          el.selectArduinoPort.appendChild(opt);
        });
        logEvent(`Scanned COM ports: ${data.ports.length} available.`, "sys");
      }
    } catch (e) {
      console.debug("Failed to list ports:", e);
    }
  }

  // ===========================================================================
  // Event Listeners
  // ===========================================================================

  // Audio Toggle
  el.soundToggle.addEventListener("click", () => {
    state.audioEnabled = !state.audioEnabled;
    el.soundIcon.textContent = state.audioEnabled ? "🔊" : "🔇";
    el.soundToggle.style.color = state.audioEnabled ? "var(--accent-cyan)" : "var(--text-muted)";
    logEvent(`Audio feedback ${state.audioEnabled ? "ENABLED" : "MUTED"}.`, "sys");
  });

  // Safety Arm Switch
  el.safetySwitch.addEventListener("change", (e) => {
    state.safetyArmed = e.target.checked;
    if (state.safetyArmed) {
      el.btnFire.disabled = false;
      el.btnFire.classList.remove("btn-disabled");
      el.safetyLabelText.innerHTML = `SAFETY ARMED: <b class="text-green">READY TO FIRE</b>`;
      logEvent("⚠️ SAFETY DISENGAGED — TURRET WEAPON ARMED!", "fire");
      playTone(1000, "square", 0.3, 0.15);
    } else {
      el.btnFire.disabled = true;
      el.btnFire.classList.add("btn-disabled");
      el.safetyLabelText.innerHTML = `SAFETY ARMED: <b class="text-red">LOCKED</b>`;
      logEvent("Safety engaged. Firing trigger locked.", "sys");
    }
  });

  // FIRE Button — Human-in-the-loop two-step confirmation
  let fireArmed = false;
  let fireTimeout = null;

  el.btnFire.addEventListener("click", async () => {
    if (!state.safetyArmed) return;

    if (!fireArmed) {
      // STEP 1: ARM — request fire authorization
      try {
        const res = await fetch("/api/turret/fire", { method: "POST" });
        if (res.ok) {
          fireArmed = true;
          el.btnFire.querySelector(".fire-text").textContent = "⚠️ CONFIRM FIRE (5s)";
          el.btnFire.style.background = "#b00";
          el.btnFire.style.animation = "pulse 0.5s infinite";
          logEvent("⚠️ FIRE ARMED — Click again within 5 seconds to CONFIRM launch!", "fire");
          playTone(800, "square", 0.2, 0.1);

          // Auto-disarm after 5 seconds
          fireTimeout = setTimeout(() => {
            fireArmed = false;
            el.btnFire.querySelector(".fire-text").textContent = "FIRE NET LAUNCHER";
            el.btnFire.style.background = "";
            el.btnFire.style.animation = "";
            logEvent("Fire authorization expired. Re-arm to fire.", "sys");
          }, 5000);
        }
      } catch (err) {
        logEvent(`Fire arm failed: ${err.message}`, "warn");
      }
    } else {
      // STEP 2: CONFIRM — execute fire
      clearTimeout(fireTimeout);
      fireArmed = false;
      el.btnFire.querySelector(".fire-text").textContent = "FIRE NET LAUNCHER";
      el.btnFire.style.background = "";
      el.btnFire.style.animation = "";

      playFireBlast();
      logEvent("💥 FIRE CONFIRMED: Launching pneumatic net interceptor!", "fire");
      el.btnFire.style.transform = "scale(0.96)";
      setTimeout(() => { el.btnFire.style.transform = ""; }, 150);

      try {
        const res = await fetch("/api/turret/fire/confirm", { method: "POST" });
        if (res.ok) {
          const json = await res.json();
          logEvent(`Launch result: ${json.status}`, json.status === "fired" ? "fire" : "warn");
        }
      } catch (err) {
        logEvent(`Firing trigger failed: ${err.message}`, "warn");
      }
    }
  });

  // HOME Turret
  el.btnHome.addEventListener("click", async () => {
    try {
      const res = await fetch("/api/turret/home", { method: "POST" });
      if (res.ok) {
        logEvent("Turret servos homed to 90°, 90°.", "sys");
      }
    } catch (e) {
      logEvent(`Homing failed: ${e.message}`, "warn");
    }
  });

  // Clear Log
  el.btnClearLog.addEventListener("click", () => {
    el.logStream.innerHTML = "";
  });

  // Fullscreen
  el.btnFullscreen.addEventListener("click", () => {
    const container = document.getElementById("video-frame-container");
    if (!document.fullscreenElement) {
      container.requestFullscreen().catch((err) => console.debug(err));
    } else {
      document.exitFullscreen();
    }
  });

  // Reconnect Stream
  el.btnReconnectStream.addEventListener("click", () => {
    if (el.liveImg) {
      el.liveImg.src = "/video_feed?t=" + new Date().getTime();
      logEvent("Video stream reconnected.", "sys");
    }
  });

  // Snapshot
  el.btnSnapshot.addEventListener("click", () => {
    const a = document.createElement("a");
    a.href = "/video_feed";
    a.download = `hud_snapshot_${Date.now()}.jpg`;
    logEvent("Snapshot captured.", "sys");
  });

  // Slider Input Listeners
  el.sliderConf.addEventListener("input", (e) => {
    el.valSliderConf.textContent = Math.round(e.target.value * 100) + "%";
  });
  el.sliderV0.addEventListener("input", (e) => {
    el.valSliderV0.textContent = e.target.value + " m/s";
  });
  el.sliderMass.addEventListener("input", (e) => {
    el.valSliderMass.textContent = Number(e.target.value).toFixed(2) + " kg";
  });
  el.sliderCd.addEventListener("input", (e) => {
    el.valSliderCd.textContent = Number(e.target.value).toFixed(2);
  });
  el.sliderKp.addEventListener("input", (e) => {
    el.valSliderKp.textContent = Number(e.target.value).toFixed(2);
  });
  el.sliderKd.addEventListener("input", (e) => {
    el.valSliderKd.textContent = Number(e.target.value).toFixed(3);
  });

  // Apply Parameters Button
  el.btnSaveParams.addEventListener("click", () => {
    const payload = {
      confidence: parseFloat(el.sliderConf.value),
      muzzle_velocity: parseFloat(el.sliderV0.value),
      net_mass: parseFloat(el.sliderMass.value),
      net_cd: parseFloat(el.sliderCd.value),
      pid_kp: parseFloat(el.sliderKp.value),
      pid_kd: parseFloat(el.sliderKd.value),
    };
    postConfig(payload);
  });

  // Reset Parameters
  el.btnResetParams.addEventListener("click", () => {
    const defaults = {
      confidence: 0.50,
      muzzle_velocity: 80.0,
      net_mass: 0.60,
      net_cd: 1.20,
      pid_kp: 0.12,
      pid_kd: 0.03,
    };
    populateConfigUI(defaults);
    postConfig(defaults);
  });

  // Model Selector
  el.selectModel.addEventListener("change", (e) => {
    postConfig({ model_name: e.target.value });
    if (el.streamModelLabel) el.streamModelLabel.textContent = `MODEL: ${e.target.value}`;
  });

  // Camera Selector
  el.selectCamera.addEventListener("change", (e) => {
    postConfig({ camera_id: e.target.value });
  });

  // Drone Preset Selector
  el.selectDronePreset.addEventListener("change", (e) => {
    if (e.target.value !== "custom") {
      postConfig({ drone_real_size: parseFloat(e.target.value) });
    }
  });

  // Sim Mode Toggle
  el.checkSimMode.addEventListener("change", (e) => {
    postConfig({ simulation_mode: e.target.checked });
  });

  // Arduino Port Selector
  el.selectArduinoPort.addEventListener("change", (e) => {
    postConfig({ arduino_port: e.target.value });
  });

  // Refresh Buttons
  el.btnRefreshCameras.addEventListener("click", fetchCameras);
  el.btnRefreshPorts.addEventListener("click", fetchSerialPorts);

  // Initialize
  window.addEventListener("DOMContentLoaded", () => {
    fetchConfig();
    fetchCameras();
    fetchSerialPorts();
    connectTelemetryWS();
    logEvent("Combat HUD GUI initialized & connected.", "sys");
  });
})();
