const CHANNELS = ["ecg", "emg1", "emg2", "emg3"];
const CHANNEL_COLORS = {
  ecg: "--coral",
  emg1: "--amber",
  emg2: "--blue",
  emg3: "--violet",
};
const MAX_POINTS = 2500;
const DISPLAY_POINTS = 650;
const UI_UPDATE_INTERVAL = 100;
const MOTION_UPDATE_INTERVAL = 50;
const DRAW_INTERVAL_MS = 50;
const DEFAULT_SETTINGS = {
  hrLow: 50,
  hrHigh: 120,
  ecgLimit: 2,
  emg1Limit: 0.5,
  emg2Limit: 0.5,
  emg3Limit: 0.5,
  disconnectMs: 2000,
  ecgRules: true,
  emergencyPhone: "",
  emergencySmsEnabled: false,
  aiVoiceEnabled: true,
};

const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => [...document.querySelectorAll(selector)];
const elements = {
  statusPill: $("#statusPill"),
  deviceMeta: $("#deviceMeta"),
  socketUrl: $("#socketUrl"),
  connectButton: $("#connectButton"),
  demoButton: $("#demoButton"),
  recordButton: $("#recordButton"),
  pauseButton: $("#pauseButton"),
  taskName: $("#taskName"),
  taskNote: $("#taskNote"),
  taskStateText: $("#taskStateText"),
  taskDetail: $("#taskDetail"),
  heartRate: $("#heartRate"),
  heartState: $("#heartState"),
  emgRms: $("#emgRms"),
  emgState: $("#emgState"),
  emgLoadBar: $("#emgLoadBar"),
  signalQuality: $("#signalQuality"),
  qualityState: $("#qualityState"),
  packetLoss: $("#packetLoss"),
  sampleRate: $("#sampleRate"),
  recordState: $("#recordState"),
  captureDuration: $("#captureDuration"),
  sampleCounter: $("#sampleCounter"),
  alarmBanner: $("#alarmBanner"),
  alarmText: $("#alarmText"),
  aiProcessingOverlay: $("#aiProcessingOverlay"),
  toast: $("#toast"),
  tasksBody: $("#tasksBody"),
  emptyRecords: $("#emptyRecords"),
  replayPanel: $("#replayPanel"),
  replayRange: $("#replayRange"),
  aiStatus: $("#aiStatus"),
  aiTask: $("#aiTask"),
  aiReport: $("#aiReport"),
  generateReport: $("#generateReport"),
  chatLog: $("#chatLog"),
  chatInput: $("#chatInput"),
  sendChat: $("#sendChat"),
  liveRiskState: $("#liveRiskState"),
  livePostureState: $("#livePostureState"),
  liveAiSummary: $("#liveAiSummary"),
  voiceOrb: $("#voiceOrb"),
  voiceToggle: $("#voiceToggle"),
};

const state = {
  mode: "idle",
  socket: null,
  sampleRate: 500,
  buffers: Object.fromEntries(CHANNELS.map((channel) => [channel, new Float32Array(MAX_POINTS)])),
  writeIndex: 0,
  filled: 0,
  sampleCount: 0,
  droppedPackets: 0,
  expectedSequence: null,
  lastSampleAt: 0,
  phase: 0,
  paused: false,
  currentFlags: 0,
  alarmMarkers: Object.fromEntries(CHANNELS.map((channel) => [channel, []])),
  latestGyro: [0, 0, 0],
  latestFall: 0,
  latestAngles: [0, 0, 0],
  latestTemp: 0,
  task: null,
  taskStartedAt: null,
  pendingRows: [],
  flushPromise: Promise.resolve(),
  settings: loadSettings(),
  history: null,
  dismissedAlarmAt: 0,
  alarmHoldUntil: 0,
  alarmAudio: null,
  alarmMedia: null,
  alarmAudioUnlocked: false,
  lastAlarmSoundAt: 0,
  fallActive: false,
  lastEmergencySmsAt: 0,
  aiHistory: [],
  lastAiSummaryAt: 0,
  liveAiBusy: false,
  voiceListening: false,
  aiRequestId: 0,
  pendingAiRequest: null,
};

function loadSettings() {
  try {
    return { ...DEFAULT_SETTINGS, ...JSON.parse(localStorage.getItem("bioscopeSettings") || "{}") };
  } catch {
    return { ...DEFAULT_SETTINGS };
  }
}

function showToast(message) {
  elements.toast.textContent = message;
  elements.toast.classList.add("show");
  clearTimeout(showToast.timer);
  showToast.timer = setTimeout(() => elements.toast.classList.remove("show"), 2800);
}

function setStatus(type, label, meta) {
  elements.statusPill.className = `status-pill ${type}`;
  elements.statusPill.querySelector("span").textContent = label;
  elements.deviceMeta.textContent = meta;
  updateControls();
}

function updateControls() {
  const dataSourceReady = state.mode === "live" || state.mode === "demo";
  elements.recordButton.disabled = !dataSourceReady && !state.task;
  if (elements.pauseButton) elements.pauseButton.disabled = !dataSourceReady;
}

function resetSignalData() {
  CHANNELS.forEach((channel) => {
    state.buffers[channel].fill(0);
    state.alarmMarkers[channel] = [];
  });
  state.writeIndex = 0;
  state.filled = 0;
  state.sampleCount = 0;
  state.droppedPackets = 0;
  state.expectedSequence = null;
  state.lastSampleAt = 0;
  state.phase = 0;
  state.currentFlags = 0;
  document.body.classList.remove("has-data");
  elements.heartRate.textContent = "--";
  elements.heartState.textContent = "等待数据";
  elements.emgRms.textContent = "--";
  elements.emgState.textContent = "等待数据";
  elements.emgLoadBar.style.width = "0";
  elements.signalQuality.textContent = "--";
  elements.qualityState.textContent = "等待数据";
  elements.packetLoss.textContent = "--";
  elements.sampleCounter.textContent = "0 samples";
  state.latestGyro = [0, 0, 0];
  state.latestFall = 0;
  state.fallActive = false;
  state.latestAngles = [0, 0, 0];
  state.latestTemp = 0;
  if ($("#gyroX")) $("#gyroX").textContent = "--";
  if ($("#gyroY")) $("#gyroY").textContent = "--";
  if ($("#gyroZ")) $("#gyroZ").textContent = "--";
  if ($("#fallState")) $("#fallState").textContent = "NORMAL";
  if ($("#rollValue")) $("#rollValue").textContent = "--";
  if ($("#pitchValue")) $("#pitchValue").textContent = "--";
  if ($("#yawValue")) $("#yawValue").textContent = "--";
  if ($("#tempValue")) $("#tempValue").textContent = "--";
  if ($("#gpsFixState")) $("#gpsFixState").textContent = "等待定位";
  if ($("#latitudeValue")) $("#latitudeValue").textContent = "--";
  if ($("#longitudeValue")) $("#longitudeValue").textContent = "--";
  CHANNELS.forEach((channel) => {
    $(`#${channel}Value`).textContent = "--";
  });
}

function getRecent(channel, count) {
  const length = Math.min(count, state.filled);
  const values = new Array(length);
  let index = (state.writeIndex - length + MAX_POINTS) % MAX_POINTS;
  for (let i = 0; i < length; i += 1) {
    values[i] = state.buffers[channel][index];
    index = (index + 1) % MAX_POINTS;
  }
  return values;
}

function calculateRms(channel, count = 250) {
  const values = getRecent(channel, count);
  if (!values.length) return 0;
  return Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length);
}

function estimateHeart() {
  const values = getRecent("ecg", Math.min(state.filled, state.sampleRate * 8));
  if (values.length < state.sampleRate * 2) return null;
  const threshold = Math.min(0.5, state.settings.ecgLimit * 0.35);
  const minDistance = state.sampleRate * 0.3;
  const peaks = [];
  for (let i = 1; i < values.length - 1; i += 1) {
    if (values[i] > threshold && values[i] > values[i - 1] && values[i] >= values[i + 1]) {
      if (!peaks.length || i - peaks[peaks.length - 1] > minDistance) peaks.push(i);
    }
  }
  if (peaks.length < 2) return null;
  const intervals = peaks.slice(1).map((peak, index) => peak - peaks[index]);
  const average = intervals.reduce((sum, value) => sum + value, 0) / intervals.length;
  const variance = intervals.reduce((sum, value) => sum + (value - average) ** 2, 0) / intervals.length;
  const irregular = intervals.length >= 3 && Math.sqrt(variance) / average > 0.2;
  const missedBeat = intervals.some((value) => value > average * 1.65);
  return {
    rate: Math.min(240, Math.max(20, 60 * state.sampleRate / average)),
    irregular,
    missedBeat,
  };
}

function computeAlarmFlags(latestValues) {
  let flags = Math.abs(latestValues[0]) > state.settings.ecgLimit ? 4 : 0;
  const heart = estimateHeart();
  if (heart) {
    if (heart.rate < state.settings.hrLow) flags |= 1;
    if (heart.rate > state.settings.hrHigh) flags |= 2;
    if (state.settings.ecgRules && (heart.irregular || heart.missedBeat)) flags |= 4;
  }
  ["emg1", "emg2", "emg3"].forEach((channel, index) => {
    if (calculateRms(channel) > state.settings[`${channel}Limit`]) flags |= 1 << (index + 3);
  });
  return flags;
}

function flagMessages(flags) {
  const messages = [];
  if (flags & 1) messages.push("心率过低");
  if (flags & 2) messages.push("心率过高");
  if (flags & 4) messages.push("心电异常");
  if (flags & 8) messages.push("EMG1 超限");
  if (flags & 16) messages.push("EMG2 超限");
  if (flags & 32) messages.push("EMG3 超限");
  if (flags & 64) messages.push("信号断开");
  if (flags & 128) messages.push("跌倒警告");
  return messages;
}

function createAlarmMedia() {
  if (state.alarmMedia) return state.alarmMedia;

  const sampleRate = 16000;
  const seconds = 1.05;
  const samples = Math.floor(sampleRate * seconds);
  const dataSize = samples * 2;
  const buffer = new ArrayBuffer(44 + dataSize);
  const view = new DataView(buffer);
  const writeString = (offset, text) => {
    for (let i = 0; i < text.length; i += 1) view.setUint8(offset + i, text.charCodeAt(i));
  };

  writeString(0, "RIFF");
  view.setUint32(4, 36 + dataSize, true);
  writeString(8, "WAVEfmt ");
  view.setUint32(16, 16, true);
  view.setUint16(20, 1, true);
  view.setUint16(22, 1, true);
  view.setUint32(24, sampleRate, true);
  view.setUint32(28, sampleRate * 2, true);
  view.setUint16(32, 2, true);
  view.setUint16(34, 16, true);
  writeString(36, "data");
  view.setUint32(40, dataSize, true);

  for (let i = 0; i < samples; i += 1) {
    const t = i / sampleRate;
    const burst = (t < 0.18) || (t >= 0.28 && t < 0.46) || (t >= 0.56 && t < 0.74);
    const freq = t < 0.46 ? 1480 : 1040;
    const envelope = burst ? 0.94 * Math.min(1, ((t % 0.28) + 0.02) * 45) : 0;
    const value = burst ? Math.sin(2 * Math.PI * freq * t) * envelope : 0;
    view.setInt16(44 + i * 2, Math.round(value * 32767), true);
  }

  state.alarmMedia = new Audio(URL.createObjectURL(new Blob([buffer], { type: "audio/wav" })));
  state.alarmMedia.preload = "auto";
  state.alarmMedia.volume = 1;
  return state.alarmMedia;
}

function getAlarmAudio() {
  const AudioContext = window.AudioContext || window.webkitAudioContext;
  if (!AudioContext) return null;
  state.alarmAudio = state.alarmAudio || new AudioContext();
  return state.alarmAudio;
}

function unlockAlarmSound() {
  const media = createAlarmMedia();
  media.volume = 0;
  media.currentTime = 0;
  media.play().then(() => {
    media.pause();
    media.currentTime = 0;
    media.volume = 1;
    state.alarmAudioUnlocked = true;
  }).catch(() => {
    media.volume = 1;
  });

  const audio = getAlarmAudio();
  if (audio) audio.resume().catch(() => {});
}

function playAlarmTone(frequency, startAt) {
  const audio = getAlarmAudio();
  if (!audio || audio.state !== "running") return;

  try {
    const oscillator = audio.createOscillator();
    const gain = audio.createGain();
    oscillator.type = "sawtooth";
    oscillator.frequency.setValueAtTime(frequency, startAt);
    gain.gain.setValueAtTime(0.0001, startAt);
    gain.gain.exponentialRampToValueAtTime(0.45, startAt + 0.015);
    gain.gain.exponentialRampToValueAtTime(0.0001, startAt + 0.22);
    oscillator.connect(gain);
    gain.connect(audio.destination);
    oscillator.start(startAt);
    oscillator.stop(startAt + 0.24);
  } catch {
    // Audio can still be blocked by browser policy until user interaction.
  }
}

function playAlarmSound() {
  const now = Date.now();
  if (now - state.lastAlarmSoundAt < 1050) return;
  state.lastAlarmSoundAt = now;

  // Android uses the dedicated alarm stream, which remains audible when WebView media is restricted.
  if (window.AndroidHost?.playFallAlarm) {
    window.AndroidHost.playFallAlarm();
    return;
  }
  unlockAlarmSound();

  const media = createAlarmMedia();
  media.volume = 1;
  media.currentTime = 0;
  media.play().catch(() => {});

  const audio = getAlarmAudio();
  if (!audio || audio.state !== "running") return;
  playAlarmTone(1480, audio.currentTime);
  playAlarmTone(1480, audio.currentTime + 0.28);
  playAlarmTone(1040, audio.currentTime + 0.56);
}

function updateAlarm(flags) {
  const fallDetected = Boolean(flags & 128);
  if (!fallDetected) {
    if (Date.now() >= state.alarmHoldUntil) elements.alarmBanner.classList.remove("show");
    return;
  }
  state.alarmHoldUntil = Date.now() + 5000;
  if (Date.now() - state.dismissedAlarmAt < 2500) return;
  elements.alarmText.textContent = "检测到跌倒，请立即确认安全";
  elements.alarmBanner.classList.add("show");
  playAlarmSound();
}

function updateVoiceToggle() {
  const enabled = Boolean(state.settings.aiVoiceEnabled);
  elements.voiceToggle?.classList.toggle("enabled", enabled);
  elements.voiceToggle?.setAttribute("aria-pressed", String(enabled));
  if (elements.voiceToggle) elements.voiceToggle.querySelector("strong").textContent = enabled ? "开" : "关";
}

function setAiVoiceEnabled(enabled, announce = true) {
  state.settings.aiVoiceEnabled = enabled;
  localStorage.setItem("bioscopeSettings", JSON.stringify(state.settings));
  updateVoiceToggle();
  if (!enabled) window.AndroidHost?.stopSpeaking?.();
  if (announce) showToast(enabled ? "AI 语音朗读已开启" : "AI 语音朗读已关闭");
}

function sendFallSms() {
  if (!state.settings.emergencySmsEnabled || !state.settings.emergencyPhone) return;
  if (Date.now() - state.lastEmergencySmsAt < 60000) return;
  if (!window.AndroidHost?.sendEmergencySms) {
    showToast("紧急短信仅能在安卓 App 中发送");
    return;
  }
  state.lastEmergencySmsAt = Date.now();
  window.AndroidHost.sendEmergencySms(state.settings.emergencyPhone);
  showToast("已请求发送跌倒提醒短信");
}

function addSample(values, sequence, timestampUs = Date.now() * 1000) {
  const sample = [
    Number(values[0]),
    Number(values[1]),
    Number(values[2]),
    Number(values[3]),
    Number(values[4] ?? 0),
    Number(values[5] ?? 0),
    Number(values[6] ?? 0),
    Number(values[7] ?? 0) ? 1 : 0,
    Number(values[8] ?? 0),
    Number(values[9] ?? 0),
    Number(values[10] ?? 0),
    Number(values[11] ?? 0),
  ];
  if (state.paused || sample.some((value, index) => index !== 7 && !Number.isFinite(value))) return;
  if (Number.isInteger(sequence)) {
    if (state.expectedSequence !== null && sequence > state.expectedSequence) {
      state.droppedPackets += sequence - state.expectedSequence;
    }
    state.expectedSequence = sequence + 1;
  }

  CHANNELS.forEach((channel, index) => {
    state.buffers[channel][state.writeIndex] = sample[index];
  });
  state.latestGyro = sample.slice(4, 7);
  state.latestFall = sample[7];
  state.latestAngles = sample.slice(8, 11);
  state.latestTemp = sample[11];
  state.writeIndex = (state.writeIndex + 1) % MAX_POINTS;
  state.filled = Math.min(MAX_POINTS, state.filled + 1);
  state.sampleCount += 1;
  state.lastSampleAt = Date.now();
  document.body.classList.add("has-data");

  // ECG/EMG 阈值只用于显示与 AI 观察，不再触发报警；仅由跌倒检测结果报警。
  state.currentFlags = sample[7] ? 128 : 0;
  if (state.currentFlags) {
    if (state.currentFlags & 7) state.alarmMarkers.ecg.push(state.sampleCount);
    if (state.currentFlags & 8) state.alarmMarkers.emg1.push(state.sampleCount);
    if (state.currentFlags & 16) state.alarmMarkers.emg2.push(state.sampleCount);
    if (state.currentFlags & 32) state.alarmMarkers.emg3.push(state.sampleCount);
    CHANNELS.forEach((channel) => {
      if (state.alarmMarkers[channel].length > 200) state.alarmMarkers[channel].shift();
    });
  }

  if (state.task) {
    state.pendingRows.push([Math.round(timestampUs), ...sample, state.currentFlags]);
    if (state.pendingRows.length >= 500) flushSamples();
  }

  if (sample[7] || state.sampleCount % MOTION_UPDATE_INTERVAL === 0) updateMotionMetrics(sample);
  if (sample[7]) {
    updateAlarm(state.currentFlags);
    if (!state.fallActive) sendFallSms();
    state.fallActive = true;
  } else {
    state.fallActive = false;
  }
  if (state.sampleCount % UI_UPDATE_INTERVAL === 0) {
    updateMetrics(sample);
    updateAlarm(state.currentFlags);
  }
}

function updateMotionMetrics(values) {
  if ($("#gyroX")) $("#gyroX").textContent = values[4].toFixed(2);
  if ($("#gyroY")) $("#gyroY").textContent = values[5].toFixed(2);
  if ($("#gyroZ")) $("#gyroZ").textContent = values[6].toFixed(2);
  if ($("#rollValue")) $("#rollValue").textContent = values[8].toFixed(2);
  if ($("#pitchValue")) $("#pitchValue").textContent = values[9].toFixed(2);
  if ($("#yawValue")) $("#yawValue").textContent = values[10].toFixed(2);
  if ($("#tempValue")) $("#tempValue").textContent = values[11].toFixed(2);
  if ($("#fallState")) {
    $("#fallState").textContent = values[7] ? "FALL" : "NORMAL";
    $("#fallState").classList.toggle("danger", values[7] === 1);
  }
}

function updateMetrics(values) {
  CHANNELS.forEach((channel, index) => {
    $(`#${channel}Value`).textContent = values[index].toFixed(3);
  });
  const heart = estimateHeart();
  if (heart) {
    const rate = Math.round(heart.rate);
    elements.heartRate.textContent = rate;
    elements.heartState.textContent =
      rate < state.settings.hrLow ? "过低" : rate > state.settings.hrHigh ? "过高" : "正常";
  }
  const rmsValues = ["emg1", "emg2", "emg3"].map((channel) => calculateRms(channel));
  const maxRms = Math.max(...rmsValues);
  const maxLimit = Math.max(state.settings.emg1Limit, state.settings.emg2Limit, state.settings.emg3Limit);
  elements.emgRms.textContent = maxRms.toFixed(3);
  elements.emgState.textContent = rmsValues.some((value, index) => value > state.settings[`emg${index + 1}Limit`])
    ? "超限"
    : "正常";
  elements.emgLoadBar.style.width = `${Math.min(100, maxRms / maxLimit * 100)}%`;
  const expected = state.sampleCount + state.droppedPackets;
  const loss = expected ? state.droppedPackets / expected * 100 : 0;
  elements.packetLoss.textContent = `${loss.toFixed(2)}%`;
  elements.signalQuality.textContent = Math.max(0, Math.round(100 - loss * 2));
  elements.qualityState.textContent = loss < 1 ? "良好" : loss < 5 ? "一般" : "较差";
  elements.sampleCounter.textContent = `${state.sampleCount.toLocaleString()} samples`;
  updateLiveContext(heart, rmsValues, loss);
}

function buildRealtimeContext() {
  const heart = estimateHeart();
  const emg = ["emg1", "emg2", "emg3"].map((channel) => Number(calculateRms(channel).toFixed(4)));
  const gyroMagnitude = Math.hypot(...state.latestGyro);
  const postureDeviation = Math.max(Math.abs(state.latestAngles[0]), Math.abs(state.latestAngles[1]));
  const expected = state.sampleCount + state.droppedPackets;
  const lossPercent = expected ? Number((state.droppedPackets / expected * 100).toFixed(2)) : 0;
  return {
    source: "本地实时窗口约0.5秒，非连续原始波形",
    sampleCount: state.sampleCount,
    heartRateBpm: heart ? Math.round(heart.rate) : null,
    heartRhythmFlags: heart ? { irregular: heart.irregular, missedBeat: heart.missedBeat } : null,
    emgRmsMv: emg,
    emgActivity: Math.max(...emg) > 0.5 ? "较高" : Math.max(...emg) > 0.15 ? "中等" : "较低",
    postureDeg: { roll: Number(state.latestAngles[0].toFixed(1)), pitch: Number(state.latestAngles[1].toFixed(1)), yaw: Number(state.latestAngles[2].toFixed(1)) },
    gyroDps: Number(gyroMagnitude.toFixed(1)),
    postureDeviationDeg: Number(postureDeviation.toFixed(1)),
    fallDetected: Boolean(state.latestFall),
    alarmFlags: state.currentFlags,
    packetLossPercent: lossPercent,
    limitations: "仅供工程调试与教学研究；本地规则负责安全，不用于医学诊断或控制关节。",
  };
}

function updateLiveContext(heart, rmsValues, loss) {
  const context = buildRealtimeContext();
  const movement = context.gyroDps >= 80 ? "运动较明显" : context.gyroDps >= 30 ? "轻度运动" : "姿态相对平稳";
  const risk = context.fallDetected ? "跌倒报警已触发" : state.currentFlags ? "存在需要关注的阈值报警" : "未见本地风险报警";
  if (elements.liveRiskState) elements.liveRiskState.textContent = risk;
  if (elements.livePostureState) elements.livePostureState.textContent = `${movement} · 偏移 ${context.postureDeviationDeg}°`;
  if (elements.liveAiSummary && !state.liveAiBusy) {
    elements.liveAiSummary.textContent = `肌电${context.emgActivity} · ${movement} · ${risk}`;
  }
  if (state.sampleCount >= state.sampleRate * 4 && Date.now() - state.lastAiSummaryAt > 60000) {
    requestLiveAiSummary(context);
  }
}

async function requestLiveAiSummary(context = buildRealtimeContext()) {
  if (state.liveAiBusy || Date.now() - state.lastAiSummaryAt < 45000) return;
  state.liveAiBusy = true;
  state.lastAiSummaryAt = Date.now();
  if (elements.liveAiSummary) elements.liveAiSummary.textContent = "AI 正在生成实时简析…";
  if (window.AndroidHost?.requestRealtimeAi) {
    window.AndroidHost.requestRealtimeAi(JSON.stringify(context));
    return;
  }
  try {
    const { summary } = await api("/api/ai/realtime", { method: "POST", body: JSON.stringify({ context }) });
    if (elements.liveAiSummary) elements.liveAiSummary.textContent = summary;
  } catch {
    if (elements.liveAiSummary) elements.liveAiSummary.textContent = "实时数据已更新，等待 AI 简析。";
  } finally {
    state.liveAiBusy = false;
  }
}

window.onRealtimeAiSummary = (summary) => {
  state.liveAiBusy = false;
  if (elements.liveAiSummary) elements.liveAiSummary.textContent = summary;
};

window.onRealtimeAiError = () => {
  state.liveAiBusy = false;
  if (elements.liveAiSummary) elements.liveAiSummary.textContent = "实时数据已更新，等待 AI 简析。";
};

function parsePayload(payload) {
  if (payload instanceof Blob) {
    payload.text().then(parsePayload);
    return;
  }
  if (payload instanceof ArrayBuffer) {
    const data = new Float32Array(payload);
    const stride = data.length % 12 === 0 ? 12 : data.length % 8 === 0 ? 8 : 4;
    for (let i = 0; i + 3 < data.length; i += stride) addSample([...data.slice(i, i + stride)]);
    return;
  }
  const text = String(payload).trim();
  if (!text) return;
  if (text.startsWith("{") || text.startsWith("[")) {
    try {
      const message = JSON.parse(text);
      if (message.gps) updateGps(message.gps);
      if (message.type === "status") return;
      const samples = Array.isArray(message) ? message : message.samples;
      if (Array.isArray(samples)) {
        if (Number.isFinite(message.sampleRate)) setSampleRate(message.sampleRate);
        const baseTimestamp = Number(message.timestampUs) || Date.now() * 1000;
        samples.forEach((sample, index) => {
          const values = Array.isArray(sample)
            ? sample.slice(0, 12).map(Number)
            : [
                sample.ecg,
                sample.emg1,
                sample.emg2,
                sample.emg3,
                sample.gyroX ?? sample.gyro_x_dps ?? 0,
                sample.gyroY ?? sample.gyro_y_dps ?? 0,
                sample.gyroZ ?? sample.gyro_z_dps ?? 0,
                sample.fall ?? 0,
                sample.roll ?? sample.rollDeg ?? sample.roll_deg ?? 0,
                sample.pitch ?? sample.pitchDeg ?? sample.pitch_deg ?? 0,
                sample.yaw ?? sample.yawDeg ?? sample.yaw_deg ?? 0,
                sample.temperature ?? sample.temp ?? sample.temperatureC ?? sample.temperature_c ?? 0,
              ].map(Number);
          addSample(
            values,
            Number.isInteger(message.seqStart) ? message.seqStart + index : sample.seq,
            baseTimestamp + index * 1_000_000 / state.sampleRate,
          );
        });
        return;
      }
    } catch {
      // Fall back to comma-separated lines.
    }
  }
  text.split(/\r?\n/).forEach((line) => {
    const values = line.split(",").slice(0, 12).map(Number);
    if (values.length >= 4) addSample(values);
  });
}

window.onPhoneLocation = (latitude, longitude, accuracy) => {
  const gps = { fix: true, latitude: Number(latitude), longitude: Number(longitude), accuracy, source: "phone" };
  updateGps(gps);
  if (state.socket?.readyState === WebSocket.OPEN) {
    state.socket.send(JSON.stringify({ type: "phoneLocation", ...gps }));
  }
};

function updateGps(gps) {
  const latitude = Number(gps?.latitude);
  const longitude = Number(gps?.longitude);
  const rxBytes = Number(gps?.rxBytes ?? 0);
  const validSentences = Number(gps?.validSentences ?? 0);
  const satellites = Number(gps?.satellites ?? 0);
  const fixed = (gps?.fix === true || Number(gps?.fix) > 0)
    && Number.isFinite(latitude)
    && Number.isFinite(longitude);

  $("#gpsFixState").textContent = fixed
    ? "已定位"
    : rxBytes === 0
      ? "GPS无串口数据"
      : validSentences === 0
        ? "收到数据但解析失败"
        : `已收到数据，等待定位（${satellites}颗卫星）`;
  $("#latitudeValue").textContent = fixed ? latitude.toFixed(6) : "--";
  $("#longitudeValue").textContent = fixed ? longitude.toFixed(6) : "--";
}

function setSampleRate(value) {
  state.sampleRate = Math.max(1, Math.round(value));
  elements.sampleRate.textContent = state.sampleRate;
}

function createDemoBatch() {
  if (state.mode !== "demo" || state.paused) return;
  const period = state.sampleRate * 60 / 72;
  const batchTimestamp = Date.now() * 1000;
  for (let i = 0; i < 10; i += 1) {
    const p = state.phase % period;
    const ecg =
      1.05 * Math.exp(-0.5 * ((p - period * 0.18) / 4.2) ** 2) -
      0.16 * Math.exp(-0.5 * ((p - period * 0.16) / 3.2) ** 2) -
      0.28 * Math.exp(-0.5 * ((p - period * 0.205) / 5.5) ** 2) +
      0.1 * Math.exp(-0.5 * ((p - period * 0.07) / 15) ** 2) +
      0.24 * Math.exp(-0.5 * ((p - period * 0.42) / 28) ** 2) +
      (Math.random() - 0.5) * 0.018;
    const envelope = 0.1 + 0.25 * Math.max(0, Math.sin(state.phase / 900 * Math.PI * 2 - 1));
    const makeEmg = (a, b) => envelope * (0.55 * Math.sin(state.phase * a) + 0.3 * Math.sin(state.phase * b) + (Math.random() - 0.5) * 0.22);
    addSample(
      [ecg, makeEmg(.41, .73), makeEmg(.36, .66), makeEmg(.48, .81), 0, 0, 0, 0, 0, 0, 0, 0],
      undefined,
      batchTimestamp + i * 1_000_000 / state.sampleRate,
    );
    state.phase += 1;
  }
}

function closeSocket() {
  if (state.socket) {
    const socket = state.socket;
    state.socket = null;
    socket.close();
  }
}

function startDemo() {
  closeSocket();
  resetSignalData();
  setSampleRate(500);
  state.mode = "demo";
  state.paused = false;
  setStatus("demo", "模拟运行", "四通道模拟信号 · 500 Hz");
  elements.connectButton.textContent = "连接设备";
  showToast("模拟数据已启动");
}

function connectSocket() {
  const url = elements.socketUrl.value.trim();
  if (!/^wss?:\/\//i.test(url)) {
    showToast("地址必须以 ws:// 或 wss:// 开头");
    return;
  }
  closeSocket();
  state.mode = "connecting";
  setStatus("connecting", "正在连接", `${url} · 等待 ESP32 响应`);
  elements.connectButton.textContent = "取消连接";
  try {
    const socket = new WebSocket(url);
    socket.binaryType = "arraybuffer";
    state.socket = socket;
    socket.addEventListener("open", () => {
      if (state.socket !== socket) return;
      socket.send(JSON.stringify({ type: "timeSync", timestampUs: Date.now() * 1000 }));
      resetSignalData();
      setSampleRate(500);
      state.mode = "live";
      state.paused = false;
      setStatus("connected", "设备在线", `${url} · 等待第一帧样本`);
      elements.connectButton.textContent = "断开设备";
      window.AndroidHost?.startLocation();
    });
    socket.addEventListener("message", (event) => {
      parsePayload(event.data);
      if (state.sampleCount === 1) setStatus("connected", "正在接收", `${url} · 四通道实时传输`);
    });
    socket.addEventListener("error", () => showToast("连接失败，请检查 ESP32 网络和 WebSocket 地址"));
    socket.addEventListener("close", () => {
      if (state.socket !== socket) return;
      state.socket = null;
      state.mode = "idle";
      setStatus("disconnected", "设备离线", "WebSocket 已断开");
      elements.connectButton.textContent = "重新连接";
    });
  } catch (error) {
    state.mode = "idle";
    setStatus("disconnected", "设备离线", "WebSocket 地址无效");
    showToast(error.message);
  }
}

async function api(path, options = {}) {
  if (window.AndroidHost) {
    const response = JSON.parse(window.AndroidHost.api(path, options.method || "GET", options.body || ""));
    if (response.error) throw new Error(response.error);
    return response;
  }
  const response = await fetch(path, {
    ...options,
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
  });
  if (!response.ok) {
    let message = `请求失败 (${response.status})`;
    try { message = (await response.json()).error || message; } catch {}
    throw new Error(message);
  }
  return response.json();
}

async function startTask() {
  const name = elements.taskName.value.trim();
  if (!name) {
    showToast("请填写任务名称");
    elements.taskName.focus();
    return;
  }
  try {
    const task = await api("/api/tasks", {
      method: "POST",
      body: JSON.stringify({ name, note: elements.taskNote.value.trim(), sampleRate: state.sampleRate }),
    });
    state.task = task;
    state.taskStartedAt = Date.now();
    state.pendingRows = [];
    elements.recordButton.classList.add("recording");
    elements.recordButton.querySelector("span").textContent = "停止采集";
    elements.taskStateText.textContent = `正在采集：${task.name}`;
    elements.taskDetail.textContent = "数据正在实时写入本地 CSV";
    elements.recordState.textContent = "记录中";
    elements.taskName.disabled = true;
    elements.taskNote.disabled = true;
    showToast("采集任务已开始");
  } catch (error) {
    showToast(`无法开始任务：${error.message}`);
  }
}

function flushSamples() {
  if (!state.task || !state.pendingRows.length) return state.flushPromise;
  const rows = state.pendingRows.splice(0, state.pendingRows.length);
  const taskId = state.task.id;
  state.flushPromise = state.flushPromise
    .then(() => api(`/api/tasks/${taskId}/samples`, {
      method: "POST",
      body: JSON.stringify({ samples: rows }),
    }))
    .catch((error) => {
      state.pendingRows.unshift(...rows);
      showToast(`CSV 写入失败：${error.message}`);
    });
  return state.flushPromise;
}

async function stopTask() {
  const task = state.task;
  if (!task) return;
  await flushSamples();
  await state.flushPromise;
  try {
    await api(`/api/tasks/${task.id}/stop`, { method: "POST", body: "{}" });
    state.task = null;
    state.taskStartedAt = null;
    elements.recordButton.classList.remove("recording");
    elements.recordButton.querySelector("span").textContent = "开始采集";
    elements.taskStateText.textContent = "采集任务已完成";
    elements.taskDetail.textContent = "CSV 已保存在 data 文件夹，可在采集记录中回放";
    elements.recordState.textContent = "已保存";
    elements.taskName.disabled = false;
    elements.taskNote.disabled = false;
    elements.taskName.value = "";
    elements.taskNote.value = "";
    showToast("任务已停止并保存");
  } catch (error) {
    showToast(`停止任务失败：${error.message}`);
  }
}

async function loadTasks() {
  try {
    const { tasks } = await api("/api/tasks");
    elements.tasksBody.innerHTML = "";
    elements.emptyRecords.style.display = tasks.length ? "none" : "block";
    tasks.forEach((task) => {
      const row = document.createElement("tr");
      const duration = formatDuration(task.durationSeconds || 0);
      row.innerHTML = `
        <td class="task-name-cell"><strong></strong><span></span></td>
        <td>${formatDate(task.startedAt)}</td><td>${duration}</td>
        <td>${Number(task.sampleCount || 0).toLocaleString()}</td>
        <td>${Number(task.alarmCount || 0).toLocaleString()}</td>
        <td><div class="table-actions"><button class="mini-button replay">回放</button><a class="mini-button export">CSV</a><button class="mini-button danger delete">删除</button></div></td>`;
      row.querySelector(".task-name-cell strong").textContent = task.name;
      row.querySelector(".task-name-cell span").textContent = task.note || "无备注";
      row.querySelector(".export").href = `/api/tasks/${task.id}/csv`;
      row.querySelector(".export").setAttribute("download", "");
      row.querySelector(".replay").addEventListener("click", () => openReplay(task));
      row.querySelector(".delete").addEventListener("click", () => deleteTask(task));
      elements.tasksBody.appendChild(row);
    });
  } catch (error) {
    showToast(`读取任务失败：${error.message}`);
  }
}

async function loadAiTasks() {
  const selected = elements.aiTask.value;
  try {
    const { tasks } = await api("/api/tasks");
    elements.aiTask.replaceChildren(new Option("不附加信号任务", ""));
    tasks.filter((task) => task.sampleCount > 0).forEach((task) => {
      elements.aiTask.add(new Option(
        `${task.name} · ${Number(task.sampleCount).toLocaleString()} 样本`,
        task.id,
      ));
    });
    if ([...elements.aiTask.options].some((option) => option.value === selected)) {
      elements.aiTask.value = selected;
    }
  } catch (error) {
    showToast(`读取 AI 任务失败：${error.message}`);
  }
}

function setAiBusy(busy, label = "") {
  elements.generateReport.disabled = busy;
  elements.sendChat.disabled = busy;
  elements.aiTask.disabled = busy;
  elements.aiProcessingOverlay?.classList.toggle("show", busy);
  elements.aiProcessingOverlay?.setAttribute("aria-hidden", String(!busy));
  if (busy) {
    elements.aiStatus.textContent = label || "AI 正在生成，请稍候…";
    elements.aiStatus.classList.add("working");
  } else {
    elements.aiStatus.classList.remove("working");
    checkAiStatus();
  }
}

function beginAiRequest(type, extra = {}) {
  const id = ++state.aiRequestId;
  state.pendingAiRequest = { id, type, ...extra };
  setAiBusy(true, type === "report" ? "AI 正在整理信号简析…" : "AI 正在生成回答…");
  return id;
}

function endAiRequest(id) {
  if (!state.pendingAiRequest || state.pendingAiRequest.id !== id) return null;
  const request = state.pendingAiRequest;
  state.pendingAiRequest = null;
  setAiBusy(false);
  elements.chatInput.focus();
  return request;
}

async function generateAiReport() {
  const taskId = elements.aiTask.value;
  if (!taskId) {
    showToast("请先选择一个有信号数据的采集任务");
    return;
  }
  const requestId = beginAiRequest("report");
  elements.aiReport.textContent = "AI 正在读取采集统计并生成简析…";
  if (window.AndroidHost?.requestAiReport) {
    window.AndroidHost.requestAiReport(JSON.stringify({ requestId, taskId }));
    return;
  }
  try {
    const { report } = await api("/api/ai/report", {
      method: "POST",
      body: JSON.stringify({ taskId }),
    });
    elements.aiReport.textContent = report;
  } catch (error) {
    elements.aiReport.textContent = `报告生成失败：${error.message}`;
  } finally {
    endAiRequest(requestId);
  }
}

function appendChat(role, content) {
  const message = document.createElement("div");
  message.className = `chat-message ${role}`;
  message.textContent = content;
  elements.chatLog.appendChild(message);
  elements.chatLog.scrollTop = elements.chatLog.scrollHeight;
  return message;
}

async function sendAiMessage(event) {
  event.preventDefault();
  const message = elements.chatInput.value.trim();
  if (!message) return;
  elements.chatInput.value = "";
  await askAi(message, Boolean(state.settings.aiVoiceEnabled));
}

async function askAi(message, speakAnswer = false) {
  const history = state.aiHistory.slice();
  appendChat("user", message);
  state.aiHistory.push({ role: "user", content: message });
  const pendingMessage = appendChat("assistant pending", "AI 正在生成回答，请稍候…");
  const requestId = beginAiRequest("chat", { pendingMessage, speakAnswer });
  const payload = { requestId, message, history, taskId: elements.aiTask.value, realtimeContext: buildRealtimeContext() };
  if (window.AndroidHost?.requestAiChat) {
    window.AndroidHost.requestAiChat(JSON.stringify(payload));
    return;
  }
  try {
    const { answer } = await api("/api/ai/chat", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    completeAiChat(requestId, answer);
  } catch (error) {
    failAiChat(requestId, error.message);
  } finally {
    endAiRequest(requestId);
  }
}

function completeAiChat(requestId, answer) {
  const request = state.pendingAiRequest;
  if (!request || request.id !== requestId || request.type !== "chat") return;
  request.pendingMessage.classList.remove("pending");
  request.pendingMessage.textContent = answer;
  state.aiHistory.push({ role: "assistant", content: answer });
  state.aiHistory = state.aiHistory.slice(-10);
  elements.chatLog.scrollTop = elements.chatLog.scrollHeight;
  if (request.speakAnswer && state.settings.aiVoiceEnabled) window.AndroidHost?.speak?.(answer);
}

function failAiChat(requestId, message) {
  const request = state.pendingAiRequest;
  if (!request || request.id !== requestId || request.type !== "chat") return;
  request.pendingMessage.classList.remove("pending");
  request.pendingMessage.textContent = `暂时无法回答：${message}`;
}

window.onAiChatResult = (requestId, answer) => {
  completeAiChat(Number(requestId), String(answer || ""));
  endAiRequest(Number(requestId));
};

window.onAiChatError = (requestId, message) => {
  failAiChat(Number(requestId), String(message || "AI 对话失败"));
  endAiRequest(Number(requestId));
};

window.onAiReportResult = (requestId, report) => {
  const request = state.pendingAiRequest;
  if (!request || request.id !== Number(requestId) || request.type !== "report") return;
  elements.aiReport.textContent = String(report || "报告内容为空");
  endAiRequest(Number(requestId));
};

window.onAiReportError = (requestId, message) => {
  const request = state.pendingAiRequest;
  if (!request || request.id !== Number(requestId) || request.type !== "report") return;
  elements.aiReport.textContent = `报告生成失败：${message}`;
  endAiRequest(Number(requestId));
};

window.onVoiceQuestion = (message) => {
  const text = String(message || "").trim();
  if (!text) return;
  showToast(`识别到：${text}`);
  askAi(text, true);
};

window.onVoiceError = (message) => {
  showToast(String(message || "语音识别失败"));
};

window.onVoiceCancelled = () => {
  state.voiceListening = false;
  elements.voiceOrb?.classList.remove("listening");
  if (elements.voiceOrb) elements.voiceOrb.querySelector("strong").textContent = "已取消";
  setTimeout(() => { if (elements.voiceOrb && !state.voiceListening) elements.voiceOrb.querySelector("strong").textContent = "长按问 AI"; }, 900);
};

window.onAiSpeechState = (status, detail = "") => {
  if (status === "speaking") showToast("AI 正在语音朗读，关闭开关可停止");
  if (status === "error") showToast("系统语音朗读不可用");
  if (status === "fallback") showToast(`豆包语音不可用，已回退系统朗读${detail ? `：${detail}` : ""}`);
};

function clearAiChat() {
  state.aiHistory = [];
  elements.chatLog.replaceChildren();
  appendChat("assistant", "对话已清空。你可以继续询问信号指标或采集质量。");
}

async function checkAiStatus() {
  try {
    const status = await api("/api/ai/status");
    elements.aiStatus.textContent = status.configured
      ? `API 已配置 · ${status.volcTtsConfigured ? "豆包语音已启用" : "系统朗读"}`
      : window.AndroidHost ? "点击此处配置 ARK_API_KEY" : "未配置 ARK_API_KEY";
    elements.aiStatus.classList.toggle("ready", status.configured);
  } catch {
    elements.aiStatus.textContent = "本地服务未启动";
  }
}

async function deleteTask(task) {
  if (!confirm(`确定删除任务“${task.name}”及其 CSV 文件吗？`)) return;
  try {
    await api(`/api/tasks/${task.id}`, { method: "DELETE" });
    if (state.history?.task.id === task.id) elements.replayPanel.classList.remove("show");
    loadTasks();
  } catch (error) {
    showToast(`删除失败：${error.message}`);
  }
}

async function openReplay(task) {
  try {
    const data = await api(`/api/tasks/${task.id}/data?max=12000`);
    state.history = data;
    $("#replayTitle").textContent = task.name;
    $("#replayMeta").textContent = `${formatDate(task.startedAt)} · ${formatDuration(task.durationSeconds || 0)} · ${Number(task.sampleCount).toLocaleString()} 个样本`;
    $("#downloadCsv").href = `/api/tasks/${task.id}/csv`;
    $("#downloadCsv").onclick = window.AndroidHost ? (event) => {
      event.preventDefault();
      window.AndroidHost.exportCsv(task.id);
    } : null;
    elements.replayRange.value = 0;
    elements.replayPanel.classList.add("show");
    drawReplay();
    elements.replayPanel.scrollIntoView({ behavior: "smooth", block: "start" });
  } catch (error) {
    showToast(`回放加载失败：${error.message}`);
  }
}

function fitCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const ratio = Math.min(window.devicePixelRatio || 1, 1.5);
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height, ratio };
}

function cssColor(variable) {
  if (variable === "--violet") return "#8b6ba8";
  return getComputedStyle(document.body).getPropertyValue(variable).trim();
}

function drawLine(canvas, values, color, scale, markers = []) {
  const { width, height, ratio } = fitCanvas(canvas);
  const context = canvas.getContext("2d");
  context.clearRect(0, 0, width, height);
  if (values.length < 2) return;
  context.beginPath();
  values.forEach((value, index) => {
    const x = index / (values.length - 1) * width;
    const y = height / 2 - Math.max(-scale, Math.min(scale, value)) / scale * height * .43;
    if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
  });
  context.strokeStyle = color;
  context.lineWidth = 1.25 * ratio;
  context.lineJoin = "round";
  context.stroke();
  if (markers.length) {
    context.fillStyle = "#d63e35";
    markers.forEach((index) => {
      if (index < 0 || index >= values.length) return;
      const x = index / (values.length - 1) * width;
      const y = height / 2 - Math.max(-scale, Math.min(scale, values[index])) / scale * height * .43;
      context.beginPath();
      context.arc(x, y, 2.5 * ratio, 0, Math.PI * 2);
      context.fill();
    });
  }
}

function drawLive() {
  if (!$("#monitorView").classList.contains("active")) return;
  CHANNELS.forEach((channel) => {
    const values = getRecent(channel, DISPLAY_POINTS);
    const firstSample = state.sampleCount - values.length + 1;
    const markers = state.alarmMarkers[channel]
      .filter((sample) => sample >= firstSample)
      .map((sample) => sample - firstSample);
    drawLine($(`#${channel}Canvas`), values, cssColor(CHANNEL_COLORS[channel]), channel === "ecg" ? 2 : 1, markers);
  });
}

function drawReplay() {
  if (!state.history?.samples.length) return;
  const all = state.history.samples;
  const windowSize = Math.min(1000, all.length);
  const maxStart = Math.max(0, all.length - windowSize);
  const start = Math.round(Number(elements.replayRange.value) / 100 * maxStart);
  const segment = all.slice(start, start + windowSize);
  const mapping = { ecg: 2, emg1: 3, emg2: 4, emg3: 5 };
  const flagBits = { ecg: 7, emg1: 8, emg2: 16, emg3: 32 };
  CHANNELS.forEach((channel) => {
    const values = segment.map((row) => row[mapping[channel]]);
    const markers = segment.map((row, index) => row[row.length - 1] & flagBits[channel] ? index : -1).filter((index) => index >= 0);
    const id = `#history${channel[0].toUpperCase()}${channel.slice(1)}`;
    drawLine($(id), values, cssColor(CHANNEL_COLORS[channel]), channel === "ecg" ? 2 : 1, markers);
  });
  const sampleIndex = segment[0]?.[0] || 0;
  $("#replayPosition").textContent = formatDuration(sampleIndex / (state.history.task.sampleRate || 500));
}

function formatDuration(seconds) {
  const value = Math.max(0, Math.floor(seconds || 0));
  const hours = String(Math.floor(value / 3600)).padStart(2, "0");
  const minutes = String(Math.floor(value % 3600 / 60)).padStart(2, "0");
  const secs = String(value % 60).padStart(2, "0");
  return `${hours}:${minutes}:${secs}`;
}

function formatDate(value) {
  if (!value) return "—";
  return new Date(value).toLocaleString("zh-CN", { hour12: false });
}

function updateClock() {
  const now = new Date();
  $("#dateText").textContent = new Intl.DateTimeFormat("zh-CN", { month: "long", day: "numeric", weekday: "short" }).format(now);
  $("#timeText").textContent = now.toLocaleTimeString("zh-CN", { hour12: false });
  elements.captureDuration.textContent = state.taskStartedAt ? formatDuration((Date.now() - state.taskStartedAt) / 1000) : "00:00:00";
}

let lastDrawAt = 0;
function render(now) {
  if (now - lastDrawAt >= DRAW_INTERVAL_MS) {
    drawLive();
    lastDrawAt = now;
  }
  requestAnimationFrame(render);
}

function openSettings() {
  Object.entries(state.settings).forEach(([key, value]) => {
    const input = $(`#${key}`);
    if (input) input.type === "checkbox" ? input.checked = value : input.value = value;
  });
  $("#settingsDialog").showModal();
}

function saveSettings(event) {
  event.preventDefault();
  const next = { ...state.settings };
  Object.keys(DEFAULT_SETTINGS).forEach((key) => {
    const input = $(`#${key}`);
    next[key] = input.type === "checkbox" ? input.checked
      : key === "emergencyPhone" ? input.value.trim() : Number(input.value);
  });
  if (next.hrLow >= next.hrHigh) {
    showToast("心率下限必须小于上限");
    return;
  }
  if (next.emergencySmsEnabled && !/^\+?[0-9 -]{6,20}$/.test(next.emergencyPhone)) {
    showToast("请输入有效的紧急联系人手机号");
    return;
  }
  state.settings = next;
  setAiVoiceEnabled(next.aiVoiceEnabled, false);
  $("#heartRange").textContent = `${next.hrLow}–${next.hrHigh} BPM`;
  $("#emgThresholdText").textContent = `${Math.max(next.emg1Limit, next.emg2Limit, next.emg3Limit).toFixed(2)} mV`;
  $("#settingsDialog").close();
  showToast("报警设置已保存");
}

elements.connectButton.addEventListener("click", () => {
  if (state.mode === "live" || state.mode === "connecting") {
    closeSocket();
    state.mode = "idle";
    setStatus("disconnected", "设备离线", "已主动断开");
    elements.connectButton.textContent = "重新连接";
  } else connectSocket();
});
elements.demoButton.addEventListener("click", startDemo);
elements.recordButton.addEventListener("click", () => state.task ? stopTask() : startTask());
window.addEventListener("pointerdown", unlockAlarmSound, { once: true });
window.addEventListener("keydown", unlockAlarmSound, { once: true });
elements.pauseButton?.addEventListener("click", () => {
  state.paused = !state.paused;
  elements.pauseButton.textContent = state.paused ? "继续" : "暂停";
  const resumedMeta = state.mode === "demo"
    ? "四通道模拟信号 · 500 Hz"
    : `${elements.socketUrl.value.trim()} · 四通道实时传输`;
  setStatus(
    state.paused ? "connecting" : state.mode === "demo" ? "demo" : "connected",
    state.paused ? "已暂停" : state.mode === "demo" ? "模拟运行" : "正在接收",
    state.paused ? "显示和记录均已暂停" : resumedMeta,
  );
});
$("#dismissAlarm").addEventListener("click", () => {
  state.dismissedAlarmAt = Date.now();
  elements.alarmBanner.classList.remove("show");
});
elements.alarmBanner.addEventListener("click", (event) => {
  if (event.target.id !== "dismissAlarm") playAlarmSound();
});
$("#themeButton").addEventListener("click", () => document.body.classList.toggle("dark"));
elements.voiceToggle?.addEventListener("click", () => setAiVoiceEnabled(!state.settings.aiVoiceEnabled));
$("#settingsButton").addEventListener("click", openSettings);
$("#saveSettings").addEventListener("click", saveSettings);
$("#refreshTasks").addEventListener("click", loadTasks);
elements.replayRange.addEventListener("input", drawReplay);
elements.generateReport.addEventListener("click", generateAiReport);
$("#chatForm").addEventListener("submit", sendAiMessage);
$("#clearChat").addEventListener("click", clearAiChat);

let voicePressTimer = null;
let voiceDrag = null;

function restoreVoiceOrbPosition() {
  const saved = JSON.parse(localStorage.getItem("bioscopeVoiceOrb") || "null");
  if (!saved || !elements.voiceOrb) return;
  elements.voiceOrb.style.left = `${saved.left}px`;
  elements.voiceOrb.style.top = `${saved.top}px`;
  elements.voiceOrb.style.right = "auto";
  elements.voiceOrb.style.bottom = "auto";
}

function moveVoiceOrb(clientX, clientY) {
  if (!voiceDrag || !elements.voiceOrb) return;
  const size = elements.voiceOrb.offsetWidth;
  const left = Math.max(8, Math.min(window.innerWidth - size - 8, voiceDrag.left + clientX - voiceDrag.x));
  const top = Math.max(8, Math.min(window.innerHeight - size - 28, voiceDrag.top + clientY - voiceDrag.y));
  elements.voiceOrb.style.left = `${left}px`;
  elements.voiceOrb.style.top = `${top}px`;
  elements.voiceOrb.style.right = "auto";
  elements.voiceOrb.style.bottom = "auto";
}

elements.voiceOrb?.addEventListener("pointerdown", (event) => {
  event.preventDefault();
  const rect = elements.voiceOrb.getBoundingClientRect();
  voiceDrag = { pointerId: event.pointerId, x: event.clientX, y: event.clientY, left: rect.left, top: rect.top, moved: false };
  elements.voiceOrb.setPointerCapture?.(event.pointerId);
  voicePressTimer = setTimeout(() => {
    if (voiceDrag?.moved || !window.AndroidHost?.startVoiceQuestion) {
      if (!voiceDrag?.moved) showToast("语音问答仅在 Android App 中可用");
      return;
    }
    state.voiceListening = true;
    elements.voiceOrb.classList.add("listening");
    elements.voiceOrb.querySelector("strong").textContent = "松开后提问";
    window.AndroidHost.startVoiceQuestion();
    elements.voiceOrb.querySelector("strong").textContent = "松开提交 · 滑动取消";
  }, 350);
});

elements.voiceOrb?.addEventListener("pointermove", (event) => {
  if (!voiceDrag || event.pointerId !== voiceDrag.pointerId) return;
  const distance = Math.hypot(event.clientX - voiceDrag.x, event.clientY - voiceDrag.y);
  if (state.voiceListening && distance > 28) {
    voiceDrag.cancelled = true;
    state.voiceListening = false;
    elements.voiceOrb.classList.remove("listening");
    elements.voiceOrb.querySelector("strong").textContent = "已取消";
    window.AndroidHost?.cancelVoiceQuestion?.();
    return;
  }
  if (distance > 12) {
    voiceDrag.moved = true;
    clearTimeout(voicePressTimer);
    if (state.voiceListening) window.AndroidHost?.cancelVoiceQuestion?.();
    state.voiceListening = false;
    elements.voiceOrb.classList.remove("listening");
    elements.voiceOrb.querySelector("strong").textContent = "拖动定位";
    moveVoiceOrb(event.clientX, event.clientY);
  }
});

const finishVoicePointer = (event) => {
  if (!voiceDrag || (event && event.pointerId !== voiceDrag.pointerId)) return;
  clearTimeout(voicePressTimer);
  const moved = voiceDrag.moved;
  const cancelled = voiceDrag.cancelled;
  voiceDrag = null;
  if (cancelled && elements.voiceOrb) {
    setTimeout(() => { if (!state.voiceListening) elements.voiceOrb.querySelector("strong").textContent = "长按问 AI"; }, 900);
    return;
  }
  if (moved && elements.voiceOrb) {
    const rect = elements.voiceOrb.getBoundingClientRect();
    localStorage.setItem("bioscopeVoiceOrb", JSON.stringify({ left: Math.round(rect.left), top: Math.round(rect.top) }));
    elements.voiceOrb.querySelector("strong").textContent = "长按问 AI";
    return;
  }
  if (!state.voiceListening) return;
  state.voiceListening = false;
  elements.voiceOrb.classList.remove("listening");
  elements.voiceOrb.querySelector("strong").textContent = "正在识别…";
  window.AndroidHost?.stopVoiceQuestion?.();
  setTimeout(() => { if (!state.voiceListening) elements.voiceOrb.querySelector("strong").textContent = "长按问 AI"; }, 2000);
};
["pointerup", "pointercancel"].forEach((eventName) => elements.voiceOrb?.addEventListener(eventName, finishVoicePointer));
restoreVoiceOrbPosition();
updateVoiceToggle();
elements.aiStatus.addEventListener("click", () => window.AndroidHost?.configureAi());

$$(".nav-item[data-view]").forEach((button) => {
  button.addEventListener("click", () => {
    $$(".nav-item[data-view]").forEach((item) => item.classList.toggle("active", item === button));
    const view = button.dataset.view;
    const titles = { monitor: "实时生理信号", records: "采集记录", ai: "AI 健康助手" };
    ["monitor", "records", "ai"].forEach((name) => {
      $(`#${name}View`).classList.toggle("active", view === name);
    });
    $("#pageTitle").textContent = titles[view];
    if (view === "records") loadTasks();
    if (view === "ai") loadAiTasks();
  });
});

setInterval(() => {
  if (state.mode === "live" && state.lastSampleAt && Date.now() - state.lastSampleAt > state.settings.disconnectMs) {
    updateAlarm(64);
  }
}, 250);
setInterval(createDemoBatch, 20);
setInterval(flushSamples, 2000);
setInterval(updateClock, 1000);
window.addEventListener("beforeunload", () => {
  closeSocket();
  if (state.task && state.pendingRows.length) navigator.sendBeacon(
    `/api/tasks/${state.task.id}/samples`,
    new Blob([JSON.stringify({ samples: state.pendingRows })], { type: "application/json" }),
  );
});

updateClock();
updateControls();
if (!window.AndroidHost) fetch("/api/health").catch(() => showToast("本地服务未启动，CSV 保存和历史功能不可用"));
checkAiStatus();
render();
