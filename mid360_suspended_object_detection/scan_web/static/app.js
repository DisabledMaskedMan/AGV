"use strict";

const element = (identifier) => document.getElementById(identifier);
const canvas = element("scan-canvas");
const context = canvas.getContext("2d");
const fieldGroups = {
  "roi-fields": [["min_x", "X 最小"], ["max_x", "X 最大"], ["min_y", "Y 最小"], ["max_y", "Y 最大"]],
  "cluster-fields": [["cluster_distance", "邻点距离 / m"], ["cloud_timeout", "接收超时 / s"], ["min_cluster_points", "最少点数"], ["max_cluster_points", "最多点数"]],
  "box-fields": [["min_width", "宽度最小 / Y"], ["max_width", "宽度最大 / Y"], ["min_depth", "深度最小 / X"], ["max_depth", "深度最大 / X"]],
};
const fields = [];
let latest = null;
let initialConfig = null;
let appliedConfig = null;
let view = { centerX: 2.5, centerY: 0, scale: 45 };
let canvasWidth = 1;
let canvasHeight = 1;
let drag = null;
let requestFailed = false;
let submitting = false;
let roiDraft = null;
let stateEpoch = 0;
document.querySelector(".plot-help").textContent = "框内移动 · 边角缩放 · 框外平移 · 松手应用";

for (const [group, specifications] of Object.entries(fieldGroups)) {
  for (const [name, labelText] of specifications) {
    const label = document.createElement("label");
    label.textContent = labelText;
    const input = document.createElement("input");
    input.name = name;
    input.id = name;
    input.type = "number";
    input.step = name.includes("points") ? "1" : "any";
    input.required = true;
    label.append(input);
    element(group).append(label);
    fields.push(name);
  }
}

const statusLabels = {
  waiting: "已连接 · 等待目标扫描", connecting: "正在连接数据源", streaming: "正在接收扫描",
  processing: "扫描已接收 · 正在分析", clear: "数据正常 · 未检出候选", detected: "数据正常 · 检出候选",
  invalid: "雷达数据异常", timeout: "目标扫描接收超时", disconnected: "数据源断开 · 自动重连中",
};

function worldToScreen(pointX, pointY) {
  return [canvasWidth / 2 - (pointY - view.centerY) * view.scale,
    canvasHeight / 2 - (pointX - view.centerX) * view.scale];
}

function screenToWorld(screenX, screenY) {
  return [view.centerX - (screenY - canvasHeight / 2) / view.scale,
    view.centerY - (screenX - canvasWidth / 2) / view.scale];
}

function fitBounds(minX, maxX, minY, maxY) {
  view.centerX = (minX + maxX) / 2;
  view.centerY = (minY + maxY) / 2;
  view.scale = Math.max(2, Math.min(1000, Math.min(canvasHeight / Math.max(1, maxX - minX),
    canvasWidth / Math.max(1, maxY - minY)) * 0.72));
  draw();
}

function fitRoi() {
  if (!appliedConfig || drag || submitting) return;
  fitBounds(Math.min(0, appliedConfig.min_x), appliedConfig.max_x, appliedConfig.min_y, appliedConfig.max_y);
}

function rectangle(minX, maxX, minY, maxY) {
  const [left, top] = worldToScreen(maxX, maxY);
  return [left, top, (maxY - minY) * view.scale, (maxX - minX) * view.scale];
}

function draw() {
  const pixelRatio = window.devicePixelRatio || 1;
  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  context.clearRect(0, 0, canvasWidth, canvasHeight);
  const [maxX, maxY] = screenToWorld(0, 0);
  const [minX, minY] = screenToWorld(canvasWidth, canvasHeight);
  const rawStep = 65 / view.scale;
  const magnitude = 10 ** Math.floor(Math.log10(rawStep));
  const step = [1, 2, 5, 10].map((value) => value * magnitude).find((value) => value >= rawStep);
  context.font = '10px Consolas, "Microsoft YaHei", monospace';
  context.lineWidth = 1;
  for (let gridX = Math.ceil(minX / step) * step; gridX <= maxX; gridX += step) {
    const [, screenY] = worldToScreen(gridX, 0);
    context.strokeStyle = Math.abs(gridX) < step * 0.01 ? "#365264" : "#1b2a38";
    context.beginPath(); context.moveTo(0, screenY); context.lineTo(canvasWidth, screenY); context.stroke();
    context.fillStyle = "#617e96"; context.fillText(gridX.toFixed(step < 1 ? 1 : 0), 8, screenY - 4);
  }
  for (let gridY = Math.ceil(minY / step) * step; gridY <= maxY; gridY += step) {
    const [screenX] = worldToScreen(0, gridY);
    context.strokeStyle = Math.abs(gridY) < step * 0.01 ? "#365264" : "#1b2a38";
    context.beginPath(); context.moveTo(screenX, 0); context.lineTo(screenX, canvasHeight); context.stroke();
    context.fillStyle = "#617e96"; context.fillText(gridY.toFixed(step < 1 ? 1 : 0), screenX + 4, 14);
  }
  element("scale-label").textContent = `网格 ${Number(step.toPrecision(3))} m`;
  const visibleConfig = roiDraft || appliedConfig;
  if (visibleConfig) {
    const bounds = rectangle(visibleConfig.min_x, visibleConfig.max_x, visibleConfig.min_y, visibleConfig.max_y);
    context.fillStyle = "#65d9cc0c"; context.fillRect(...bounds);
    context.strokeStyle = "#65d9cc"; context.setLineDash([5, 4]); context.strokeRect(...bounds); context.setLineDash([]);
    context.fillStyle = "#65d9cc"; context.fillText("ROI", bounds[0] + 8, bounds[1] + 17);
    for (const [, handleX, handleY] of RoiGeometry.handles(visibleConfig, worldToScreen)) {
      context.fillStyle = "#0b141e"; context.fillRect(handleX - 4, handleY - 4, 8, 8);
      context.strokeStyle = "#65d9cc"; context.strokeRect(handleX - 4, handleY - 4, 8, 8);
    }
  }
  for (const [pointX, pointY] of latest?.points || []) {
    const [screenX, screenY] = worldToScreen(pointX, pointY);
    if (screenX < 0 || screenX > canvasWidth || screenY < 0 || screenY > canvasHeight) continue;
    const inside = visibleConfig && pointX >= visibleConfig.min_x && pointX <= visibleConfig.max_x
      && pointY >= visibleConfig.min_y && pointY <= visibleConfig.max_y;
    context.fillStyle = inside ? "#65d9cc" : "#86a9cb";
    context.fillRect(screenX - 1.1, screenY - 1.1, 2.2, 2.2);
  }
  context.lineWidth = 2;
  const finalIndex = selectedIndex(latest);
  (latest?.objects || []).forEach((box, index) => {
    if (index === finalIndex) return;
    const bounds = rectangle(box.min_x, box.max_x, box.min_y, box.max_y);
    context.strokeStyle = "#ffd166"; context.strokeRect(...bounds);
    context.fillStyle = "#ffd166"; context.fillText(`#${index + 1} · ${box.distance.toFixed(2)} m`, bounds[0], bounds[1] - 7);
  });
  if (finalIndex >= 0) {
    const box = latest.largest_obstacle;
    const bounds = rectangle(box.min_x, box.max_x, box.min_y, box.max_y);
    context.lineWidth = 3;
    context.strokeStyle = "#ff5252"; context.strokeRect(...bounds);
    context.fillStyle = "#ff8a8a";
    context.fillText(`#${finalIndex + 1} 最终障碍物 · ${box.distance.toFixed(2)} m`, bounds[0], bounds[1] - 7);
  }
  const [originX, originY] = worldToScreen(0, 0);
  context.fillStyle = "#e3eef8"; context.beginPath();
  context.moveTo(originX, originY - 8); context.lineTo(originX - 5, originY + 5);
  context.lineTo(originX + 5, originY + 5); context.closePath(); context.fill();
  context.fillText("雷达", originX + 10, originY + 4);
}

function selectedIndex(state) {
  if (state?.status !== "detected" || !state.largest_obstacle) return -1;
  return state.objects.findIndex((box) => ["min_x", "max_x", "min_y", "max_y", "count", "distance"]
    .every((key) => box[key] === state.largest_obstacle[key]));
}

function showObjects(state) {
  const body = element("objects");
  body.replaceChildren();
  if (!state.objects.length) {
    const row = body.insertRow();
    const cell = row.insertCell();
    cell.colSpan = 6; cell.className = "empty";
    cell.textContent = state.status === "clear" ? "当前 ROI 未检出障碍物候选"
      : state.status === "processing" ? "正在分析最新扫描…" : "暂无有效检测结果";
    return;
  }
  state.objects.forEach((box, index) => {
    const row = body.insertRow();
    const selected = index === selectedIndex(state);
    if (selected) row.className = "selected-object";
    const values = [`#${index + 1}${selected ? " 最终障碍物" : ""}`, `${box.distance.toFixed(2)} m`, `${box.center_x.toFixed(2)} / ${box.center_y.toFixed(2)} m`,
      `${box.width.toFixed(2)} m`, `${box.depth.toFixed(2)} m`, box.count];
    for (const value of values) row.insertCell().textContent = value;
  });
}

function showPublication(publication, disconnected = false) {
  const labels = { unsupported: "当前后端不支持 ROS 发布", disabled: "ROS 发布已关闭", waiting: "等待首次发布",
    publishing: "正在发布", stalled: "发布停滞", error: "发布异常" };
  const available = !disconnected && publication?.supported;
  element("ros-status").textContent = disconnected ? "Web 服务不可达" : labels[publication?.status] || "等待发布状态";
  element("ros-topic").textContent = available ? publication.topic || "—" : "—";
  element("ros-type").textContent = available ? publication.message_type : "—";
  element("ros-hz").textContent = available && publication.enabled && Number.isFinite(publication.frequency_hz)
    ? `${publication.frequency_hz.toFixed(1)} Hz` : "—";
  const age = publication?.last_publish_age_s;
  element("ros-count-age").textContent = available ? `${publication.published_count} 条 / ${age === null ? "尚未发布" : `${(age * 1000).toFixed(0)} ms 前`}` : "—";
  const message = available && publication.enabled ? publication.message : null;
  element("ros-error").textContent = available ? [publication.error, message?.error].filter(Boolean).join("；") : "";
  for (const id of ["ros-valid", "ros-message-status", "ros-candidates", "ros-center", "ros-center-distance", "ros-nearest", "ros-size", "ros-area"]) element(id).textContent = "—";
  element("ros-message").textContent = message ? JSON.stringify(message, null, 2) : "—";
  if (!message) return;
  element("ros-valid").textContent = `${message.data_valid ? "有效" : "无效"} / ${message.detected ? "检出" : "未检出"}`;
  element("ros-message-status").textContent = statusLabels[message.status] || message.status;
  element("ros-candidates").textContent = `${message.candidate_count} / ${message.point_count}`;
  element("ros-center").textContent = `${message.distance_x.toFixed(3)} / ${message.distance_y.toFixed(3)} m`;
  element("ros-center-distance").textContent = `${message.center_distance.toFixed(3)} m`;
  element("ros-nearest").textContent = `${message.distance.toFixed(3)} m`;
  element("ros-size").textContent = `${message.width.toFixed(3)} / ${message.depth.toFixed(3)} m`;
  element("ros-area").textContent = `${message.area.toFixed(3)} m²`;
}

function populate(config) {
  for (const name of fields) element(name).value = config[name];
}

function showState(state) {
  latest = state;
  if (!drag && !submitting) appliedConfig = state.config;
  if (!initialConfig) {
    initialConfig = { ...state.config };
    populate(state.config);
    fitRoi();
  }
  const usable = state.status === "clear" || state.status === "detected";
  element("status").className = `status ${state.status}`;
  element("status").textContent = statusLabels[state.status] || state.status;
  element("error").textContent = state.error || "";
  element("source").textContent = state.source_url;
  element("frame").textContent = state.frame || `等待 ${state.target_frame}`;
  element("offset").textContent = state.laser_offset ? state.laser_offset.map((value) => value.toFixed(3)).join(", ") : "—";
  element("point-count").textContent = usable ? `${state.valid_count} / ${state.scan_count}` : "—";
  element("roi-count").textContent = usable ? state.roi_count : "—";
  element("roi-hint").textContent = !usable ? "等待有效分析" : state.roi_count ? "正在分析区域内扫描点" : "区域内无有效点，可调整 ROI";
  element("object-count").textContent = usable ? state.objects.length : "—";
  element("rates").textContent = `${state.receive_hz.toFixed(1)} / ${state.content_hz.toFixed(1)} Hz`;
  element("receive-age").textContent = state.receive_age_s === null ? "尚未收到扫描" : `距接收 ${(state.receive_age_s * 1000).toFixed(0)} ms`;
  element("processing").textContent = `分析 ${state.processing_ms.toFixed(1)} ms`;
  element("footer-stats").textContent = `接收 ${state.received_count} 帧 · 内容 ${state.content_version} 版 · 分析 ${state.processed_count} 次 · 重连 ${state.reconnects}`;
  element("plot-overlay").textContent = usable ? "" : statusLabels[state.status] || "等待扫描";
  canvas.setAttribute("aria-label", `二维扫描：${state.valid_count} 个有效点，ROI 内 ${state.roi_count} 点，${state.objects.length} 个障碍物候选`);
  showObjects(state);
  showPublication(state.ros_publication);
  draw();
}

async function poll() {
  const epoch = stateEpoch;
  try {
    const response = await fetch("/api/state", { cache: "no-store", signal: AbortSignal.timeout(3000) });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    if (epoch !== stateEpoch) return;
    showState(state);
    requestFailed = false;
  } catch (error) {
    if (epoch !== stateEpoch) return;
    requestFailed = true;
    cancelDrag();
    latest = null;
    element("status").className = "status disconnected";
    element("status").textContent = "本机 Web 服务不可达";
    element("error").textContent = error.message;
    element("plot-overlay").textContent = "Web 服务连接中断";
    for (const name of ["point-count", "roi-count", "object-count", "rates", "receive-age", "processing"]) element(name).textContent = "—";
    element("roi-hint").textContent = "等待服务恢复";
    showObjects({ objects: [], status: "disconnected" });
    showPublication(null, true);
    draw();
  } finally {
    window.setTimeout(poll, 200);
  }
}

async function submitConfig(changes, recenter = false, persist = false) {
  if (requestFailed || submitting) return false;
  submitting = true;
  stateEpoch += 1;
  for (const control of element("config-form").elements) control.disabled = true;
  const previous = { ...appliedConfig };
  try {
    const response = await fetch(persist ? "/api/config/save" : "/api/config", { method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(changes), signal: AbortSignal.timeout(3000) });
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    appliedConfig = result;
    populate(result);
    element("config-message").textContent = persist ? "参数已应用并保存到 YAML，重启后保留。" : "已应用，尚未保存；点击“应用并保存”写入 YAML。";
    if (latest) {
      latest = { ...latest, points: latest.points, objects: [], largest_obstacle: null, status: "processing" };
      showObjects(latest);
    }
    if (recenter) fitRoi();
    return true;
  } catch (error) {
    appliedConfig = previous;
    populate(previous);
    element("config-message").textContent = `应用失败：${error.message}`;
    return false;
  } finally {
    roiDraft = null;
    submitting = false;
    stateEpoch += 1;
    for (const control of element("config-form").elements) control.disabled = false;
    draw();
  }
}
element("config-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  if (drag || submitting) return;
  const changes = Object.fromEntries(fields.map((name) => [name, Number(element(name).value)]));
  await submitConfig(changes, false, true);
});
element("restore").addEventListener("click", () => {
  if (drag || submitting) return;
  if (initialConfig) populate(initialConfig);
  element("config-message").textContent = "已填入启动值，点击“应用并保存”生效并写入 YAML。";
});
element("fit-roi").addEventListener("click", fitRoi);
element("fit-all").addEventListener("click", () => {
  if (drag || submitting) return;
  const points = latest?.points;
  if (!points?.length) return;
  const pointXs = [0, ...points.map((point) => point[0])];
  const pointYs = [0, ...points.map((point) => point[1])];
  fitBounds(Math.min(...pointXs), Math.max(...pointXs), Math.min(...pointYs), Math.max(...pointYs));
});
function zoom(factor, screenX = canvasWidth / 2, screenY = canvasHeight / 2) {
  if (drag) return;
  const [beforeX, beforeY] = screenToWorld(screenX, screenY);
  view.scale = Math.max(2, Math.min(1000, view.scale * factor));
  const [afterX, afterY] = screenToWorld(screenX, screenY);
  view.centerX += beforeX - afterX;
  view.centerY += beforeY - afterY;
  draw();
}
element("zoom-in").addEventListener("click", () => zoom(1.4));
element("zoom-out").addEventListener("click", () => zoom(1 / 1.4));
canvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const bounds = canvas.getBoundingClientRect();
  zoom(event.deltaY < 0 ? 1.15 : 1 / 1.15, event.clientX - bounds.left, event.clientY - bounds.top);
}, { passive: false });
canvas.addEventListener("pointerdown", (event) => {
  if (event.button !== 0 || submitting || requestFailed || !appliedConfig || drag) return;
  const bounds = canvas.getBoundingClientRect();
  const mode = RoiGeometry.hit(appliedConfig, worldToScreen, event.clientX - bounds.left, event.clientY - bounds.top);
  drag = { mode, pointerId: event.pointerId, screenX: event.clientX, screenY: event.clientY,
    centerX: view.centerX, centerY: view.centerY, original: { ...appliedConfig } };
  if (mode !== "pan") roiDraft = { ...appliedConfig };
  canvas.setPointerCapture(event.pointerId);
});
canvas.addEventListener("pointermove", (event) => {
  if (drag && event.pointerId === drag.pointerId) {
    if (drag.mode === "pan") {
      view.centerX = drag.centerX + (event.clientY - drag.screenY) / view.scale;
      view.centerY = drag.centerY + (event.clientX - drag.screenX) / view.scale;
    } else {
      roiDraft = RoiGeometry.update(drag.original, drag.mode,
        -(event.clientY - drag.screenY) / view.scale, -(event.clientX - drag.screenX) / view.scale);
      for (const name of ["min_x", "max_x", "min_y", "max_y"]) element(name).value = Number(roiDraft[name].toFixed(4));
    }
    draw();
  }
  const bounds = canvas.getBoundingClientRect();
  if (!drag && appliedConfig) {
    const mode = submitting ? "wait" : RoiGeometry.hit(appliedConfig, worldToScreen, event.clientX - bounds.left, event.clientY - bounds.top);
    canvas.style.cursor = mode === "pan" ? "grab" : mode === "move" || mode === "wait" ? mode
      : mode === "n" || mode === "s" ? "ns-resize" : mode === "e" || mode === "w" ? "ew-resize"
        : mode === "nw" || mode === "se" ? "nwse-resize" : "nesw-resize";
  }
  const [pointX, pointY] = screenToWorld(event.clientX - bounds.left, event.clientY - bounds.top);
  element("cursor").textContent = `X ${pointX.toFixed(2)} / Y ${pointY.toFixed(2)} m`;
});
function cancelDrag() {
  if (!drag) return;
  const previous = drag;
  drag = null; roiDraft = null;
  if (previous.mode === "pan") { view.centerX = previous.centerX; view.centerY = previous.centerY; }
  else for (const name of ["min_x", "max_x", "min_y", "max_y"]) element(name).value = previous.original[name];
  if (canvas.hasPointerCapture(previous.pointerId)) canvas.releasePointerCapture(previous.pointerId);
  draw();
}
canvas.addEventListener("pointerup", async (event) => {
  if (!drag || event.pointerId !== drag.pointerId) return;
  const completed = drag;
  const draft = roiDraft;
  drag = null;
  if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
  if (completed.mode !== "pan" && draft) {
    const changes = Object.fromEntries(["min_x", "max_x", "min_y", "max_y"].map((name) => [name, draft[name]]));
    if (Object.keys(changes).some((name) => changes[name] !== completed.original[name])) await submitConfig(changes);
    else { roiDraft = null; draw(); }
  }
});
canvas.addEventListener("pointercancel", cancelDrag);
canvas.addEventListener("lostpointercapture", cancelDrag);
document.addEventListener("keydown", (event) => { if (event.key === "Escape") cancelDrag(); });
new ResizeObserver(() => {
  const bounds = canvas.getBoundingClientRect();
  canvasWidth = bounds.width; canvasHeight = bounds.height;
  canvas.width = Math.round(bounds.width * (window.devicePixelRatio || 1));
  canvas.height = Math.round(bounds.height * (window.devicePixelRatio || 1));
  draw();
}).observe(canvas);
poll();
