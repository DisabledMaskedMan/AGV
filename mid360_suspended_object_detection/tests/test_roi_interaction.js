"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const vm = require("node:vm");

async function main() {
  const nodes = new Map();
  const controls = [];
  const strokes = [];
  const drawing = new Proxy({}, { get: (target, key) => key === 'strokeRect'
    ? (...bounds) => strokes.push({ color: target.strokeStyle, bounds })
    : () => {}, set: (target, key, value) => { target[key] = value; return true; } });
  class Node {
    constructor() { this.listeners = {}; this.value = ""; this.style = {}; this.captured = false; }
    append(child) { if (child.id) { nodes.set(child.id, child); controls.push(child); } }
    addEventListener(name, callback) { this.listeners[name] = callback; }
    setAttribute() {}
    replaceChildren() { this.rows = []; }
    insertRow() { const row = { cells: [], insertCell() { const cell = {}; this.cells.push(cell); return cell; } }; (this.rows ||= []).push(row); return row; }
    getBoundingClientRect() { return { left: 0, top: 0, width: 800, height: 600 }; }
    setPointerCapture() { this.captured = true; }
    hasPointerCapture() { return this.captured; }
    releasePointerCapture() { this.captured = false; }
    getContext() { return drawing; }
  }
  const getNode = (identifier) => { if (!nodes.has(identifier)) nodes.set(identifier, new Node()); return nodes.get(identifier); };
  const config = { min_x: .3, max_x: 5, min_y: -1, max_y: 1, cluster_distance: .12,
    min_cluster_points: 8, max_cluster_points: 100000, min_width: .08, max_width: 2,
    min_depth: .08, max_depth: 3, cloud_timeout: 1 };
  const state = { config, status: "clear", objects: [], points: [[1, 0]], valid_count: 1,
    scan_count: 6000, roi_count: 1, receive_hz: 65, content_hz: 10, receive_age_s: .01,
    processing_ms: 2, processed_count: 1, received_count: 1, content_version: 1,
    reconnects: 0, frame: "scan_link", laser_offset: null };
  let postHandler = async (changes) => ({ ok: true, json: async () => ({ ...config, ...changes }) });
  let posts = [];
  let postUrls = [];
  let offline = false;
  const sandbox = {
    console, AbortSignal,
    document: { getElementById: getNode, createElement: () => new Node(),
      querySelector: () => getNode("help"), addEventListener: (name, handler) => { getNode("document").listeners[name] = handler; } },
    window: { devicePixelRatio: 1, setTimeout() {} },
    ResizeObserver: class { constructor(callback) { this.callback = callback; } observe() { this.callback(); } },
    fetch: async (url, options = {}) => {
      if (offline) throw new Error('offline test');
      if (options.method === "POST") { const changes = JSON.parse(options.body); posts.push(changes); postUrls.push(url); return postHandler(changes); }
      return { ok: true, json: async () => structuredClone(state) };
    },
  };
  getNode("config-form").elements = controls;
  vm.createContext(sandbox);
  vm.runInContext(fs.readFileSync("scan_web/static/roi.js", "utf8"), sandbox);
  vm.runInContext(fs.readFileSync("scan_web/static/app.js", "utf8"), sandbox);
  await new Promise((resolve) => setImmediate(resolve));
  const evaluate = (code) => vm.runInContext(code, sandbox);
  const canvas = getNode("scan-canvas");
  const pointer = (point, extra = {}) => ({ button: 0, pointerId: 1, clientX: point[0], clientY: point[1], ...extra });
  const center = evaluate("worldToScreen(2.5, 0)");
  const oldView = evaluate("JSON.stringify(view)");
  canvas.listeners.pointerdown(pointer(center));
  canvas.listeners.pointermove(pointer([center[0] + 30, center[1] + 40]));
  const draft = evaluate("JSON.stringify(roiDraft)");
  sandbox.injected = structuredClone(state);
  sandbox.injected.config.min_x = -100;
  evaluate("showState(injected)");
  assert.equal(evaluate("JSON.stringify(roiDraft)"), draft);
  assert.equal(evaluate("appliedConfig.min_x"), config.min_x);
  await canvas.listeners.pointerup(pointer([center[0] + 30, center[1] + 40]));
  assert.equal(posts.length, 1);
  assert.equal(postUrls[0], "/api/config");
  assert.match(getNode("config-message").textContent, /尚未保存/);
  assert.deepEqual(Object.keys(posts[0]).sort(), ["max_x", "max_y", "min_x", "min_y"]);
  assert.equal(evaluate("JSON.stringify(view)"), oldView);
  assert.equal(evaluate("roiDraft"), null);
  assert.notEqual(evaluate("appliedConfig.min_x"), config.min_x);
  evaluate("appliedConfig = initialConfig");
  posts = [];
  canvas.listeners.pointerdown(pointer(center));
  canvas.listeners.pointermove(pointer([center[0] + 20, center[1]]));
  getNode("document").listeners.keydown({ key: "Escape" });
  assert.equal(evaluate("drag"), null);
  assert.equal(evaluate("roiDraft"), null);
  assert.equal(posts.length, 0);
  assert.equal(Number(getNode("min_x").value), config.min_x);
  postHandler = async () => ({ ok: false, json: async () => ({ error: "test rejection" }) });
  canvas.listeners.pointerdown(pointer(center));
  canvas.listeners.pointermove(pointer([center[0] + 10, center[1]]));
  await canvas.listeners.pointerup(pointer([center[0] + 10, center[1]]));
  assert.equal(evaluate("appliedConfig.min_x"), config.min_x);
  assert.match(getNode("config-message").textContent, /test rejection/);
  let complete;
  postHandler = () => new Promise((resolve) => { complete = resolve; });
  const first = evaluate("submitConfig({min_x:-1})");
  await evaluate("submitConfig({min_x:-2})");
  assert.equal(evaluate("submitting"), true);
  canvas.listeners.pointerdown(pointer(center));
  assert.equal(evaluate("drag"), null);
  complete({ ok: true, json: async () => ({ ...config, min_x: -1 }) });
  await first;
  assert.equal(evaluate("submitting"), false);
  assert.ok(controls.every((control) => !control.disabled));
  postHandler = async (changes) => ({ ok: true, json: async () => ({ ...config, ...changes }) });
  await getNode("config-form").listeners.submit({ preventDefault() {} });
  assert.equal(postUrls.at(-1), "/api/config/save");
  assert.deepEqual(Object.keys(posts.at(-1)).sort(), Object.keys(config).sort());
  assert.match(getNode("config-message").textContent, /已应用并保存/);
  const confirmed = evaluate("JSON.stringify(appliedConfig)");
  postHandler = async () => ({ ok: false, json: async () => ({ error: "disk write failure" }) });
  getNode("min_x").value = -3;
  await getNode("config-form").listeners.submit({ preventDefault() {} });
  assert.equal(evaluate("JSON.stringify(appliedConfig)"), confirmed);
  assert.equal(Number(getNode("min_x").value), JSON.parse(confirmed).min_x);
  assert.match(getNode("config-message").textContent, /disk write failure/);
  evaluate("appliedConfig = initialConfig");
  canvas.listeners.pointerdown(pointer([5, 500]));
  assert.equal(evaluate("drag.mode"), "pan");
  canvas.listeners.pointermove(pointer([25, 530]));
  canvas.listeners.pointercancel();
  assert.equal(evaluate("JSON.stringify(view)"), oldView);
  const handles = evaluate("RoiGeometry.handles(appliedConfig,worldToScreen)");
  for (const [mode, screenX, screenY] of handles) {
    canvas.listeners.pointerdown(pointer([screenX, screenY]));
    assert.equal(evaluate("drag.mode"), mode);
    canvas.listeners.pointermove(pointer([screenX + 10, screenY + 10]));
    canvas.listeners.lostpointercapture();
    assert.equal(evaluate("roiDraft"), null);
  }
  const box = (x) => ({ min_x: x, max_x: x + .5, min_y: -.5, max_y: .5,
    center_x: x + .25, center_y: 0, width: 1, depth: .5, count: 9, distance: x });
  sandbox.injected = { ...structuredClone(state), status: 'detected', objects: [box(1), box(3)], largest_obstacle: box(3) };
  strokes.length = 0;
  evaluate('showState(injected)');
  const boxesDrawn = strokes.filter(s => ['#ffd166', '#ff5252'].includes(s.color));
  assert.deepEqual(boxesDrawn.map(s => s.color), ['#ffd166', '#ff5252']);
  assert.equal(evaluate('selectedIndex(latest)'), 1); // backend choice, never client sorting
  assert.match(getNode('objects').rows[1].cells[0].textContent, /最终障碍物/);
  assert.equal(getNode('objects').rows[1].className, 'selected-object');
  sandbox.injected.objects = [box(3)];
  evaluate('showState(injected)');
  assert.equal(evaluate('selectedIndex(latest)'), 0);
  const message = { header: { stamp: { sec: 123, nanosec: 456 }, frame_id: 'scan_link' },
    data_valid: true, detected: false, status: 'clear', error: '', candidate_count: 0, point_count: 0,
    center: { x: 0, y: 0, z: 0 }, nearest_point: { x: 0, y: 0, z: 0 }, distance_x: 0, distance_y: 0,
    center_distance: 0, distance: 0, width: 0, depth: 0, area: 0, receive_age_s: .01, result_age_s: .02 };
  sandbox.publication = { supported: true, enabled: true, topic: '/custom/remapped', message_type: 'mid360_suspended_object_detection/msg/LargestObstacle',
    status: 'publishing', frequency_hz: 10, published_count: 102, last_publish_age_s: .04, error: '', message };
  evaluate('showPublication(publication)');
  assert.equal(getNode('ros-hz').textContent, '10.0 Hz');
  assert.equal(getNode('ros-topic').textContent, '/custom/remapped');
  assert.match(getNode('ros-count-age').textContent, /102 条 \/ 40 ms/);
  assert.equal(getNode('ros-center').textContent, '0.000 / 0.000 m');
  assert.equal(getNode('ros-area').textContent, '0.000 m²');
  assert.equal(getNode('ros-candidates').textContent, '0 / 0');
  assert.deepEqual(JSON.parse(getNode('ros-message').textContent), message);
  assert.equal(evaluate('selectedIndex(latest)'), 0); // old clear publication cannot erase current red box
  sandbox.publication.status = 'error'; sandbox.publication.error = 'publish failure';
  evaluate('showPublication(publication)');
  assert.match(getNode('ros-error').textContent, /publish failure/);
  assert.deepEqual(JSON.parse(getNode('ros-message').textContent), message);
  sandbox.publication.status = 'stalled'; sandbox.publication.frequency_hz = 0;
  evaluate('showPublication(publication)');
  assert.equal(getNode('ros-hz').textContent, '0.0 Hz');
  assert.equal(getNode('ros-status').textContent, '发布停滞');
  sandbox.publication.status = 'waiting'; sandbox.publication.frequency_hz = null; sandbox.publication.message = null;
  evaluate('showPublication(publication)');
  assert.equal(getNode('ros-hz').textContent, '—');
  assert.equal(getNode('ros-center').textContent, '—');
  sandbox.publication.status = 'disabled'; sandbox.publication.enabled = false;
  evaluate('showPublication(publication)');
  assert.equal(getNode('ros-status').textContent, 'ROS 发布已关闭');
  sandbox.publication.status = 'unsupported'; sandbox.publication.supported = false;
  evaluate('showPublication(publication)');
  assert.equal(getNode('ros-status').textContent, '当前后端不支持 ROS 发布');
  offline = true;
  await evaluate('poll()');
  assert.equal(evaluate('latest'), null);
  assert.equal(getNode('ros-hz').textContent, '—');
  assert.equal(getNode('ros-message').textContent, '—');
  assert.equal(getNode('objects').rows[0].cells[0].className, 'empty');
  assert.equal(getNode('ros-status').textContent, 'Web 服务不可达');
  offline = false;
  await evaluate('poll()');
  assert.equal(evaluate('latest.status'), 'clear');
  console.log("ROI interaction tests passed: drag, 8 handles, polling, cancel, rejection, submission lock, pan");
  console.log('Publication UI tests passed: red last, backend selection, table, zero values, JSON, errors, disabled, unsupported, offline/recovery');
}
main().catch((error) => { console.error(error); process.exitCode = 1; });
