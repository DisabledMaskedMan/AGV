"use strict";
const assert = require("node:assert/strict");
const roi = require("../scan_web/static/roi.js");
const config = { min_x: -1, max_x: 4, min_y: -2, max_y: 2 };
const project = (pointX, pointY) => [300 - pointY * 40, 300 - pointX * 40];
assert.equal(roi.handles(config, project).length, 8);
for (const [mode, screenX, screenY] of roi.handles(config, project)) {
  assert.equal(roi.hit(config, project, screenX + 3, screenY - 3), mode);
  const next = roi.update(config, mode, -100, 100);
  assert.ok(next.max_x - next.min_x >= 0.05 - 1e-12);
  assert.ok(next.max_y - next.min_y >= 0.05 - 1e-12);
}
assert.equal(roi.hit(config, project, ...project(1, 0)), "move");
assert.equal(roi.hit(config, project, 1, 1), "pan");
const moved = roi.update(config, "move", -3, -4);
assert.equal(moved.min_x, -4); assert.equal(moved.max_x, 1);
assert.equal(moved.min_y, -6); assert.equal(moved.max_y, -2);
const zoomed = (pointX, pointY) => [300 - pointY * 5, 300 - pointX * 5];
const handle = roi.handles(config, zoomed)[0];
assert.equal(roi.hit(config, zoomed, handle[1] - 6, handle[2] - 4), "nw");
assert.deepEqual(config, { min_x: -1, max_x: 4, min_y: -2, max_y: 2 });
console.log("ROI geometry tests passed");
