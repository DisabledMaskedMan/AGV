"use strict";

const RoiGeometry = {
  minimum: 0.05,
  handles(config, project) {
    const [left, top] = project(config.max_x, config.max_y);
    const [right, bottom] = project(config.min_x, config.min_y);
    return [["nw", left, top], ["n", (left + right) / 2, top], ["ne", right, top],
      ["w", left, (top + bottom) / 2], ["e", right, (top + bottom) / 2],
      ["sw", left, bottom], ["s", (left + right) / 2, bottom], ["se", right, bottom]];
  },
  hit(config, project, screenX, screenY) {
    for (const [name, handleX, handleY] of this.handles(config, project)) {
      if (Math.hypot(handleX - screenX, handleY - screenY) <= 9) return name;
    }
    const [left, top] = project(config.max_x, config.max_y);
    const [right, bottom] = project(config.min_x, config.min_y);
    return screenX >= left && screenX <= right && screenY >= top && screenY <= bottom ? "move" : "pan";
  },
  update(original, mode, deltaX, deltaY) {
    const result = { ...original };
    if (mode === "move") {
      result.min_x += deltaX; result.max_x += deltaX;
      result.min_y += deltaY; result.max_y += deltaY;
    } else {
      if (mode.includes("n")) result.max_x = Math.max(original.min_x + this.minimum, original.max_x + deltaX);
      if (mode.includes("s")) result.min_x = Math.min(original.max_x - this.minimum, original.min_x + deltaX);
      if (mode.includes("w")) result.max_y = Math.max(original.min_y + this.minimum, original.max_y + deltaY);
      if (mode.includes("e")) result.min_y = Math.min(original.max_y - this.minimum, original.min_y + deltaY);
    }
    return result;
  },
};
if (typeof module !== "undefined") module.exports = RoiGeometry;
