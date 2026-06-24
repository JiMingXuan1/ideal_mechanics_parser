let _counters = { n: 0, e: 0 };

export function uid(prefix = 'n') {
  if (!_counters[prefix]) _counters[prefix] = 0;
  _counters[prefix]++;
  return `${prefix}${_counters[prefix]}`;
}

export function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

export function dist(x1, y1, x2, y2) {
  return Math.hypot(x2 - x1, y2 - y1);
}

export function pointToSegmentDist(px, py, x1, y1, x2, y2) {
  const dx = x2 - x1, dy = y2 - y1;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return dist(px, py, x1, y1);
  let t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
  t = Math.max(0, Math.min(1, t));
  return dist(px, py, x1 + t * dx, y1 + t * dy);
}
