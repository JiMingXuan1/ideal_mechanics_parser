export class Camera {
  constructor() {
    this.zoom = 40;
    this.offsetX = 0;
    this.offsetY = 0;
    this._targetZoom = 40;
    this._targetOffsetX = 0;
    this._targetOffsetY = 0;
    this._centered = false;
  }

  centerOn(cx, cy) {
    this._targetOffsetX = cx;
    this._targetOffsetY = cy;
    this.offsetX = cx;
    this.offsetY = cy;
    this._centered = true;
  }

  screenToWorld(sx, sy) {
    return {
      x: (sx - this.offsetX) / this.zoom,
      y: (this.offsetY - sy) / this.zoom,
    };
  }

  worldToScreen(wx, wy) {
    return {
      x: wx * this.zoom + this.offsetX,
      y: -wy * this.zoom + this.offsetY,
    };
  }

  pan(dx, dy) {
    this._targetOffsetX += dx;
    this._targetOffsetY += dy;
  }

  zoomAt(worldX, worldY, factor) {
    const oldZoom = this.zoom;
    this._targetZoom = clamp(this._targetZoom * factor, 5, 500);
    const ratio = this._targetZoom / oldZoom;
    const screen = this.worldToScreen(worldX, worldY);
    this._targetOffsetX = screen.x - worldX * this._targetZoom;
    this._targetOffsetY = screen.y + worldY * this._targetZoom;
  }

  update(lerp = 0.15) {
    this.zoom += (this._targetZoom - this.zoom) * lerp;
    this.offsetX += (this._targetOffsetX - this.offsetX) * lerp;
    this.offsetY += (this._targetOffsetY - this.offsetY) * lerp;
  }
}

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}
