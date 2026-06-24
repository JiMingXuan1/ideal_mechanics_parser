export class Entities {
  static RADIUS_NODE = 8;
  static RADIUS_ANCHOR = 6;

  static _color(type) {
    const c = {
      anchor:    { fill: '#e1e4e8', stroke: '#959da5', hover: '#54aeff', select: '#f0883e' },
      point:     { fill: '#0969da', stroke: '#0969da', hover: '#54aeff', select: '#f0883e' },
      rod:       { base: '#57606a', hover: '#54aeff', select: '#f0883e' },
      spring:    { base: '#2da44e', hover: '#54aeff', select: '#f0883e' },
      rail:      { base: '#d29922', hover: '#54aeff', select: '#f0883e' },
      fixed:     { base: '#cf222e', hover: '#54aeff', select: '#f0883e' },
      linear:    { base: '#8250df', hover: '#54aeff', select: '#f0883e' },
      sum:       { base: '#1a7f37', hover: '#54aeff', select: '#f0883e' },
      angle:     { base: '#0550ae', hover: '#54aeff', select: '#f0883e' },
    };
    return c[type] || c.rod;
  }

  static drawAnchor(ctx, x, y, hovered, selected) {
    const c = this._color('anchor');
    ctx.beginPath();
    for (let i = 0; i < 4; i++) {
      const a = (i * Math.PI) / 2 + Math.PI / 4;
      const px = x + this.RADIUS_ANCHOR * Math.cos(a);
      const py = y + this.RADIUS_ANCHOR * Math.sin(a);
      i === 0 ? ctx.moveTo(px, py) : ctx.lineTo(px, py);
    }
    ctx.closePath();
    ctx.fillStyle = hovered ? c.hover : c.fill;
    ctx.fill();
    ctx.strokeStyle = selected ? c.select : c.stroke;
    ctx.lineWidth = selected ? 2 : 1;
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x - 4, y); ctx.lineTo(x + 4, y);
    ctx.moveTo(x, y - 4); ctx.lineTo(x, y + 4);
    ctx.strokeStyle = c.stroke;
    ctx.lineWidth = 1;
    ctx.stroke();
  }

  static drawMassPoint(ctx, x, y, hovered, selected, label) {
    const c = this._color('point');
    ctx.beginPath();
    ctx.arc(x, y, this.RADIUS_NODE, 0, Math.PI * 2);
    ctx.fillStyle = hovered ? c.hover : selected ? c.select : c.fill;
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.stroke();
    if (label) {
      ctx.fillStyle = '#656d76';
      ctx.font = '10px -apple-system, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(label, x, y - this.RADIUS_NODE - 4);
    }
  }

  static drawIdealRod(ctx, x1, y1, x2, y2, hovered, selected) {
    const c = this._color('rod');
    const color = selected ? c.select : hovered ? c.hover : c.base;
    ctx.beginPath();
    ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
    ctx.strokeStyle = color;
    ctx.lineWidth = selected ? 3 : 2;
    ctx.stroke();
    const mx = (x1 + x2)/2, my = (y1 + y2)/2;
    ctx.fillStyle = c.base;
    ctx.font = '10px -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('rod', mx, my - 4);
  }

  static drawIdealSpring(ctx, x1, y1, x2, y2, hovered, selected) {
    const c = this._color('spring');
    const color = selected ? c.select : hovered ? c.hover : c.base;
    const dx = x2 - x1, dy = y2 - y1;
    const len = Math.hypot(dx, dy);
    if (len < 1) return;
    const nx = dx/len, ny = dy/len;
    const segs = 8, amp = 7, sl = len/segs;
    ctx.strokeStyle = color;
    ctx.lineWidth = selected ? 3 : 2;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    for (let i = 1; i < segs; i++) {
      const f = i/segs, off = i%2===0 ? amp : -amp;
      ctx.lineTo(x1+dx*f - ny*off, y1+dy*f + nx*off);
    }
    ctx.lineTo(x2, y2);
    ctx.stroke();
    const mx = (x1+x2)/2, my = (y1+y2)/2;
    ctx.fillStyle = c.base;
    ctx.font = '10px -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('spring', mx, my - 8);
  }

  static drawSmoothRail(ctx, x1, y1, x2, y2, hovered, selected) {
    const c = this._color('rail');
    const color = selected ? c.select : hovered ? c.hover : c.base;
    ctx.setLineDash([5, 4]);
    ctx.strokeStyle = color;
    ctx.lineWidth = selected ? 3 : 2;
    ctx.beginPath();
    ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.setLineDash([]);
    const mx = (x1+x2)/2, my = (y1+y2)/2;
    ctx.fillStyle = c.base;
    ctx.font = '10px -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('rail', mx, my - 8);
  }

  static drawFixedCoordinate(ctx, x1, y1, params, hovered, selected) {
    const c = this._color('fixed');
    const color = selected ? c.select : hovered ? c.hover : c.base;
    const s = 10;
    ctx.beginPath();
    ctx.arc(x1, y1, s, 0, Math.PI * 2);
    ctx.strokeStyle = color;
    ctx.lineWidth = selected ? 2.5 : 1.5;
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x1 - s*0.6, y1); ctx.lineTo(x1 + s*0.6, y1);
    ctx.moveTo(x1, y1 - s*0.6); ctx.lineTo(x1, y1 + s*0.6);
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.stroke();
    const coord = params?.coord ? params.coord.toUpperCase() : '';
    const val = params?.value !== undefined ? params.value : '';
    ctx.fillStyle = c.base;
    ctx.font = '10px -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(`${coord}=${val}`, x1, y1 + s + 14);
  }

  static drawLinearRelation(ctx, x1, y1, x2, y2, params, hovered, selected) {
    const c = this._color('linear');
    const color = selected ? c.select : hovered ? c.hover : c.base;
    ctx.setLineDash([4, 3]);
    ctx.strokeStyle = color;
    ctx.lineWidth = selected ? 2.5 : 1.5;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    if (x2 != null) ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.setLineDash([]);
    const mx = x2 != null ? (x1+x2)/2 : x1;
    const my = y2 != null ? (y1+y2)/2 : y1 - 14;
    ctx.fillStyle = c.base;
    ctx.font = '10px -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('linear', mx, my - 6);
  }

  static drawDistanceSum(ctx, x1, y1, x2, y2, via, hovered, selected) {
    const c = this._color('sum');
    const color = selected ? c.select : hovered ? c.hover : c.base;
    ctx.strokeStyle = color;
    ctx.lineWidth = selected ? 2.5 : 1.5;
    ctx.setLineDash([5, 4]);
    ctx.beginPath();
    ctx.moveTo(x1, y1); ctx.lineTo(via.x, via.y); ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.arc(via.x, via.y, 4, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    const mx = (x1+x2)/2, my = (y1+y2)/2;
    ctx.fillStyle = c.base;
    ctx.font = '10px -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('string', mx, my - 12);
  }

  static drawAngleConstraint(ctx, x1, y1, x2, y2, angle, hovered, selected) {
    const c = this._color('angle');
    const color = selected ? c.select : hovered ? c.hover : c.base;
    ctx.strokeStyle = color;
    ctx.lineWidth = selected ? 2.5 : 1.5;
    ctx.beginPath();
    ctx.moveTo(x1, y1); ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(x1, y1, 14, -angle, 0);
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.stroke();
    const deg = Math.round(Math.abs(angle) * 180 / Math.PI);
    const mx = (x1+x2)/2, my = (y1+y2)/2;
    ctx.fillStyle = c.base;
    ctx.font = '10px -apple-system, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(`${deg}°`, mx, my - 6);
  }

  static drawGrid(ctx, camera, width, height) {
    const gs = 1;
    const tl = camera.screenToWorld(0, 0);
    const br = camera.screenToWorld(width, height);
    const minX = Math.floor(tl.x/gs)*gs, maxX = Math.ceil(br.x/gs)*gs;
    const minY = Math.floor(br.y/gs)*gs, maxY = Math.ceil(tl.y/gs)*gs;

    ctx.strokeStyle = '#e8eaed';
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (let x = minX; x <= maxX; x += gs) {
      const sx = camera.worldToScreen(x, 0).x;
      ctx.moveTo(sx, 0); ctx.lineTo(sx, height);
    }
    for (let y = minY; y <= maxY; y += gs) {
      const sy = camera.worldToScreen(0, y).y;
      ctx.moveTo(0, sy); ctx.lineTo(width, sy);
    }
    ctx.stroke();

    ctx.strokeStyle = '#d0d7de';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    const o = camera.worldToScreen(0, 0);
    ctx.moveTo(o.x, 0); ctx.lineTo(o.x, height);
    ctx.moveTo(0, o.y); ctx.lineTo(width, o.y);
    ctx.stroke();

    ctx.fillStyle = '#959da5';
    ctx.font = '11px -apple-system, sans-serif';
    ctx.textAlign = 'left';
    for (let x = Math.max(minX, 0); x <= maxX; x += gs) {
      if (x === 0) continue;
      const sx = camera.worldToScreen(x, 0).x;
      ctx.fillText(`${x}m`, sx + 3, o.y - 3);
    }
    for (let y = Math.max(minY, 0); y <= maxY; y += gs) {
      if (y === 0) continue;
      const sy = camera.worldToScreen(0, y).y;
      ctx.fillText(`${y}m`, o.x + 3, sy - 3);
    }
  }
}
