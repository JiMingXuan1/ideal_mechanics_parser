const EDGE_FIELDS = {
  IdealRod: ['length'],
  IdealSpring: ['k', 'l0'],
  SmoothRail: ['expr'],
  FixedCoordinate: [
    { key: 'coord', type: 'select', options: ['x', 'y'] },
    { key: 'value', type: 'number' },
  ],
  LinearRelation: ['coeffs', { key: 'constant', type: 'number' }],
  DistanceSum: ['via_id', { key: 'length', type: 'number' }],
  AngleConstraint: ['angle'],
};

export class PropertiesPanel {
  constructor(container, eventBus, stateMachine) {
    this.container = container;
    this.eb = eventBus;
    this.sm = stateMachine;
    this._body = document.getElementById('properties-body');
    this._selectedId = null;
    this._isEdge = false;

    this.eb.on('SELECT', ({ id, isEdge }) => {
      this._selectedId = id;
      this._isEdge = !!isEdge;
      this.container.classList.remove('hidden');
      this._body.innerHTML = '';
      this._render();
    });

    this.eb.on('DESELECT_ALL', () => {
      this._selectedId = null;
      this.container.classList.add('hidden');
    });
  }

  _render() {
    if (this._isEdge) {
      const edge = this.sm.getEdge(this._selectedId);
      if (edge) this._renderEdge(edge);
    } else {
      const entity = this.sm.getEntity(this._selectedId);
      if (entity) this._renderEntity(entity);
    }
  }

  _renderEntity(entity) {
    this._row('ID', entity.id);
    this._row('Type', entity.type);

    if (entity.type === 'Anchor') {
      this._inp('x', entity.x, (v) => { entity.x = v; });
      this._inp('y', entity.y, (v) => { entity.y = v; });
    } else if (entity.type === 'MassPoint') {
      this._inp('m', entity.params.m || 1.0, (v) => { entity.params.m = v; });
      this._inp('x', entity.x, (v) => { entity.x = v; });
      this._inp('y', entity.y, (v) => { entity.y = v; });
      this._inp('vx', entity.vx || 0, (v) => { entity.vx = v; });
      this._inp('vy', entity.vy || 0, (v) => { entity.vy = v; });
    }
  }

  _renderEdge(edge) {
    this._row('ID', edge.id);
    this._sel('Type', edge.type,
      ['IdealRod', 'IdealSpring', 'SmoothRail', 'FixedCoordinate', 'LinearRelation', 'DistanceSum', 'AngleConstraint'],
      (nv) => {
        edge.type = nv;
        edge.params = {};
        this._body.innerHTML = '';
        this._renderEdge(edge);
      }
    );
    this._row('From', edge.from);
    if (edge.to) this._row('To', edge.to);

    const fields = EDGE_FIELDS[edge.type] || [];
    if (!edge.params) edge.params = {};

    for (const f of fields) {
      const key = typeof f === 'string' ? f : f.key;
      const type = typeof f === 'string' ? 'text' : f.type;
      const opts = type === 'select' ? f.options : null;
      const val = edge.params[key] !== undefined ? edge.params[key] : '';

      if (type === 'select') {
        this._sel(key, val, opts, (nv) => { edge.params[key] = nv; });
      } else {
        this._inp(key, val, (nv) => { edge.params[key] = nv; });
      }
    }
  }

  _row(label, value) {
    const d = document.createElement('div');
    d.className = 'prop-row';
    d.innerHTML = `<label>${label}</label><span>${value}</span>`;
    this._body.appendChild(d);
  }

  _inp(key, value, onChange) {
    const d = document.createElement('div');
    d.className = 'prop-row';
    d.innerHTML = `<label>${key}</label><input type="text" value="${value}" />`;
    const inp = d.querySelector('input');
    inp.addEventListener('input', () => {
      const raw = inp.value;
      const n = parseFloat(raw);
      onChange(isNaN(n) ? raw : n);
    });
    this._body.appendChild(d);
  }

  _sel(key, value, options, onChange) {
    const d = document.createElement('div');
    d.className = 'prop-row';
    let html = `<label>${key}</label><select>`;
    for (const o of options) {
      html += `<option value="${o}"${o === value ? ' selected' : ''}>${o}</option>`;
    }
    html += '</select>';
    d.innerHTML = html;
    d.querySelector('select').addEventListener('change', (e) => {
      onChange(e.target.value);
    });
    this._body.appendChild(d);
  }
}
