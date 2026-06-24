export class Toolbar {
  constructor(container, eventBus) {
    this.container = container;
    this.eb = eventBus;
    this._activeTool = 'select';
    this._btnMap = {};

    this._add('select',    '↖', 'Select',  'select');
    this._add('add_point', '⊕', 'Point',   'add_node');
    this._add('add_anchor','⊡', 'Anchor',  'add_anchor');
    this._add('add_edge',  '╳', 'Edge',    'add_edge');
    this._add('delete',    '✕', 'Delete',  'delete');
    this._stopBtn = this._add('stop', '⏹', 'Stop', 'stop', false, true);
    this._stopBtn.style.display = 'none';
    this._runBtn = this._add('run', '▶', 'Run', 'run', true);

    this.eb.on('TOOL_SET', ({ tool }) => this._activate(tool));
    this.eb.on('SIM_STATE', (s) => this._onSimState(s));
  }

  _add(id, icon, label, toolId, isRun, isStop) {
    const btn = document.createElement('div');
    btn.className = 'tool-btn' + (isRun ? ' run-btn' : '');
    btn.innerHTML = `<div class="icon">${icon}</div><div class="label">${label}</div>`;
    btn.addEventListener('click', () => {
      if (isRun) {
        this.eb.emit('CMD_SIMULATION_START_GUI');
      } else if (isStop) {
        this.eb.emit('CMD_SIMULATION_STOP');
      } else {
        this._activate(toolId);
        this.eb.emit('TOOL_SET', { tool: toolId });
      }
    });
    this.container.appendChild(btn);
    this._btnMap[toolId] = btn;
    return btn;
  }

  _onSimState(state) {
    const icon = this._runBtn.querySelector('.icon');
    const label = this._runBtn.querySelector('.label');
    if (state === 'playing') {
      icon.textContent = '⏸';
      label.textContent = 'Pause';
      this._runBtn.classList.add('active');
      this._stopBtn.style.display = '';
    } else if (state === 'paused') {
      icon.textContent = '▶';
      label.textContent = 'Resume';
      this._runBtn.classList.add('active');
      this._stopBtn.style.display = '';
    } else {
      icon.textContent = '▶';
      label.textContent = 'Run';
      this._runBtn.classList.remove('active');
      this._stopBtn.style.display = 'none';
    }
  }

  setStatus(text) {
    const el = document.getElementById('status-text');
    if (el) el.innerHTML = text;
  }

  _activate(toolId) {
    this._activeTool = toolId;
    for (const [k, v] of Object.entries(this._btnMap)) {
      if (k === 'run') continue;
      v.classList.toggle('active', k === toolId);
    }
  }
}
