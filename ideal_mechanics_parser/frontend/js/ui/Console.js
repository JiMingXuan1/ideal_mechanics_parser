export class Console {
  constructor(container, eventBus, stateMachine) {
    this.container = container;
    this.eb = eventBus;
    this.sm = stateMachine;
    this._visible = false;

    this._historyEl = document.getElementById('console-history');
    this._inputEl = document.getElementById('console-input');

    this._inputEl.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') this._execute();
      if (e.key === 'Escape') this.hide();
    });

    document.addEventListener('keydown', (e) => {
      if (e.key === '`' && document.activeElement !== this._inputEl) {
        e.preventDefault();
        this.toggle();
      }
    });

    this.eb.on('CONSOLE_LOG', ({ level, text }) => this._log(level, text));
    this.eb.on('CMD_SIMULATION_STOP', () => this.hide());

    this._log('info', 'Ideal Mechanics Parser Console v0.1');
    this._log('info', 'Type "help" for available commands.');
  }

  toggle() {
    if (this._visible) this.hide();
    else this.show();
  }

  show() {
    this._visible = true;
    this.container.classList.remove('hidden');
    this._inputEl.focus();
    this.eb.emit('CONSOLE_TOGGLED');
  }

  hide() {
    this._visible = false;
    this.container.classList.add('hidden');
    this.eb.emit('CONSOLE_TOGGLED');
  }

  _execute() {
    const raw = this._inputEl.value.trim();
    if (!raw) return;
    this._inputEl.value = '';
    this._log('info', `> ${raw}`);
    this._dispatch(raw);
  }

  _dispatch(raw) {
    const parts = raw.split(/\s+/);
    const cmd = parts[0].toLowerCase();

    switch (cmd) {
      case 'gamemode': {
        const mode = parts[1];
        if (mode !== 'xy' && mode !== 'xz') {
          this._log('error', `Invalid gamemode "${mode}". Use "xy" or "xz".`);
          return;
        }
        this.eb.emit('CMD_GAMEMODE_CHANGE', mode.toUpperCase());
        this._log('success', `View plane set to ${mode.toUpperCase()}`);
        break;
      }
      case 'duration': {
        const dur = parseFloat(parts[1]);
        if (isNaN(dur) || dur <= 0) {
          this._log('error', 'Usage: duration <seconds> (e.g. duration 60)');
          return;
        }
        this.sm._simDuration = dur;
        this._log('success', `Simulation duration set to ${dur}s (default 300s)`);
        break;
      }
      case 'run': {
        this.eb.emit('CMD_SIMULATION_START_GUI');
        break;
      }
      case 'stop': {
        if (this.sm.mode !== 'simulation') {
          this._log('error', 'No simulation running.');
          return;
        }
        this.eb.emit('CMD_SIMULATION_STOP');
        this._log('success', 'Simulation stopped.');
        break;
      }
      case 'clear': {
        this.eb.emit('CMD_CANVAS_CLEAR');
        this._log('success', 'Canvas cleared.');
        break;
      }
      case 'set': {
        const key = parts[1];
        const val = parts.slice(2).join(' ');
        if (!key || !val) {
          this._log('error', 'Usage: set <key> <value>');
          return;
        }
        if (this.sm.selectedCount !== 1) {
          this._log('error', 'Select exactly one entity to modify.');
          return;
        }
        const id = [...this.sm.selectedEntityIds][0];
        this.eb.emit('CMD_ENTITY_MODIFY', { id, key, value: val });
        this._log('success', `${id}.${key} = ${val}`);
        break;
      }
      case 'undo': {
        this.eb.emit('CMD_UNDO');
        this._log('info', 'Undo');
        break;
      }
      case 'redo': {
        this.eb.emit('CMD_REDO');
        this._log('info', 'Redo');
        break;
      }
      case 'help': {
        this._log('info', 'Commands:');
        this._log('info', '  gamemode xy|xz  — Switch plane (xy=no gravity, xz=gravity)');
        this._log('info', '  duration <sec>  — Set max sim time (default 300)');
        this._log('info', '  run / stop      — Start / stop simulation');
        this._log('info', '  Space           — Pause / resume (while running)');
        this._log('info', '  ← / →           — Step frame by frame (paused)');
        this._log('info', '  clear           — Clear all entities');
        this._log('info', '  set <k> <v>     — Modify selected entity property');
        this._log('info', '  undo / redo     — Undo/Redo');
        this._log('info', '  help            — This help');
        break;
      }
      default: {
        this._log('error', `Unknown command "${cmd}". Type "help".`);
      }
    }
  }

  _log(level, text) {
    const el = document.createElement('div');
    el.className = `entry ${level}`;
    el.textContent = text;
    this._historyEl.appendChild(el);
    this._historyEl.scrollTop = this._historyEl.scrollHeight;
  }
}
