export class ErrorToast {
  constructor(eventBus) {
    this._el = document.createElement('div');
    this._el.style.cssText = `
      position: absolute; bottom: 24px; right: 24px;
      background: #f85149; color: #fff;
      padding: 10px 18px; border-radius: 6px;
      font-family: 'Courier New', monospace; font-size: 13px;
      z-index: 200; max-width: 400px;
      transform: translateY(120%); opacity: 0;
      transition: all 0.3s ease-out;
      pointer-events: none;
    `;
    this._el.textContent = '';
    document.body.appendChild(this._el);
    this._timer = null;

    eventBus.on('ERROR', ({ message }) => this.show(message));
  }

  show(message) {
    this._el.textContent = `ERROR: ${message}`;
    this._el.style.transform = 'translateY(0)';
    this._el.style.opacity = '1';
    if (this._timer) clearTimeout(this._timer);
    this._timer = setTimeout(() => {
      this._el.style.transform = 'translateY(120%)';
      this._el.style.opacity = '0';
    }, 5000);
  }
}
