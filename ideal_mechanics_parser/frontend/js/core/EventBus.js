export class EventBus {
  constructor() {
    this._handlers = {};
  }

  on(event, callback) {
    if (!this._handlers[event]) {
      this._handlers[event] = [];
    }
    this._handlers[event].push(callback);
    return this;
  }

  off(event, callback) {
    const handlers = this._handlers[event];
    if (!handlers) return this;
    const idx = handlers.indexOf(callback);
    if (idx !== -1) handlers.splice(idx, 1);
    return this;
  }

  emit(event, data) {
    const handlers = this._handlers[event];
    if (!handlers) return this;
    for (const cb of handlers.slice()) {
      cb(data);
    }
    return this;
  }
}
