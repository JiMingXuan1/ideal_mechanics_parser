export class CommandHistory {
  constructor() {
    this._stack = [];
    this._cursor = -1;
    this._maxSize = 100;
  }

  push(command) {
    this._stack = this._stack.slice(0, this._cursor + 1);
    this._stack.push(command);
    if (this._stack.length > this._maxSize) {
      this._stack.shift();
    }
    this._cursor = this._stack.length - 1;
  }

  undo() {
    if (this._cursor < 0) return false;
    this._stack[this._cursor].undo();
    this._cursor--;
    return true;
  }

  redo() {
    if (this._cursor + 1 >= this._stack.length) return false;
    this._cursor++;
    this._stack[this._cursor].redo();
    return true;
  }

  canUndo() {
    return this._cursor >= 0;
  }

  canRedo() {
    return this._cursor + 1 < this._stack.length;
  }
}
