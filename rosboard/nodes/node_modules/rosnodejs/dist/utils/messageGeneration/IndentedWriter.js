'use strict';

const util = require('util');

class IndentedWriter {
  constructor() {
    this._str = '';
    this._indentation = 0;
  }

  write(args) {
    let formattedStr = util.format.apply(this, arguments);
    if (this.isIndented()) {
      for (let i = 0; i < this._indentation; ++i) {
        this._str += '  ';
      }
    }
    this._str += formattedStr;
    return this.newline();
  }

  newline() {
    let indentDir = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : undefined;

    this._str += '\n';
    if (indentDir === undefined) {
      return this;
    } else if (indentDir > 0) {
      return this.indent();
    } else if (indentDir < 0) {
      return this.dedent();
    }
    // else
    return this;
  }

  indent() {
    ++this._indentation;
    if (arguments.length > 0) {
      return this.write.apply(this, arguments);
    }
    // else
    return this;
  }

  isIndented() {
    return this._indentation > 0;
  }

  dedent() {
    --this._indentation;
    if (this._indentation < 0) {
      this.resetIndent();
    }
    if (arguments.length > 0) {
      return this.write.apply(this, arguments);
    }
    // else
    return this;
  }

  resetIndent() {
    this._indentation = 0;
    return this;
  }

  dividingLine() {
    return this.write('//-----------------------------------------------------------');
  }

  get() {
    return this._str;
  }
}

module.exports = IndentedWriter;