'use strict';

var _slicedToArray = function () { function sliceIterator(arr, i) { var _arr = []; var _n = true; var _d = false; var _e = undefined; try { for (var _i = arr[Symbol.iterator](), _s; !(_n = (_s = _i.next()).done); _n = true) { _arr.push(_s.value); if (i && _arr.length === i) break; } } catch (err) { _d = true; _e = err; } finally { try { if (!_n && _i["return"]) _i["return"](); } finally { if (_d) throw _e; } } return _arr; } return function (arr, i) { if (Array.isArray(arr)) { return arr; } else if (Symbol.iterator in Object(arr)) { return sliceIterator(arr, i); } else { throw new TypeError("Invalid attempt to destructure non-iterable instance"); } }; }();

const CLEAN_REGEX = /\/\//g;

// http://wiki.ros.org/Names
class Names {
  constructor() {
    this._remappings = {};
    this._namespace = '';
  }

  init(remappings, namespace) {
    this._namespace = namespace;
    this._remappings = {};

    Object.keys(remappings).forEach(left => {
      if (left && !left.startsWith('_')) {
        const right = remappings[left];

        const resolvedLeft = this.resolve(left, false);
        const resolvedRight = this.resolve(right, false);

        this._remappings[resolvedLeft] = resolvedRight;
      }
    });
  }

  validate(name) {
    let throwError = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : false;

    if (typeof name !== 'string') {
      if (throwError) {
        throw new Error('Unable to validate non-string name');
      }
      return false;
    }

    const len = name.length;
    if (len === 0) {
      return true;
    }
    // else
    // First character must be alphanumeric, '/', or '~'
    const c = name[0];
    if (!isAlpha(c) && c !== '/' && c !== '~') {
      if (throwError) {
        throw new Error(`Character [${c}] is not valid as the first character in Graph Resource Name [${name}].  Valid characters are a-z, A-Z, / and in some cases ~.`);
      }
      // else
      return false;
    }

    for (let i = 1; i < len; ++i) {
      if (!isValidCharInName(name[i])) {
        if (throwError) {
          throw new Error(`Character [${c}] at element [${i}] is not valid in Graph Resource Name [${name}].  Valid characters are a-z, A-Z, 0-9, / and _.`);
        }
        // else
        return false;
      }
    }

    return true;
  }

  clean(name) {
    name = name.replace(CLEAN_REGEX, '/');

    if (name.endsWith('/')) {
      return name.substr(0, -1);
    }
    // else
    return name;
  }

  append(left, right) {
    return this.clean(left + '/' + right);
  }

  remap(name) {
    return this.resolve(name, true);
  }

  /**
   * @param [namespace] {string} namespace to resolve name to. If not provided, node's namespace will be used
   * @param name {string} name to resolve
   * @param [remap] {bool} flag indicating if we should also attempt to remap the name
   */
  resolve() {
    for (var _len = arguments.length, args = Array(_len), _key = 0; _key < _len; _key++) {
      args[_key] = arguments[_key];
    }

    var _parseResolveArgs = this._parseResolveArgs(args),
        _parseResolveArgs2 = _slicedToArray(_parseResolveArgs, 3);

    let namespace = _parseResolveArgs2[0],
        name = _parseResolveArgs2[1],
        remap = _parseResolveArgs2[2];


    this.validate(name, true);

    if (name.length === 0) {
      if (namespace.length === 0) {
        return '/';
      } else if (namespace[0] === '/') {
        return namespace;
      }
      // else
      return '/' + namespace;
    }

    if (name.startsWith('~')) {
      name = name.replace('~', this._namespace + '/');
    }

    if (!name.startsWith('/')) {
      name = namespace + '/' + name;
    }

    name = this.clean(name);

    if (remap) {
      name = this._remap(name);
    }

    return name;
  }

  parentNamespace(name) {
    this.validate(name, true);

    if (name.length === 0) {
      return '';
    } else if (name === '/') {
      return '/';
    }

    let p = name.lastIndexOf('/');
    if (p === name.length - 1) {
      p = name.lastIndexOf('/', p - 1);
    }

    if (p < 0) {
      return '';
    } else if (p === 0) {
      return '/';
    }
    // else
    return name.substring(0, p);
  }

  _remap(name) {
    return this._remappings[name] || name;
  }

  _parseResolveArgs(args) {
    let name,
        namespace = this._namespace,
        remap = true;
    switch (args.length) {
      case 0:
        name = '';
        break;
      case 1:
        name = args[0];
        break;
      case 2:
        if (typeof args[1] === 'string') {
          var _args = _slicedToArray(args, 2);

          namespace = _args[0];
          name = _args[1];
        } else {
          var _args2 = _slicedToArray(args, 2);

          name = _args2[0];
          remap = _args2[1];
        }
        break;
      default:
        var _args3 = _slicedToArray(args, 3);

        namespace = _args3[0];
        name = _args3[1];
        remap = _args3[2];

        break;
    }

    return [namespace, name, remap];
  }
}

module.exports = new Names();

//------------------------------------------------------------------
// Local Helper functions
//------------------------------------------------------------------


function isAlpha(char) {
  if (char >= 'A' && char <= 'Z') {
    return true;
  } else if (char >= 'a' && char <= 'z') {
    return true;
  }
  // else
  return false;
}

function isAlnum(char) {
  if (isAlpha(char)) {
    return true;
  } else if (char >= '0' && char <= '9') {
    return true;
  }
  // else
  return false;
}

function isValidCharInName(char) {
  return isAlnum(char) || char == '/' || char == '_';
}