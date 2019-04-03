'use strict';

const fs = require('fs');
const path = require('path');
const util = require('util');
const md5 = require('md5');
const async = require('async');

const packages = require('./packages');
const fieldsUtil = require('./fields');
const IndentedWriter = require('./IndentedWriter.js');
const MsgSpec = require('./MessageSpec.js');

const Field = fieldsUtil.Field;

let packageCache = null;

const PKG_LOADING = 'loading';
const PKG_LOADED = 'loaded';

function createDirectory(directory) {
  let curPath = '/';
  const paths = directory.split(path.sep);

  function createLocal(dirPath) {
    return new Promise((resolve, reject) => {
      fs.mkdir(dirPath, err => {
        if (err && err.code !== 'EEXIST') {
          reject(err);
        }
        resolve();
      });
    });
  }

  return paths.reduce((prev, cur, index, array) => {
    curPath = path.join(curPath, array[index]);
    return prev.then(createLocal.bind(null, curPath));
  }, Promise.resolve());
}

function writeFile(filepath, data) {
  return new Promise((resolve, reject) => {
    fs.writeFile(filepath, data, err => {
      if (err) {
        reject(err);
      } else {
        resolve();
      }
    });
  });
}

class MessageManager {
  constructor() {
    let verbose = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : false;

    this._packageChain = [];
    this._loadingPkgs = new Map();

    this._verbose = verbose;
  }

  log() {
    if (this._verbose) {
      var _console;

      (_console = console).log.apply(_console, arguments);
    }
  }

  getCache() {
    return packageCache;
  }

  getMessageSpec(msgType) {
    let type = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : MsgSpec.MSG_TYPE;

    const messageName = fieldsUtil.getMessageNameFromMessageType(msgType);
    const pkg = fieldsUtil.getPackageNameFromMessageType(msgType);
    if (packageCache.hasOwnProperty(pkg)) {
      let pkgCache;
      switch (type) {
        case MsgSpec.MSG_TYPE:
          pkgCache = packageCache[pkg].messages;
          break;
        case MsgSpec.SRV_TYPE:
          pkgCache = packageCache[pkg].services;
          break;
      }
      if (pkgCache) {
        // be case insensitive here...
        if (pkgCache.hasOwnProperty(messageName)) {
          return pkgCache[messageName].msgSpec;
        }
        const lcName = messageName.toLowerCase();
        if (pkgCache.hasOwnProperty(lcName)) {
          return pkgCache[lcName].msgSpec;
        }
      }
    }
    // fall through
    return null;
  }

  buildPackageTree(outputDirectory) {
    let writeFiles = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : true;

    return this.initTree().then(() => {
      this._packageChain = this._buildMessageDependencyChain();

      // none of the loading here depends on message dependencies
      // so don't worry about doing it in order, just do it all...
      return Promise.all(this._packageChain.map(pkgName => {
        return this.loadPackage(pkgName, outputDirectory, false, writeFiles);
      }));
    }).catch(err => {
      console.error(err.stack);
      throw err;
    });
  }

  buildPackage(packageName, outputDirectory) {
    const deps = new Set();
    return this.initTree().then(() => {
      this.loadPackage(packageName, outputDirectory, true, true, depName => {
        if (!deps.has(depName)) {
          deps.add(depName);
          return true;
        }
        return false;
      });
    });
  }

  initTree() {
    let p;
    if (packageCache === null) {
      this.log('Traversing ROS_PACKAGE_PATH...');
      p = packages.findMessagePackages();
    } else {
      p = Promise.resolve();
    }
    return p.then(() => {
      packageCache = packages.getPackageCache();

      // load all the messages
      // TODO: only load messages we need
      this._loadMessagesInCache();
    });
  }

  loadPackage(packageName, outputDirectory) {
    let loadDeps = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : true;
    let writeFiles = arguments.length > 3 && arguments[3] !== undefined ? arguments[3] : true;
    let filterDepFunc = arguments.length > 4 && arguments[4] !== undefined ? arguments[4] : null;

    if (this._loadingPkgs.has(packageName)) {
      return Promise.resolve();
    }
    // else
    this.log('Loading package %s', packageName);
    this._loadingPkgs.set(packageName, PKG_LOADING);

    if (loadDeps) {
      // get an ordered list of dependencies for this message package
      const dependencies = this._buildMessageDependencyChain(this._getFullDependencyChain(packageName));

      // filter out any packages that have already been loaded or are loading
      let depsToLoad = dependencies;
      if (filterDepFunc && typeof filterDepFunc === 'function') {
        depsToLoad = dependencies.filter(filterDepFunc);
      }

      depsToLoad.forEach(depName => {
        this.loadPackage(depName, outputDirectory, loadDeps, filterDepFunc);
      });
    }

    // actions get parsed and are then cached with the rest of the messages
    // which is why there isn't a loadPackageActions
    if (writeFiles) {
      return this.initPackageWrite(packageName, outputDirectory).then(this.writePackageMessages.bind(this, packageName, outputDirectory)).then(this.writePackageServices.bind(this, packageName, outputDirectory)).then(() => {
        this._loadingPkgs.set(packageName, PKG_LOADED);
        console.log('Finished building package %s', packageName);
      });
    }
    // else
    return Promise.resolve();
  }

  initPackageWrite(packageName, jsMsgDir) {
    const packageDir = path.join(jsMsgDir, packageName);
    packageCache[packageName].directory = packageDir;

    return createDirectory(packageDir).then(() => {
      if (this.packageHasMessages(packageName) || this.packageHasActions(packageName)) {
        const msgDir = path.join(packageDir, 'msg');
        return createDirectory(msgDir).then(this.createMessageIndex.bind(this, packageName, msgDir));
      }
    }).then(() => {
      if (this.packageHasServices(packageName)) {
        const srvDir = path.join(packageDir, 'srv');
        return createDirectory(srvDir).then(this.createServiceIndex.bind(this, packageName, srvDir));
      }
    }).then(this.createPackageIndex.bind(this, packageName, packageDir));
  }

  createPackageIndex(packageName, directory) {
    const w = new IndentedWriter();
    w.write('module.exports = {').indent();

    const hasMessages = this.packageHasMessages(packageName) || this.packageHasActions(packageName);
    const hasServices = this.packageHasServices(packageName);
    if (hasMessages) {
      w.write('msg: require(\'./msg/_index.js\'),');
    }
    if (hasServices) {
      w.write('srv: require(\'./srv/_index.js\')');
    }
    w.dedent().write('};');

    return writeFile(path.join(directory, '_index.js'), w.get());
  }

  createIndex(packageName, directory, msgKey) {
    const messages = Object.keys(packageCache[packageName][msgKey]);
    const w = new IndentedWriter();
    w.write('module.exports = {').indent();

    messages.forEach(message => {
      w.write('%s: require(\'./%s.js\'),', message, message);
    });

    w.dedent().write('};');

    return writeFile(path.join(directory, '_index.js'), w.get());
  }

  createMessageIndex(packageName, directory) {
    return this.createIndex(packageName, directory, 'messages');
  }

  createServiceIndex(packageName, directory) {
    return this.createIndex(packageName, directory, 'services');
  }

  packageHasMessages(packageName) {
    return Object.keys(packageCache[packageName].messages).length > 0;
  }

  packageHasServices(packageName) {
    return Object.keys(packageCache[packageName].services).length > 0;
  }

  packageHasActions(packageName) {
    return Object.keys(packageCache[packageName].actions).length > 0;
  }

  writePackageMessages(packageName, jsMsgDir) {
    const msgDir = path.join(jsMsgDir, packageName, 'msg');

    const packageMsgs = packageCache[packageName].messages;
    const numMsgs = Object.keys(packageMsgs).length;
    if (numMsgs > 0) {
      this.log('Building %d messages from %s', numMsgs, packageName);
      const promises = [];
      Object.keys(packageMsgs).forEach(msgName => {
        const spec = packageMsgs[msgName].msgSpec;
        this.log(`Building message ${spec.packageName}/${spec.messageName}`);
        promises.push(writeFile(path.join(msgDir, `${msgName}.js`), spec.generateMessageClassFile()));
      });

      return Promise.all(promises);
    }
    // else
    return Promise.resolve();
  }

  writePackageServices(packageName, jsMsgDir) {
    const msgDir = path.join(jsMsgDir, packageName, 'srv');

    const packageSrvs = packageCache[packageName].services;
    const numSrvs = Object.keys(packageSrvs).length;
    if (numSrvs > 0) {
      this.log('Building %d services from %s', numSrvs, packageName);
      const promises = [];
      Object.keys(packageSrvs).forEach(srvName => {
        const spec = packageSrvs[srvName].msgSpec;
        this.log(`Building service ${spec.packageName}/${spec.messageName}`);
        promises.push(writeFile(path.join(msgDir, `${srvName}.js`), spec.generateMessageClassFile()));
      });

      return Promise.all(promises);
    }
    // else
    return Promise.resolve();
  }

  _loadMessagesInCache() {
    this.log('Loading messages...');
    Object.keys(packageCache).forEach(packageName => {

      const packageInfo = packageCache[packageName];
      const packageDeps = new Set();

      packageInfo.forEach = (item, func) => {
        let itemInfo = packageInfo[item];
        Object.keys(itemInfo).forEach(item => {
          const ret = func(item, itemInfo[item]);
          if (ret) {
            itemInfo[item][ret.key] = ret.val;
          }
        });
      };

      packageInfo.forEach('messages', (message, _ref) => {
        let file = _ref.file;

        this.log('Loading message %s from %s', message, file);
        const msgSpec = MsgSpec.create(this, packageName, message, MsgSpec.MSG_TYPE, file);

        msgSpec.getMessageDependencies(packageDeps);

        return {
          key: 'msgSpec',
          val: msgSpec
        };
      });

      packageInfo.forEach('services', (message, _ref2) => {
        let file = _ref2.file;

        this.log('Loading service %s from %s', message, file);
        const msgSpec = MsgSpec.create(this, packageName, message, MsgSpec.SRV_TYPE, file);

        msgSpec.getMessageDependencies(packageDeps);

        return {
          key: 'msgSpec',
          val: msgSpec
        };
      });

      packageInfo.forEach('actions', (message, _ref3) => {
        let file = _ref3.file;

        this.log('Loading action %s from %s', message, file);
        const msgSpec = MsgSpec.create(this, packageName, message, MsgSpec.ACTION_TYPE, file);

        // cache the individual messages for later lookup (needed when writing files)
        const packageMsgs = packageInfo.messages;
        msgSpec.getMessages().forEach(spec => {
          // only write this action if it doesn't exist yet - this should be expected if people
          // have already run catkin_make, as it will generate action message definitions that
          // will just get loaded as regular messages
          if (!packageMsgs.hasOwnProperty(spec.messageName)) {
            packageMsgs[spec.messageName] = { file: null, msgSpec: spec };
          }
        });

        msgSpec.getMessageDependencies(packageDeps);

        return {
          key: 'msgSpec',
          val: msgSpec
        };
      });

      packageInfo.dependencies = packageDeps;
    });
  }

  _getFullDependencyChain(msgPackage) {
    let originalPackage = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : null;
    let dependencyList = arguments.length > 2 && arguments[2] !== undefined ? arguments[2] : null;

    if (dependencyList === null) {
      dependencyList = packageCache[msgPackage].dependencies;
    }
    if (originalPackage === null) {
      originalPackage = msgPackage;
    }
    const localDeps = packageCache[msgPackage].dependencies;
    localDeps.forEach(dep => {
      if (dep === originalPackage) {
        throw new Error('Found circular dependency while building chain');
      }
      dependencyList.add(dep);
      this._getFullDependencyChain(dep, originalPackage, dependencyList);
    });

    return dependencyList;
  }

  _recurseDependencyChain(dependencyChain, packageName) {
    const packageDeps = packageCache[packageName].dependencies;
    let maxInsertionIndex = -1;
    packageDeps.forEach(depName => {
      const depIndex = dependencyChain.indexOf(depName);
      if (depIndex === -1) {
        // this dependency is not yet in the list anywhere
        const insertionIndex = this._recurseDependencyChain(dependencyChain, depName);
        if (insertionIndex > maxInsertionIndex) {
          maxInsertionIndex = insertionIndex;
        }
      } else {
        maxInsertionIndex = depIndex;
      }
    });

    if (maxInsertionIndex < 0) {
      dependencyChain.unshift(packageName);
      return 0;
    } else {
      dependencyChain.splice(maxInsertionIndex + 1, 0, packageName);
      return maxInsertionIndex + 1;
    }
  }

  _buildMessageDependencyChain() {
    let packageList = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : null;

    if (packageList === null) {
      packageList = Object.keys(packageCache);
    }
    const dependencyChain = [];
    packageList.forEach(this._recurseDependencyChain.bind(this, dependencyChain));
    return dependencyChain;
  }
}

module.exports = MessageManager;