'use strict';

const fs = require('fs'),
      path = require('path'),
      walker = require('walker'),
      async = require('async');

let packageCache = {};
const cache = {};

function packageWalk(directory, symlinks) {
  var noSubDirs = new Set();
  var stopped = false;
  symlinks = symlinks || [];

  return walker(directory).filterDir(function (dir, stat) {
    // return true to explore this directory...

    // Exclude any subdirectory to an excluded directory
    const ignoreFile = path.join(dir, 'CATKIN_IGNORE');

    function fileExists(file) {
      try {
        fs.statSync(file);
      } catch (err) {
        return false;
      }
      return true;
    }

    // if CATKIN_IGNORE exists, just don't even look
    if (fileExists(ignoreFile)) {
      return false;
    }

    // Check if this is a package - if it is, we need to add it to noSubDirs now
    // Otherwise it could start exploring subdirs before discovering its a package.
    const packageFile = path.join(dir, 'package.xml');
    const manifestFile = path.join(dir, 'manifest.xml');

    if (fileExists(packageFile) || fileExists(manifestFile)) {
      noSubDirs.add(dir);
    }

    // don't explore the directory if this dir's parent is
    // in noSubDirs or we have stopped
    const parent = path.dirname(dir);
    return !(noSubDirs.has(parent) || stopped);
  }).on('file', function (file, stat) {
    var shortname = path.basename(file);
    var dir = path.dirname(file);

    if (shortname === 'manifest.xml' || shortname === 'package.xml') {
      this.emit('package', path.basename(dir), dir, file);
    } else if (shortname === 'rospack_nosubdirs') {
      // Explicitly asked to not go into subdirectories
      noSubDirs.add(dir);
    }
  }).on('symlink', function (symlink, stat) {
    var walker = this;
    fs.readlink(symlink, function (error, link) {
      if (error) {
        return;
      }

      var destination = path.resolve(path.dirname(symlink), link);

      // Stores symlinks to avoid circular references
      if (~symlinks.indexOf(destination)) {
        return;
      } else {
        symlinks.concat(destination);
      }

      fs.stat(destination, function (error, stat) {
        if (error) {
          return;
        } else if (stat.isDirectory()) {
          walker.emit('dir', destination, stat);
          return walker.go(destination);
        }
      });
    });
  }).on('error', function (err) {
    console.error('Error while walking directory %s: %s', directory, err.message);
  }).on('end', function () {
    stopped = true;
    // Quit emitting
    this.emit = function () {};
  });
}

function messageWalk(directory, symlinks) {
  var stopped = false;
  symlinks = symlinks || [];

  return walker(directory).filterDir(function (dir, stat) {
    // Exclude any subdirectory to an excluded directory
    const ignoreFile = path.join(dir, 'CATKIN_IGNORE');

    function fileExists(file) {
      try {
        fs.statSync(file);
      } catch (err) {
        return false;
      }
      return true;
    }

    // if CATKIN_IGNORE exists, just don't even look
    if (fileExists(ignoreFile)) {
      return false;
    }
    // else
    return true;
  }).on('file', function (file) {
    let extension = path.extname(file);
    let dir = path.dirname(file);
    let name = path.basename(file, extension);

    if (extension === '.msg') {
      this.emit('message', name, file);
    } else if (extension === '.srv') {
      this.emit('service', name, file);
    } else if (extension === '.action') {
      this.emit('action', name, file);
    }
  }).on('symlink', function (symlink, stat) {
    var walker = this;
    fs.readlink(symlink, function (error, link) {
      if (error) {
        return;
      }

      var destination = path.resolve(path.dirname(symlink), link);

      // Stores symlinks to avoid circular references
      if (~symlinks.indexOf(destination)) {
        return;
      } else {
        symlinks.concat(destination);
      }

      fs.stat(destination, function (error, stat) {
        if (error) {
          return;
        } else if (stat.isDirectory()) {
          walker.emit('dir', destination, stat);
          return walker.go(destination);
        }
      });
    });
  }).on('error', function (err) {
    console.error('Error while walking directory %s: %s', directory, err.message);
  }).on('end', function () {
    stopped = true;
    // Quit emitting
    this.emit = function () {};
  });
}

function findPackageInDirectory(directory, packageName, callback) {
  var found = false;
  return packageWalk(directory).on('package', function (name, dir) {
    if (name === packageName) {
      this.emit('stop');
      found = true;
      callback(null, dir);
    }
  }).on('end', function () {
    if (!found) {
      var error = new Error('ENOTFOUND - Package ' + packageName + ' not found');
      error.name = 'PackageNotFoundError';
      callback(error);
    }
  });
}

function findPackagesInDirectory(directory) {
  const promises = [];
  promises.push(new Promise(resolve => {
    const subPromises = [];
    packageWalk(directory).on('package', (packageName, dir, fileName) => {
      packageName = packageName.toLowerCase();
      if (!packageCache.hasOwnProperty(packageName)) {
        const packageEntry = {
          directory: dir,
          messages: {},
          services: {},
          actions: {}
        };
        subPromises.push(new Promise(resolve => {
          messageWalk(dir, null).on('message', (name, file) => {
            packageEntry.messages[name] = { file };
          }).on('service', (name, file) => {
            packageEntry.services[name] = { file };
          }).on('action', (name, file) => {
            packageEntry.actions[name] = { file };
          }).on('end', () => {
            if (Object.keys(packageEntry.messages).length > 0 || Object.keys(packageEntry.services).length > 0 || Object.keys(packageEntry.actions).length > 0) {
              packageCache[packageName] = packageEntry;
            }
            resolve();
          });
        }));
      }
    }).on('end', () => {
      Promise.all(subPromises).then(resolve);
    });
  }));

  return Promise.all(promises);
}

function findPackageInDirectoryChain(directories, packageName, callback) {
  if (directories.length < 1) {
    var error = new Error('ENOTFOUND - Package ' + packageName + ' not found');
    error.name = 'PackageNotFoundError';
    callback(error);
  } else {
    findPackageInDirectory(directories.shift(), packageName, function (error, directory) {
      if (error) {
        if (error.name === 'PackageNotFoundError') {
          // Recursive call, try in next directory
          return findPackageInDirectoryChain(directories, packageName, callback);
        } else {
          callback(error);
        }
      } else {
        callback(null, directory);
      }
    });
  }
}

function findPackagesInDirectoryChain(directories) {
  const funcs = directories.map(directory => {
    return findPackagesInDirectory.bind(null, directory);
  });
  return funcs.reduce((prev, cur, index) => {
    return prev.then(() => {
      return cur();
    });
  }, Promise.resolve());
}

// ---------------------------------------------------------

function getRosEnvVar(envVarName) {
  const envVar = process.env[envVarName];

  if (!envVar) {
    throw new Error(`Unable to find required environment variable ${envVarName}`);
  }

  return envVar;
}

function getRosPackagePath() {
  return getRosEnvVar('ROS_PACKAGE_PATH');
}

function getRosRoot() {
  return getRosEnvVar('ROS_ROOT');
}

// Implements the same crawling algorithm as rospack find
// See http://ros.org/doc/api/rospkg/html/rospack.html
// packages = {};
exports.findPackage = function (packageName, callback) {
  var directory = cache[packageName.toLowerCase()];
  if (directory) {
    callback(null, directory);
    return;
  }

  const packagePath = getRosPackagePath();
  var rosPackagePaths = packagePath.split(':');
  var directories = rosPackagePaths;
  return findPackageInDirectoryChain(directories, packageName, function (err, directory) {
    cache[packageName.toLowerCase()] = directory;
    callback(err, directory);
  });
};

exports.findMessagePackages = function () {
  var packagePath = getRosPackagePath();
  var rosPackagePaths = packagePath.split(':');
  return findPackagesInDirectoryChain(rosPackagePaths);
};

exports.getPackageCache = function () {
  return Object.assign({}, packageCache);
};

function forEachPackageInDirectory(directory, list, onEnd) {
  fs.access(directory, fs.R_OK, err => {
    if (!err) {
      packageWalk(directory).on('package', function (name, dir) {
        list.push(dir);
      }).on('end', onEnd);
    } else {
      onEnd();
    }
  });
}

/** get list of package directories */
exports.getAllPackages = function (done) {
  var rosRoot = getRosRoot();
  var packagePath = getRosPackagePath();
  var rosPackagePaths = packagePath.split(':');
  var directories = [rosRoot].concat(rosPackagePaths);
  async.reduce(directories, [], function (memo, directory, callback) {
    forEachPackageInDirectory(directory, memo, function () {
      callback(null, memo);
    });
  }, function (err, directories) {
    directories.forEach(function (directory) {
      var packageName = path.basename(directory);
      cache[packageName.toLowerCase()] = directory;
    });
    done(err, directories);
  });
};