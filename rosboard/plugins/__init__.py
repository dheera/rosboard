#!/usr/bin/env python3

import pathlib
import pkgutil
import sys
import traceback


_plugins = {}

def _load_plugins(dirname: str):
    global _plugins
    for importer, package_name, _ in pkgutil.iter_modules([dirname]):
        full_package_name = '%s.%s' % (dirname, package_name)
        if full_package_name in sys.modules:
            print(f'Plugin "{full_package_name}" not loaded, already present. Check name clashes.')
            continue

        try:
            module = importer.find_module(package_name).load_module(package_name)
            _plugins[package_name] = module
        except Exception as e:
            traceback.print_exception(e)

def get_plugin_js_files():
    global _plugins
    js_files = []

    for plugin in _plugins.values():
        js_files.extend(plugin.js_files)

    return js_files

def received_plugin_message(name: str, message):
    global _plugins
    if name not in _plugins:
        print(f'Plugin unknown "{name}"')
        return

    try:
        _plugins[name].receive(message)
    except Exception as e:
        traceback.print_exception(e)

_load_plugins(str(pathlib.Path(__file__).parent.resolve()))
