#!/usr/bin/env python3

import subprocess
import time
import threading
import traceback

class DMesgSubscriber(object):
    def __init__(self, callback):
        self.callback = callback
        self.process = None
        threading.Thread(target = self.loop, daemon = True).start()

    def __del__(self):
        if self.process:
            self.process.terminate()
            self.process = None
            
    def unregister(self):
        if self.process:
            self.process.terminate()
            self.process = None

    def loop(self):
        while True:
            self.start()
            time.sleep(2)

    def start(self):
        try:
            self.process = subprocess.Popen(['dmesg', '--follow'], stdout = subprocess.PIPE)
    
            while True:
                line = self.process.stdout.readline().decode('utf-8').strip()
                if len(line) > 0:
                    self.callback(line)
        except:
            traceback.print_exc()


if __name__ == "__main__":
    # Run test
    DMesgSubscriber(lambda msg: print("Received msg: %s" % msg))
    time.sleep(100)
