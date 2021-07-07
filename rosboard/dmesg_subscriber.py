#!/usr/bin/env python3

import subprocess
import time
import threading
import traceback

class DMesgSubscriber(object):
    def __init__(self, callback):
        self.callback = callback
        threading.Thread(target = self.loop, daemon = True).start()

    def loop(self):
        while True:
            self.start()
            time.sleep(2)

    def start(self):
        try:
            process = subprocess.Popen(['dmesg', '--follow'], stdout = subprocess.PIPE)
    
            while True:
                line = process.stdout.readline().decode('utf-8').strip()
                if len(line) > 0:
                    self.callback(line)
        except:
            traceback.print_exc()


if __name__ == "__main__":
    # Run test
    DMesgSubscriber(lambda msg: print("Received msg: %s" % msg))
    time.sleep(100)
