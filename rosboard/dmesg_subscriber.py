#!/usr/bin/env python3

import select
import subprocess
import time
import threading
import traceback

class DMesgSubscriber(object):
    def __init__(self, callback):
        self.callback = callback
        self.process = None
        threading.Thread(target = self.start, daemon = True).start()

    def __del__(self):
        if self.process:
            self.process.terminate()
            self.process = None

    def unregister(self):
        if self.process:
            self.process.terminate()
            self.process = None

    def start(self):
        try:
            self.process = subprocess.Popen(['dmesg', '--follow'], stdout = subprocess.PIPE)
            p = select.poll()
            p.register(self.process.stdout, select.POLLIN)
    
            while True:
                time.sleep(0.1)

                if self.process is None:
                    break
                
                lines = []
                while p.poll(1):
                    lines.append(self.process.stdout.readline().decode('utf-8').strip())

                text = "\n".join(lines)
                if len(text) > 0:
                    self.callback("\n".join(lines))
        except:
            traceback.print_exc()


if __name__ == "__main__":
    # Run test
    DMesgSubscriber(lambda msg: print("Received msg: %s" % msg))
    time.sleep(100)
