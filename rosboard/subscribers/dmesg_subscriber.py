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
            # Use 'dmesg --human --read-clear' once to dump existing log, then follow via journalctl
            # This avoids the "Operation not permitted" error in containers / restricted environments
            # where dmesg --follow requires CAP_SYSLOG.
            once_proc = subprocess.Popen(['dmesg', '--human', '--read-clear'],
                                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            once_stdout, _ = once_proc.communicate()
            if once_stdout:
                self.callback(once_stdout.decode('utf-8').strip())

            try:
                self.process = subprocess.Popen(['journalctl', '-k', '--follow', '-o', 'cat'],
                                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            except FileNotFoundError:
                # journalctl not available either – give up silently
                self.process = None
                return

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
