#!/usr/bin/env python3

import re
import select
import subprocess
import time
import threading
import traceback

class ProcessesSubscriber(object):
    def __init__(self, callback):
        self.callback = callback
        self.stop_signal = None
        threading.Thread(target = self.start, daemon = True).start()

    def __del__(self):
        self.stop_signal = True

    def unregister(self):
        self.stop_signal = True

    def start(self):
        re_field = re.compile("(^ *PID|USER +| *%CPU| *%MEM|COMMAND.*)")

        self.stop_signal = None
        try:
            while not self.stop_signal:
                lines = subprocess.check_output(['top', '-bn', '1']).decode('utf-8').split("\n")
                
                fields = None
                output = []

                for line in lines:
                    if len(line.strip()) == 0:
                        continue

                    if "PID" in line and "%CPU" in line and "%MEM" in line and "COMMAND" in line and "USER" in line:
                        fields = {}
                        for m in re_field.finditer(line):
                            fields[m.group().strip()] = (m.start(), m.end())
                        continue

                    if fields is None:
                        continue
                    output.append({
                        "pid": int(line[fields["PID"][0] : fields["PID"][1]]),
                        "user": line[fields["USER"][0] : fields["USER"][1]].strip(),
                        "cpu": float(line[fields["%CPU"][0] : fields["%CPU"][1]].replace(',','.')),
                        "mem": float(line[fields["%MEM"][0] : fields["%MEM"][1]].replace(',','.')),
                        "command": line[fields["COMMAND"][0] : ].strip(),
                    })

                self.callback(output)
                time.sleep(2)
        except:
            traceback.print_exc()

if __name__ == "__main__":
    # Run test
    ProcessesSubscriber(lambda msg: print("Received msg: %s" % msg))
    time.sleep(100)
