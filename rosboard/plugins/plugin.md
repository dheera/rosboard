Example of a possible plugin with ros support:

```python
#!/usr/bin/env python3

import os
import pathlib
import sys

sys.path.append(f"{pathlib.Path(__file__).parent.resolve()}/.")
if os.environ.get("ROS_VERSION") == "1":
    import rospy  # ROS1
elif os.environ.get("ROS_VERSION") == "2":
    import rospy2 as rospy  # ROS2
else:
    print("ROS not detected. Please source your ROS environment\n(e.g. 'source /opt/ros/DISTRO/setup.bash')")
    exit(1)

from std_msgs.msg import String


class DummyPlugin:

    def __init__(self):
        self.publisher = rospy.Publisher('dummy_topic', String)

    def receive(self, message):
        msg = String()
        msg.data = str(message)
        self.publisher.publish(msg)


# This is the most important function for all plugins,
# it's called when a message from the client is sent towards the backend
def receive(message):
    global _instance
    if _instance == None:
        _instance = DummyPlugin()

    _instance.receive(message)

_instance = None

# Add all javascript files which are needed for this plugin on the client side:
# e.g. "js/plugins/dummy.js", "js/viewer/dummy.js"
# From client side use the javascript function, like this:
# currentTransport.send_plugin_message("dummy", "myImportantMessage"})
# currentTransport.send_plugin_message("dummy", {myKey: "myImportantMessage"})
js_files = []
```