"use strict";

class TwistViewer extends RawViewer { }

TwistViewer._TYPE = 'geometry_msgs/Twist';
TwistViewer._SINGLE = true;
addViewer(TwistViewer);

