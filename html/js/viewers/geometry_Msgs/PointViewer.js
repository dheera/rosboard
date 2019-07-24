"use strict";

class PointViewer extends RawViewer { }

PointViewer._TYPE = 'geometry_msgs/Point';
PointViewer._SINGLE = true;
addViewer(PointViewer);

