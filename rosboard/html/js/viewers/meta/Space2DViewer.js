"use strict";

// Space2DViewer is an extension of a Viewer that implements the common visualization
// framework for 2D plots, e.g. paths, points in a 2D space.
// Space2DViewer implements drawing functionality, but does not implement any
// message decoding functionality. Child classes that inherit from Space2DViewer
// should decode a message and instruct the plotting framework what to do.

class Space2DViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    // silly css nested div hell to force 100% width
    // but keep 1:1 aspect ratio

    this.wrapper = $('<div></div>')
      .css({
        "position": "relative",
        "width": "100%",
      })
      .appendTo(this.card.content);

    this.wrapper2 = $('<div></div>')
      .css({
        "width": "100%",
        "padding-bottom": "100%",
        "background": "#303035",
        "position": "relative",
        "overflow": "hidden",
      })
      .appendTo(this.wrapper);

    // virtual size of the canvas (not actual pixel size)
    // this is relevant to draw commands inside ctx
    // the actual canvas just scales the drawing
    this.size = 500;

    // bounds of ROI in meters
    this.xmin = -10;
    this.xmax = 10;
    this.ymin = -10;
    this.ymax = 10;

    // canvas that shall be drawn upon
    this.canvas = $('<canvas width="' + this.size + '" height="' + this.size + '"></canvas>')
      .css({
        "width": "100%",
        "height": "100%",
        "position": "absolute",
        "top": 0,
        "left": 0,
        "right": 0, 
        "bottom": 0,
      })
      .appendTo(this.wrapper2);

    // context; draw commands are issued to the context
    this.ctx = this.canvas[0].getContext("2d");

    let that = this;

    this.canvas[0].addEventListener('pointermove', function(e) {
      let x = e.offsetX / that.canvas[0].clientWidth * (that.xmax - that.xmin) + that.xmin;
      let y = (1 - e.offsetY / that.canvas[0].clientHeight) * (that.ymax - that.ymin) + that.ymin;
      that.tip("(" + x.toFixed(3) + ", " + y.toFixed(3) + ")");
    });

    this.canvas[0].addEventListener('mousemove', function(e) {
      if(this.dragging) {
        let deltax = e.clientX - this.lastX;
        let deltay = e.clientY - this.lastY;
        that.pan(-deltax * (that.xmax - that.xmin) / that.size, deltay * (that.ymax - that.ymin) / that.size);
        this.lastX = e.clientX;
        this.lastY = e.clientY;
      }
    });

    this.canvas[0].addEventListener('click', function(e) {
      let x = e.offsetX / that.canvas[0].clientWidth * (that.xmax - that.xmin) + that.xmin;
      let y = (1 - e.offsetY / that.canvas[0].clientHeight) * (that.ymax - that.ymin) + that.ymin;
      that.onSpace2DClick({x: x, y: y});
    });

    this.canvas[0].addEventListener('mousedown', function(e) {
      if(e === null) e = window.event;
      if(!(e.buttons === 1)) return;
      if(e && e.preventDefault) e.preventDefault();
      this.dragging = true;
      this.lastX = e.clientX;
      this.lastY = e.clientY;
    });

    this.canvas[0].addEventListener('mouseup', function(e) {
      if(e === null) e = window.event;
      this.dragging = false;
      if(!(e.buttons === 1)) return;
      if(e && e.preventDefault) e.preventDefault();
    });

    this.canvas[0].addEventListener('mouseout', function(e) {
      if(e === null) e = window.event;
      this.dragging = false;
      if(!(e.buttons === 1)) return;
      if(e && e.preventDefault) e.preventDefault();
    });

    this.canvas[0].addEventListener('mouseleave', function(e) {
      if(e === null) e = window.event;
      this.dragging = false;
      if(!(e.buttons === 1)) return;
      if(e && e.preventDefault) e.preventDefault();
    });

    this.canvas[0].addEventListener('mousewheel', function(e) {
      if(e === null) e = window.event;
      if(e && e.preventDefault) e.preventDefault();
      else e.returnValue=false;
      let delta = 0;
      if(e.wheelDelta) { // IE/Opera
        delta = e.wheelDelta/120;
      } else if (e.detail) { // Mozilla/WebKit
        delta = -e.detail/3;
      }
      if(delta < 0) that.zoom(1.25);
      else if (delta > 0) that.zoom(0.8);
    });

    that.isScaling = false;
    that.scalingStartDist = 0;
    this.canvas[0].addEventListener('touchstart', function(e) {
      if(e.touches.length >= 2) {
        that.isScaling = true;
        that.scalingStartDist = Math.hypot(
          e.touches[0].pageX - e.touches[1].pageX,
          e.touches[0].pageY - e.touches[1].pageY,
        );
        that.panStartX = (e.touches[0].pageX + e.touches[1].pageX)/2;
        that.panStartY = (e.touches[0].pageY + e.touches[1].pageY)/2;
      } else {
        that.isScaling = false;
      }
      that.canvas.css({
        "transition": "transform 0.05s linear",
        "-webkit-transition": "-webkit-transform 0.05s linear",
        "-moz-transition": "-moz-transform 0.05s linear",
        "-ms-transition": "-ms-transform 0.05s linear",
      });
    });

    that.simZoomFactor = 1;
    this.canvas[0].addEventListener('touchmove', function(e) {
      if(!that.isScaling) return;
      let scalingDist = Math.hypot(
        e.touches[0].pageX - e.touches[1].pageX,
        e.touches[0].pageY - e.touches[1].pageY
      );
      that.panDistX = (e.touches[0].pageX + e.touches[1].pageX) / 2 - that.panStartX;
      that.panDistY = (e.touches[0].pageY + e.touches[1].pageY) / 2 - that.panStartY;
      that.simZoomFactor = that.scalingStartDist/scalingDist;
      e.target.style.webkitTransform = "scale(" + (1/that.simZoomFactor) + ") translateX(" + that.panDistX + "px) translateY(" + that.panDistY + "px)";
      e.target.style.mozTransform = "scale(" + (1/that.simZoomFactor) + ") translateX(" + that.panDistX + "px) translateY(" + that.panDistY + "px)";
      e.target.style.msTransform = "scale(" + (1/that.simZoomFactor) + ") translateX(" + that.panDistX + "px) translateY(" + that.panDistY + "px)";
      e.target.style.transform = "scale(" + (1/that.simZoomFactor) + ") translateX(" + that.panDistX + "px) translateY(" + that.panDistY + "px)";
    });

    this.canvas[0].addEventListener('touchend', function(e) {
      that.canvas.css({
        "transition": "",
        "-webkit-transition": "",
        "-moz-transition": "",
        "-ms-transition": "",
      });
      if(that.isScaling && e.touches.length < 2) {
        that.isScaling = false;
        that.pan(
          -that.panDistX * (that.xmax - that.xmin) / that.size / this.clientWidth * that.size,
          that.panDistY * (that.ymax - that.ymin) / that.size / this.clientHeight * that.size,
        );
        that.zoom(that.simZoomFactor);
        that.panDistX = 0;
        that.panDistY = 0;
        that.simZoomFactor = 0;
        e.target.style.transform = "";
      }
    });
  }

  onSpace2DClick({x, y}) {

  }

  pan(deltax, deltay) {
    this.xmin += deltax;
    this.xmax += deltax;
    this.ymin += deltay;
    this.ymax += deltay;
    this.draw(this.drawObjects);
  }

  zoom(factor) {
    if(((this.xmax - this.xmin) > 500 || (this.ymax - this.ymin) > 500) && factor > 1)  return;
    if(((this.xmax - this.xmin) < 0.1 || (this.ymax - this.ymin) < 0.1) && factor < 1)  return;
    let xcenter = (this.xmin + this.xmax) / 2;
    let ycenter = (this.ymin + this.ymax) / 2;
    let xspan = this.xmax - this.xmin;
    let yspan = this.ymax - this.ymin;
    this.xmin = xcenter - xspan/2 * factor;
    this.xmax = xcenter + xspan/2 * factor;
    this.ymin = ycenter - yspan/2 * factor;
    this.ymax = ycenter + yspan/2 * factor;
    this.draw(this.drawObjects);
  }

  setDefaultView({xcenter, ycenter, scale}) {
    this.defaultXCenter = xcenter;
    this.defaultYCenter = ycenter;
    this.defaultScale = scale;

    if(!this.alreadyInitializedDefaultView && this.defaultScale) {
      this.xmin = this.defaultXCenter - this.defaultScale / 2;
      this.xmax = this.defaultXCenter + this.defaultScale / 2;
      this.ymin = this.defaultYCenter - this.defaultScale / 2;
      this.ymax = this.defaultYCenter + this.defaultScale / 2;
      this.alreadyInitializedDefaultView = true;
    }
  }

  draw(drawObjects) {
    // converts x in meters to pixel-wise x based on current bounds
    let x2px = (x) => Math.floor(this.size * ((x - this.xmin) / (this.xmax - this.xmin)));
    // converts y in meters to pixel-wise y based on current bounds
    let y2py = (y) => Math.floor(this.size * (1 - (y - this.ymin) / (this.ymax - this.ymin)));

    // clear the drawing
    this.ctx.clearRect(0, 0, this.size, this.size);
    this.ctx.fillStyle = "#303035";
    this.ctx.fillRect(0, 0, this.size, this.size);

    // draw grid
    if(this.xmax - this.xmin < 50 ) {
      this.ctx.lineWidth = 1;
      this.ctx.strokeStyle = "#404040";
      this.ctx.beginPath();

      for(let x=Math.floor(this.xmin);x<=Math.ceil(this.xmax+0.001);x+=1) {
          this.ctx.moveTo(x2px(x),y2py(this.ymin));
          this.ctx.lineTo(x2px(x),y2py(this.ymax));
      }

      for(let y=Math.floor(this.ymin);y<=Math.ceil(this.ymax+0.001);y+=1) {
          this.ctx.moveTo(x2px(this.xmin),y2py(y));
          this.ctx.lineTo(x2px(this.xmax),y2py(y));
      }

      this.ctx.stroke();
    }

    this.ctx.lineWidth = 1;
    this.ctx.strokeStyle = "#505050";
    this.ctx.beginPath();

    for(let x=Math.floor(this.xmin/5)*5;x<=Math.ceil(this.xmax/5+0.001)*5;x+=5) {
        this.ctx.moveTo(x2px(x),y2py(this.ymin));
        this.ctx.lineTo(x2px(x),y2py(this.ymax));
    }

    for(let y=Math.floor(this.ymin/5)*5;y<=Math.ceil(this.ymax/5+0.001)*5;y+=5) {
        this.ctx.moveTo(x2px(this.xmin),y2py(y));
        this.ctx.lineTo(x2px(this.xmax),y2py(y));
    }
    this.ctx.stroke();

    // draw actual things specified by subclass
    // e.g. lines, points, whatever

    this.ctx.fillStyle = "#e0e0e0";

    for(let i in drawObjects) {
      let drawObject = drawObjects[i];
      if(drawObject.type === "path") {
        this.ctx.lineWidth = drawObject.lineWidth || 1;
        this.ctx.strokeStyle = drawObject.color || "#e0e0e0";
        this.ctx.beginPath();
        let px = x2px(drawObject.data[0]);
        let py = y2py(drawObject.data[1]);
        this.ctx.moveTo(px, py);
        for(let i=1;i<drawObject.data.length/2;i++) {
          let px = x2px(drawObject.data[2*i]);
          let py = y2py(drawObject.data[2*i+1]);
          this.ctx.lineTo(px, py);
        }
        this.ctx.stroke();
      } else if(drawObject.type === "points") {
        this.ctx.fillStyle = drawObject.color || "#e0e0e0";
        for(let i=0; i < drawObject.data.length / 2; i++) {
          if(drawObject.data[2*i] == NaN) continue;
          if(drawObject.data[2*i+1] == NaN) continue;
          let px = x2px(drawObject.data[2*i])-1;
          let py = y2py(drawObject.data[2*i+1])-1;
          if(px < -1) continue;
          if(px > this.size + 1) continue;
          if(py < -1) continue;
          if(py > this.size + 1) continue;
          this.ctx.fillRect(px, py, 3, 3);
        }
      } else if(drawObject.type === "text") {
        this.ctx.fillStyle = drawObject.color || "#e0e0e0";
        this.ctx.font = "12px Jetbrains Mono";
        this.ctx.fillText(drawObject.text, x2px(drawObject.x), y2py(drawObject.y));
      }
    }
    this.drawObjects = drawObjects;
  }
}

Space2DViewer.supportedTypes = [
];

Space2DViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(Space2DViewer);