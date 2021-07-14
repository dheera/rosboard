"use strict";

class Space2DViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {

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

    this.size = 500;
    this.xmin = -10;
    this.xmax = 10;
    this.ymin = -10;
    this.ymax = 10;

    this.canvas = $('<canvas width="' + this.size + '" height="' + this.size + '"></canvas>')
      .css({"width": "100%", "height": "100%", "position": "absolute", "top": 0, "left": 0, "right": 0, "bottom": 0})
      .appendTo(this.wrapper2);

    this.ctx = this.canvas[0].getContext("2d");
  }

  draw(commands) {

    let x2px = (x) => this.size * ((x - this.xmin) / (this.xmax - this.xmin));
    let y2py = (y) => this.size * (1 - (y - this.ymin) / (this.ymax - this.ymin));

    this.ctx.clearRect(0, 0, this.size, this.size);

    this.ctx.fillStyle = "#303035";
    this.ctx.fillRect(0, 0, this.size, this.size);

    // draw grid
    this.ctx.strokeStyle = "#505050";
    this.ctx.beginPath();

    for(let x=this.xmin;x<=this.xmax+0.001;x+=1.0) {
        this.ctx.moveTo(x2px(x),y2py(this.ymin));
        this.ctx.lineTo(x2px(x),y2py(this.ymax));
    }

    for(let y=this.ymin;y<=this.ymax+0.001;y+=1.0) {
        this.ctx.moveTo(x2px(this.xmin),y2py(y));
        this.ctx.lineTo(x2px(this.xmax),y2py(y));
    }

    this.ctx.stroke();


    this.ctx.fillStyle = "#e0e0e0";

    for(let i in commands) {
      let command = commands[i];
      if(command[0] === "line") {
        // draw a line
      } else if(command[0] === "points") {
        for(let i in command[1]) {
          this.ctx.fillRect(x2px(command[1][i][0])-1, y2py(command[1][i][1])-1,3,3);
        }
      }
    }
  }
}

Space2DViewer.supportedTypes = [
];

Space2DViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(Space2DViewer);