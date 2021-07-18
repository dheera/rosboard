"use strict";

// Space3DViewer is an extension of a Viewer that implements the common visualization
// framework for 3D stuff.
// Space3DViewer implements drawing functionality, but does not implement any
// message decoding functionality. Child classes that inherit from Space3DViewer
// should decode a message and instruct the plotting framework what to do.

class Space3DViewer extends Viewer {
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
        "height": "0",
        "padding-bottom": "100%",
        "background": "#303035",
        "position": "relative",
        "overflow": "hidden",
      })
      .appendTo(this.wrapper);

    let that = this;

    this.gl = GL.create({ version:1, width: 500, height: 500});
	  this.wrapper2[0].appendChild(this.gl.canvas);
    $(this.gl.canvas).css("width", "100%");
	  this.gl.animate(); // launch loop

		this.cam_pos = [0,100,100];
    this.cam_theta = -1.5707;
    this.cam_phi = 1.0;
    this.cam_r = 50.0;

    this.drawObjectsGl = null;

    //create basic matrices for cameras and transformation
    this.proj = mat4.create();
    this.view = mat4.create();
    this.model = mat4.create();
    this.mvp = mat4.create();
    this.temp = mat4.create();

    this.gl.captureMouse(true);
		this.gl.onmouse = function(e) {
			if(e.dragging)
			{
				// mat4.rotateY(model,model,e.deltax * 0.01);
        that.cam_theta -= e.deltax / 300;
        that.cam_phi -= e.deltay / 300;

        // avoid euler singularities, maybe move to quaternions eventually

        if(that.cam_phi < 0) {
          that.cam_phi = 0.001;
        }
        if(that.cam_phi > Math.PI) {
          that.cam_phi = Math.PI - 0.001;
        }
        that.updatePerspective();
			}
		}

    this.gl.onmousewheel = function(e) {
      that.cam_r -= e.delta;
      if(that.cam_r < 1.0) that.cam_r = 1.0;
      if(that.cam_r > 1000.0) that.cam_r = 1000.0;
      that.updatePerspective();
    }

    this.updatePerspective = () => {
      that.cam_pos[0] = that.cam_r * Math.sin(that.cam_phi) * Math.cos(that.cam_theta);
      that.cam_pos[1] = that.cam_r * Math.sin(that.cam_phi) * Math.sin(that.cam_theta);
      that.cam_pos[2] = that.cam_r * Math.cos(that.cam_phi);

      that.view = mat4.create();
      mat4.perspective(that.proj, 45 * DEG2RAD, that.gl.canvas.width / that.gl.canvas.height, 0.1, 1000);
      mat4.lookAt(that.view, that.cam_pos, [0,0,0], [0,0,1]);
	    mat4.multiply(that.mvp, that.proj, that.view);
    }

    this.updatePerspective();

    this.shader = new Shader('\
      precision highp float;\
      attribute vec3 a_vertex;\
      attribute vec4 a_color;\
      uniform mat4 u_mvp;\
      varying vec4 v_color;\
      void main() {\
          v_color = a_color;\
          gl_Position = u_mvp * vec4(a_vertex,1.0);\
          gl_PointSize = 1.5;\
      }\
      ', '\
      precision highp float;\
      uniform vec4 u_color;\
      varying vec4 v_color;\
      void main() {\
        gl_FragColor = u_color * v_color;\
      }\
    ');

    //generic gl flags and settings
    this.gl.clearColor(0.1,0.1,0.1,1);
    this.gl.disable( this.gl.DEPTH_TEST );

    //rendering loop
    this.gl.ondraw = function() {
      that.gl.clear( that.gl.COLOR_BUFFER_BIT | that.gl.DEPTH_BUFFER_BIT );
      if(!that.drawObjectsGl) return;
      for(let i in that.drawObjectsGl) {
        if(that.drawObjectsGl[i].type === "points") {
          that.shader.uniforms({
            u_color: [1,1,1,1],
            u_mvp: that.mvp
          }).draw(that.drawObjectsGl[i].mesh, gl.POINTS);
        } else if(that.drawObjectsGl[i].type === "lines") {
          that.shader.uniforms({
            u_color: [1,1,1,1],
            u_mvp: that.mvp
          }).draw(that.drawObjectsGl[i].mesh, gl.LINES);
        }
      }
    };
  }

  _getColor(v, vmin, vmax) {
    // http://paulbourke.net/miscellaneous/colourspace/
    let c = [1.0, 1.0, 1.0];

    if (v < vmin)
       v = vmin;
    if (v > vmax)
       v = vmax;
    let dv = vmax - vmin;

    if (v < (vmin + 0.25 * dv)) {
      c[0] = 0;
      c[1] = 4 * (v - vmin) / dv;
    } else if (v < (vmin + 0.5 * dv)) {
      c[0] = 0;
      c[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
    } else if (v < (vmin + 0.75 * dv)) {
      c[0] = 4 * (v - vmin - 0.5 * dv) / dv;
      c[2] = 0;
    } else {
      c[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
      c[2] = 0;
    }

    return(c);
  }


  draw(drawObjects) {
    this.drawObjects = drawObjects;
    let drawObjectsGl = [];
    for(let i in drawObjects) {
      let drawObject = drawObjects[i];
      if(drawObject.type === "points") {
        let colors = new Float32Array(drawObject.data.length / 3 * 4);
        let zmin = drawObject.zmin || -2;
        zmin = -2;
        let zmax = drawObject.zmax || 2;
        zmax = 2;
        let zrange = zmax - zmin;
        console.log(zmin, zmax);
        for(let j=0; j < drawObject.data.length / 3; j++) {
          let c = this._getColor(drawObject.data[3*j+2], zmin, zmax)
          colors[4*j] = c[0];
          colors[4*j+1] = c[1];
          colors[4*j+2] = c[2];
          colors[4*j+3] = 1;
        }
        let points = drawObject.data;
        drawObjectsGl.push({type: "points", mesh: GL.Mesh.load({vertices: points, colors: colors})});
      }
    }
    this.drawObjectsGl = drawObjectsGl;
  }
}

Space3DViewer.supportedTypes = [
];

Space3DViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(Space3DViewer);