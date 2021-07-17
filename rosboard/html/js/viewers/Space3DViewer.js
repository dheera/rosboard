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

    /*this.container = $('<div></div>')
      .css({
        "width": "100%",
        "height": "100%",
      }).appendTo(this.wrapper2);*/

    this.gl = GL.create({ version:1, width: 500, height: 500});
	  this.wrapper2[0].appendChild(this.gl.canvas);
    $(this.gl.canvas).css("width", "100%");
	  this.gl.animate(); // launch loop

    this.meshPoints = GL.Mesh.load({
      vertices: [0,0,0, 0,100,0,  0,0,0, 100,0,0,  0,0,0, 0,0,100], 
      colors: [1,0,0,1, 1,0,0,1,  1,1,1,1, 1,1,1,1,  0,0,1,1, 0,0,1,1 ],
    });

    //create basic matrices for cameras and transformation
    this.proj = mat4.create();
    this.view = mat4.create();
    this.model = mat4.create();
    this.mvp = mat4.create();
    this.temp = mat4.create();

    mat4.perspective(this.proj, 45 * DEG2RAD, gl.canvas.width / gl.canvas.height, 0.1, 1000);
	  mat4.lookAt( this.view, [0,20,20],[0,0,0], [0,1,0]);
	  mat4.multiply(this.mvp, this.proj, this.view);

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
    let that = this;
    this.gl.ondraw = function() {
      that.gl.clear( that.gl.COLOR_BUFFER_BIT | that.gl.DEPTH_BUFFER_BIT );
      if(that.meshPoints) that.shader.uniforms({
          u_color: [1,1,1,1],
          u_mvp: that.mvp
      }).draw(that.meshPoints, gl.POINTS);

    };
    
	  //update loop
    /*
    this.gl.onupdate = function(dt)
    {
      var time = getTime() * 0.001;
      var offset = 2 * Math.PI;
      for( var i = 0; i < objects.length; ++i )
      {
        var matrix = objects[i];
        mat4.identity( matrix );
        var f = i / objects.length;
        mat4.translate( matrix, matrix, [ Math.sin( f * offset + time * 2 ) * 10, Math.sin( f * offset * 1.2 + time ) * 2, Math.cos( f * offset + time ) * 10 ] );
        var s = Math.sin( f + time * 2) * 0.5 + 1.1;
        mat4.scale( matrix, matrix, [ s, s, s] );
      }
    };
    */
  }

  draw(drawObjects) {
    this.drawObjects = drawObjects;
    for(let i in drawObjects) {
      let drawObject = drawObjects[i];
      if(drawObject.type === "points") {
        let colors = [];
        //for(let j=0; j < 100; j++) {
        for(let j=0; j < drawObject.data.length / 3; j++) {
          //if(drawObject.data[2*j] == NaN) continue;
          //if(drawObject.data[2*j+1] == NaN) continue;
          //if(drawObject.data[2*i+2] == NaN) continue;
          colors.push(1);
          colors.push(1);
          colors.push(1);
          colors.push(1);
        }
        let points = drawObject.data;
        //colors = colors.slice(0, 24000);
        this.meshPoints = GL.Mesh.load({vertices: points, colors: colors});
      }
    }
  }
}

Space3DViewer.supportedTypes = [
];

Space3DViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(Space3DViewer);