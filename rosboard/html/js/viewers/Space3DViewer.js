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
        "padding-bottom": "100%",
        "background": "#303035",
        "position": "relative",
        "overflow": "hidden",
      })
      .appendTo(this.wrapper);

    this.container = $('<div></div>')
      .css({
        "width": "100%",
        "height": "100%",
      }).appendTo(this.wrapper2);

    this.gl = GL.create({ version:1, width: this.container.offsetWidth, height: this.container.offsetHeight});
	  this.container[0].appendChild(this.gl.canvas);
	  this.gl.animate(); // launch loop

    this.objects = [];
	  this.mesh = GL.Mesh.point();
    let matrix = mat4.create();
          mat4.identity(matrix);
          mat4.translate(matrix, matrix, [
            0.0,
            0.0,
            0.0,
          ]);
          mat4.scale(matrix, matrix, [1.0, 1.0, 1.0]);
          this.objects.push(matrix);

    //create basic matrices for cameras and transformation
    this.proj = mat4.create();
    this.view = mat4.create();
    this.model = mat4.create();
    this.vp = mat4.create();
    this.temp = mat4.create();

    mat4.perspective(this.proj, 45 * DEG2RAD, gl.canvas.width / gl.canvas.height, 0.1, 1000);
	  mat4.lookAt( this.view, [0,20,20],[0,0,0], [0,1,0]);
	  mat4.multiply(this.vp, this.proj, this.view);

    this.shader = new Shader('\
      precision highp float;\
      attribute vec3 a_vertex;\
      attribute vec3 a_normal;\
      varying vec3 v_normal;\
      attribute mat4 u_model;\
      uniform mat4 u_viewprojection;\
      void main() {\
        vec3 pos = (u_model * vec4(a_vertex,1.0)).xyz;\
        v_normal = (u_model * vec4(a_normal,0.0)).xyz;\
        gl_Position = u_viewprojection * vec4(pos,1.0);\
      }\
      ', '\
      precision highp float;\
      varying vec3 v_normal;\
      uniform vec3 u_lightvector;\
      uniform vec4 u_color;\
      void main() {\
        vec3 N = normalize(v_normal);\
        gl_FragColor = u_color * max(0.0, dot(u_lightvector,N));\
      }\
    ');

    //generic gl flags and settings
    this.gl.clearColor(0.1,0.1,0.1,1);
    this.gl.enable( this.gl.DEPTH_TEST );

    this.uniforms = {
      u_color: [1,1,1,1],
      u_lightvector: vec3.normalize(vec3.create(),[1,1,0.5]),
      u_model: this.model,
      u_viewprojection: this.vp,
    };

    //rendering loop
    let that = this;
    this.gl.ondraw = function() {
      that.gl.clear( that.gl.COLOR_BUFFER_BIT | that.gl.DEPTH_BUFFER_BIT );
      if(!that.uniforms || !that.mesh || !that.objects) return;
      that.shader.uniforms(that.uniforms);
      that.shader.drawInstanced(that.mesh, GL.TRIANGLES, "triangles", { u_model: that.objects } );
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
        this.objects = [];

        for(let j=0; j < 10000; j++) {
        // for(let j=0; i < drawObject.data.length / 2; j++) {
          if(drawObject.data[2*j] == NaN) continue;
          if(drawObject.data[2*j+1] == NaN) continue;
          // if(drawObject.data[2*i+2] == NaN) continue;
          let matrix = mat4.create();
          mat4.identity(matrix);
          mat4.translate(matrix, matrix, [
            drawObject.data[2*j],
            drawObject.data[2*j+1],
            0.0,
          ]);
          mat4.scale(matrix, matrix, [0.1, 0.1, 0.1]);
          this.objects.push(matrix);
        }
      }
    }
  }
}

Space3DViewer.supportedTypes = [
];

Space3DViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(Space3DViewer);