"use strict";

class DepthImageViewer extends Viewer {
  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({'font-size': '11pt', 'position': 'relative'})
      .appendTo(this.card.content);

    this._colormapSelect = $('<select></select>')
      .css({
        'margin-bottom': '4px',
        'font-size': '10pt',
        'padding': '2px',
        'max-width': '100%',
      })
      .appendTo(this.viewerNode);

    let that = this;
    Object.keys(DepthImageViewer._COLORMAPS).forEach(function(name) {
      $('<option></option>').val(name).text(name).appendTo(that._colormapSelect);
    });

    this._colormapSelect.val('rainbow');
    this._colormapSelect.on('change', function() {
      that._activeLUT = DepthImageViewer._getLUT(this.value);
      if(that.depthData) that._redraw();
    });

    this._activeLUT = DepthImageViewer._getLUT('rainbow');

    this.canvas = $('<canvas></canvas>')
      .css({"width": "100%"})
      .appendTo(this.viewerNode);

    this.canvas[0].addEventListener('pointermove', function(e) {
      if(!that.depthData) return;
      let rect = that.canvas[0].getBoundingClientRect();
      if(!rect.width || !rect.height) return;

      let x = Math.floor((e.clientX - rect.left) / rect.width * that.depthData.shape[1]);
      let y = Math.floor((e.clientY - rect.top) / rect.height * that.depthData.shape[0]);
      x = Math.min(Math.max(x, 0), that.depthData.shape[1] - 1);
      y = Math.min(Math.max(y, 0), that.depthData.shape[0] - 1);

      let depth = that.depthData.values[y * that.depthData.shape[1] + x];
      that.tip("(" + x.toFixed(0) + ", " + y.toFixed(0) + ")  depth: " + depth.toFixed(3));
    });

    this.canvas[0].addEventListener('pointerdown', function(e) {
      if(!that.depthData) return;
      let rect = that.canvas[0].getBoundingClientRect();
      if(!rect.width || !rect.height) return;

      let x = Math.floor((e.clientX - rect.left) / rect.width * that.depthData.shape[1]);
      let y = Math.floor((e.clientY - rect.top) / rect.height * that.depthData.shape[0]);
      x = Math.min(Math.max(x, 0), that.depthData.shape[1] - 1);
      y = Math.min(Math.max(y, 0), that.depthData.shape[0] - 1);

      let depth = that.depthData.values[y * that.depthData.shape[1] + x];
      console.log("Depth at (" + x + ", " + y + "): " + depth);
    });

    this.depthData = null;
    this.lastMsg = null;

    super.onCreate();
  }

  onData(msg) {
    this.card.title.text(msg._topic_name);

    if(msg._data_depth) {
      this.renderDepth(msg);
    } else if(msg._data_jpeg) {
      this.error("Not a depth image (encoding: " + (msg.encoding || "unknown") + "). Use Image viewer instead.");
    }
  }

  _redraw() {
    if(!this.depthData) return;
    let width = this.depthData.shape[1];
    let height = this.depthData.shape[0];
    let values = this.depthData.values;
    let rangeMin = this.depthData.range[0];
    let rangeSpan = this.depthData.rangeSpan;

    let ctx = this.canvas[0].getContext('2d');
    let imageData = ctx.createImageData(width, height);
    let pixels = imageData.data;
    let lut = this._activeLUT;
    let lutSize = DepthImageViewer._LUT_SIZE;

    for(let i = 0; i < width * height; i++) {
      let val = values[i];
      if(!isFinite(val) || val <= 0) {
        pixels[i * 4] = 0;
        pixels[i * 4 + 1] = 0;
        pixels[i * 4 + 2] = 0;
        pixels[i * 4 + 3] = 255;
        continue;
      }

      let t = Math.floor((val - rangeMin) / rangeSpan * (lutSize - 1));
      t = Math.max(0, Math.min(lutSize - 1, t));
      let k = t * 3;
      pixels[i * 4] = lut[k];
      pixels[i * 4 + 1] = lut[k + 1];
      pixels[i * 4 + 2] = lut[k + 2];
      pixels[i * 4 + 3] = 255;
    }

    ctx.putImageData(imageData, 0, 0);
  }

  renderDepth(msg) {
    let info = msg._data_depth;
    let binaryStr = atob(info.data);
    let bytes = new Uint8Array(binaryStr.length);
    for(let i = 0; i < binaryStr.length; i++) {
      bytes[i] = binaryStr.charCodeAt(i);
    }

    let values;
    let dtype = info.dtype;
    if(dtype === "uint16" || dtype === "<u2") {
      values = new Uint16Array(bytes.buffer);
    } else if(dtype === "float32" || dtype === "<f4") {
      values = new Float32Array(bytes.buffer);
    } else if(dtype === "float64" || dtype === "<f8") {
      values = new Float64Array(bytes.buffer);
    } else if(dtype === "int16" || dtype === "<i2") {
      values = new Int16Array(bytes.buffer);
    } else if(dtype === "int32" || dtype === "<i4") {
      values = new Int32Array(bytes.buffer);
    } else if(dtype === "uint8" || dtype === "|u1") {
      values = bytes;
    } else if(dtype === "int8" || dtype === "|i1") {
      values = new Int8Array(bytes.buffer);
    } else {
      this.error("Unsupported depth dtype: " + dtype);
      return;
    }

    let width = info.shape[1];
    let height = info.shape[0];
    let rangeMin = info.range[0];
    let rangeMax = info.range[1];
    let rangeSpan = rangeMax - rangeMin;
    if(rangeSpan <= 0) rangeSpan = 1.0;

    this.depthData = {
      values: values,
      shape: [height, width],
      range: info.range,
      rangeSpan: rangeSpan,
    };

    if(this.canvas[0].width !== width || this.canvas[0].height !== height) {
      this.canvas[0].width = width;
      this.canvas[0].height = height;
    }

    this._redraw();
  }
}

DepthImageViewer._COLORMAPS = {
  'rainbow':    [[0.0,0x0000FF],[0.2,0x00FFFF],[0.5,0x00FF00],[0.8,0xFFFF00],[1.0,0xFF0000]],
  'cooltowarm': [[0.0,0x3C4EC2],[0.2,0x9BBCFF],[0.5,0xDCDCDC],[0.8,0xF6A385],[1.0,0xB40426]],
  'blackbody':  [[0.0,0x000000],[0.2,0x780000],[0.5,0xE63200],[0.8,0xFFFF00],[1.0,0xFFFFFF]],
  'grayscale':  [[0.0,0x000000],[0.2,0x404040],[0.5,0x7F7F80],[0.8,0xBFBFBF],[1.0,0xFFFFFF]],
};

DepthImageViewer._LUT_SIZE = 256;

DepthImageViewer._buildLUT = function(keypoints) {
  let lut = new Uint8Array(DepthImageViewer._LUT_SIZE * 3);
  let step = 1.0 / DepthImageViewer._LUT_SIZE;
  let k = 0;
  for(let t = 0; t <= 1.0 - step/2; t += step) {
    for(let j = 0; j < keypoints.length - 1; j++) {
      if(t >= keypoints[j][0] && t < keypoints[j+1][0]) {
        let tmin = keypoints[j][0], tmax = keypoints[j+1][0];
        let cmin = keypoints[j][1], cmax = keypoints[j+1][1];
        let f = (t - tmin) / (tmax - tmin);
        lut[k]     = (((cmin >> 16) & 0xFF) * (1-f) + ((cmax >> 16) & 0xFF) * f) | 0;
        lut[k + 1] = (((cmin >> 8) & 0xFF) * (1-f) + ((cmax >> 8) & 0xFF) * f) | 0;
        lut[k + 2] = ((cmin & 0xFF) * (1-f) + (cmax & 0xFF) * f) | 0;
        k += 3;
        break;
      }
    }
  }
  return lut;
};

DepthImageViewer._getLUT = function(name) {
  if(!this._lutCache) this._lutCache = {};
  if(!this._lutCache[name]) {
    this._lutCache[name] = this._buildLUT(this._COLORMAPS[name]);
  }
  return this._lutCache[name];
};

DepthImageViewer.friendlyName = "Depth Image";

DepthImageViewer.supportedTypes = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
];

DepthImageViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(DepthImageViewer);
