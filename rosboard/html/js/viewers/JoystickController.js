"use strict";

class JoystickController extends Viewer {
  // List of currently active joysticks.
  // TODO: Make this dynamic?
  joysticks = [false, false, false, false];

  // Install a virtual joystick
  virtualJoystick() {
    var options = {
      zone: document.getElementById(this.joyId),
      mode: 'semi',
      color: 'blue',
      size: 150,
      catchDistance: 300,
    };
    var manager = nipplejs.create(options);
    manager.on('start', function (evt, data) {
      let joystickX = 0.0;
      let joystickY = 0.0;
      currentTransport.update_joy({ joystickX, joystickY });
    }).on('end', function (evt, data) {
      let joystickX = 0.0;
      let joystickY = 0.0;
      currentTransport.update_joy({ joystickX, joystickY });
    }).on('move', function (evt, data) {
      let radian = data['angle']['radian'];
      let distance = data['distance'];
      let joystickX = Math.max(Math.min(Math.cos(radian) / 75 * distance, 1), -1);
      let joystickY = -1 * Math.max(Math.min(Math.sin(radian) / 75 * distance, 1), -1);
      currentTransport.update_joy({ joystickX, joystickY });
    });
  }

  // Set up a polling interval for real joysticks
  // TODO: Only do this when there are active joysticks?
  realJoysticks() {
    // 10hz, maybe make this faster?
    setInterval(() => {
      for (var i = 0; i < this.joysticks.length; i++) {
        if (this.joysticks[i]) {
          this.readJoystick(i);
        }
      }
    }, 100);
  }

  // Read a specific joystick.
  readJoystick(ix) {
    var gamepad = navigator.getGamepads()[ix];
    var joystickX = gamepad.axes[0];
    var joystickY = gamepad.axes[1];
    currentTransport.update_joy({joystickX, joystickY});
    for (var i = 0; i < gamepad.buttons.length; i++) {
      if (gamepad.buttons[i].value > 0) {
        currentTransport.update_button(i, 1);
      } else {
        currentTransport.update_button(i, 0);
      }
    }
  }

  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewer = $('<div></div>')
      .css({
        'font-size': '11pt'
        , "filter": "invert(100%) saturate(50%)"
      })
      .appendTo(this.card.content);

    this.joyId = "joy-" + Math.floor(Math.random() * 10000);
    this.joy = $('<div id="' + this.joyId + '"></div>')
      .css({
        "height": "250px",
      })
      .appendTo(this.viewer);

    var buttonCSS = {
      "height": "30px",
      "width": "30px",
      "padding": "5px",
      "margin": "5px"
    };
    for (var ix = 0; ix < 10; ix++) {
      $(`<button id="${ix}" >${ix}</button>`)
        .css(buttonCSS)
        .mousedown(ix, (event) => {
          currentTransport.update_button(event.data, 1);
        })
        .mouseup(ix, (event) => {
          currentTransport.update_button(event.data, 0);
        })
        .appendTo(this.viewer);
    }

    this.virtualJoystick();
    this.realJoysticks();

    window.addEventListener("gamepadconnected", (e) => {
      if (e.gamepad.index < this.joysticks.length) {
        this.joysticks[e.gamepad.index] = true;
      }
    });

    window.addEventListener("gamepaddisconnected", (e) => {
      if (e.gamepad.index < this.joysticks.length) {
        this.joysticks[e.gamepad.index] = false;
      }
    });

  }

  onData(msg) {
    this.card.title.text(msg._topic_name);
  }
}

JoystickController.friendlyName = "JoystickController";

JoystickController.supportedTypes = [
  "geometry_msgs/msg/Twist",
  "sensor_msgs/msg/Joy"
];

JoystickController.maxUpdateRate = 10.0;

Viewer.registerViewer(JoystickController);
