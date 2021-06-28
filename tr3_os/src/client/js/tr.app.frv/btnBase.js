tr.controls.frv.btnBase = function(lbl) {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 0.333,
      h: 50,
    },
    background: "rgb(80, 80, 80)",
    onDraw: function () {
      if (this.joystickPressed) {
        var p = this.getAbsolutePosition();
        var x = -(mouseX - this.startMouseX);
        var y = -(mouseY - this.startMouseY);

        var msg = {
          linear: {
            x: y / 200.0,
            y: 0,
            z: 0
          },
          angular: {
            x: 0,
            y: 0,
            z: x / 100.0
          }
        }

        if (msg.linear.x > 0.500) {
          msg.linear.x = 0.500;
        } else if (msg.linear.x < -0.500) {
          msg.linear.x = -0.500;
        }

        if (msg.angular.z > 1.000) {
          msg.angular.z = 1.000;
        } else if (msg.angular.z < -1.000) {
          msg.angular.z = -1.000;
        }

        console.log(msg.linear.x, msg.angular.z);
        tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
      }
    },
    onMousePress: function() {
      var msg = {
        linear: {
          x: 0,
          y: 0,
          z: 0
        },
        angular: {
          x: 0,
          y: 0,
          z: 0
        }
      }
      if (lbl == "▲") {
        msg.linear.x = 0.500;
      } else if (lbl == "▼") {
        msg.linear.x = -0.500;
      } else if (lbl == "▶") {
        msg.angular.z = -1.000;
      } else if (lbl == "◀") {
        msg.angular.z = 1.000;
      } else if (lbl == "◤") {
        msg.linear.x = 0.500;
        msg.angular.z = 1.000;
      } else if (lbl == "◥") {
        msg.linear.x = 0.500;
        msg.angular.z = -1.000;
      } else if (lbl == "◣") {
        msg.linear.x = -0.500;
        msg.angular.z = 1.000;
      } else if (lbl == "◢") {
        msg.linear.x = -0.500;
        msg.angular.z = -1.000;
      } else if (lbl == "●") {
        this.joystickPressed = true;
        this.startMouseX = mouseX;
        this.startMouseY = mouseY;
      }
      tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
    },
    onMouseRelease: function() {
      this.joystickPressed = false;
      var msg = {
        linear: {
          x: 0,
          y: 0,
          z: 0
        },
        angular: {
          x: 0,
          y: 0,
          z: 0
        }
      }
      tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
    },
    onClick: function() {
      if (lbl == "▲") {

      } else if (lbl == "▼") {

      } else if (lbl == "▶") {

      } else if (lbl == "◀") {

      }
    },
    children: [{
      type: "text",
      textFont: "noto",
      text: lbl,
      align: {
        v: "CENTER",
        h: "CENTER"
      },
      size: {
        w: 1,
        h: 1
      },
    }]
  }
}
