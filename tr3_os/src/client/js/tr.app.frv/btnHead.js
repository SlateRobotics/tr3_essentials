tr.controls.frv.btnHead = function(lbl) {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 0.333,
      h: 50,
    },
    background: "rgb(80, 80, 80)",
    onClick: function () { },
    onMousePress: function() {
      var h0 = tr.data.getState("h0").position;
      var h1 = tr.data.getState("h1").position;

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
        tr.data.socket.emit("/tr3/h1/control/position", {position: h1 - 0.1, duration: 0});
      } else if (lbl == "▼") {
        tr.data.socket.emit("/tr3/h1/control/position", {position: h1 + 0.1, duration: 0});
      } else if (lbl == "▶") {
        msg.angular.z = -0.5;
        tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
      } else if (lbl == "◀") {
        msg.angular.z = 0.5;
        tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
      } else if (lbl == "◤") {
        msg.angular.z = 0.5;
        tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
        tr.data.socket.emit("/tr3/h1/control/position", {position: h1 - 0.1, duration: 0});
      } else if (lbl == "◥") {
        msg.angular.z = -0.5;
        tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
        tr.data.socket.emit("/tr3/h1/control/position", {position: h1 - 0.1, duration: 0});
      } else if (lbl == "◣") {
        msg.angular.z = 0.5;
        tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
        tr.data.socket.emit("/tr3/h1/control/position", {position: h1 + 0.1, duration: 0});
      } else if (lbl == "◢") {
        msg.angular.z = -0.5;
        tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
        tr.data.socket.emit("/tr3/h1/control/position", {position: h1 + 0.1, duration: 0});
      } else if (lbl = "●") {
        tr.data.socket.emit("/tr3/h1/control/position", {position: 0, duration: 0});
      }
    },
    onMouseRelease: function () {
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
