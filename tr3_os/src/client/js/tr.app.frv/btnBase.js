tr.controls.frv.btnBase = function(lbl) {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 0.333,
      h: 50,
    },
    background: "rgb(80, 80, 80)",
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
        msg.linear.x = 1.0;
      } else if (lbl == "▼") {
        msg.linear.x = -1.0;
      } else if (lbl == "▶") {
        msg.angular.z = -1.7;
      } else if (lbl == "◀") {
        msg.angular.z = 1.7;
      } else if (lbl == "◤") {
        msg.linear.x = 1.0;
        msg.angular.z = 1.7;
      } else if (lbl == "◥") {
        msg.linear.x = 1.0;
        msg.angular.z = -1.7;
      } else if (lbl == "◣") {
        msg.linear.x = -1.0;
        msg.angular.z = 1.7;
      } else if (lbl == "◢") {
        msg.linear.x = -1.0;
        msg.angular.z = -1.7;
      }
      tr.data.socket.emit("/tr3/base/diff/cmd_vel", msg);
    },
    onMouseRelease: function() {
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
