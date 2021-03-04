tr.controls.frv.btnArm = function(lbl) {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 0.333,
      h: 50,
    },
    background: "rgb(80, 80, 80)",
    onClick: function() {
      var msg = tr.data.arm.pose;

      // abort if no pose published
      if (!msg.position) {
        return;
      }

      msg.orientation = {
        x: 0,
        y: 0,
        z: 0,
        w: 1
      };

      var incr = 0.1;

      if (lbl == "▲") {
        msg.position.x += incr;
      } else if (lbl == "▼") {
        msg.position.x -= incr;
      } else if (lbl == "▶") {
        msg.position.y -= incr;
      } else if (lbl == "◀") {
        msg.position.y += incr;
      } else if (lbl == "◤") {

      } else if (lbl == "◥") {
        
      } else if (lbl == "◣") {

      } else if (lbl == "◢") {

      } else if (lbl == "●") {

      } else if (lbl == "Z+") {
        msg.position.z += incr;
      } else if (lbl == "Z-") {
        msg.position.z -= incr;
      } else if (lbl == "G+") {
        tr.data.socket.emit("/tr3/g0/control/position", {position: 1.0, duration: 3000 });
        return;
      } else if (lbl == "G-") {
        tr.data.socket.emit("/tr3/g0/control/position", {position: 0.0, duration: 3000 });
        return;
      } else {
        return;
      }

      console.log(msg)
      tr.data.socket.emit("/tr3/arm/pose/set", msg);
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
