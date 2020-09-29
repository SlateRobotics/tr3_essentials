tr.controls.frv.btnHead = function(lbl) {
  var c = tr.controls.frv;

  return {
    type: "container",
    size: {
      w: 0.333,
      h: 50,
    },
    background: "rgb(80, 80, 80)",
    onClick: function() {
      var h0 = tr.data.getState("h0").position;
      var h1 = tr.data.getState("h1").position;

      if (lbl == "▲") {
        tr.data.socket.emit("/tr3/joints/h1/control/position", {position: h1 + 0.1, duration: 0});
      } else if (lbl == "▼") {
        tr.data.socket.emit("/tr3/joints/h1/control/position", {position: h1 - 0.1, duration: 0});
      } else if (lbl == "▶") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", {position: h0 - 0.1, duration: 0});
      } else if (lbl == "◀") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", {position: h0 + 0.1, duration: 0});
      } else if (lbl == "◤") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", {position: h0 + 0.1, duration: 0});
        tr.data.socket.emit("/tr3/joints/h1/control/position", {position: h1 + 0.1, duration: 0});
      } else if (lbl == "◥") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", {position: h0 - 0.1, duration: 0});
        tr.data.socket.emit("/tr3/joints/h1/control/position", {position: h1 + 0.1, duration: 0});
      } else if (lbl == "◣") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", {position: h0 + 0.1, duration: 0});
        tr.data.socket.emit("/tr3/joints/h1/control/position", {position: h1 - 0.1, duration: 0});
      } else if (lbl == "◢") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", {position: h0 - 0.1, duration: 0});
        tr.data.socket.emit("/tr3/joints/h1/control/position", {position: h1 - 0.1, duration: 0});
      } else if (lbl = "●") {
        tr.data.socket.emit("/tr3/joints/h0/control/position", {position: 0, duration: 0});
        tr.data.socket.emit("/tr3/joints/h1/control/position", {position: 0, duration: 0});
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
