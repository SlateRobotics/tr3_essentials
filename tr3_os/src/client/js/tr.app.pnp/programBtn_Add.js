if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.programBtn_Add = function() {
  return {
    type: "container",
    size: {
      w: 1 / 4,
      h: 50
    },
    children: [{
      type: "container",
      border: false,
      onClick: function() {
        //Do program Add
          //tr.data.socket.emit(rostopic, value);
      },
      children: [{
        type: "text",
        text: "+", // Add symbol
        textSize: 24,
        textFont: "noto",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
