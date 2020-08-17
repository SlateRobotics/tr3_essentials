if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.inverseBtn_Display = function(disp) {

  if (disp == 'x') {
    return {
      type: "container",
      size: {
        w: 1 / 5,
        h: 35
      },
      children: [{
        type: "container",
        border: false,
        onClick: function() {
          //Do waypoint Add
          //tr.data.socket.emit(rostopic, value);
        },
        children: [{
          id: "inverse_X",
          type: "text",
          text: "X",
          textSize: 12,
          textFont: "noto",
          align: {
            v: "CENTER",
            h: "CENTER"
          },
        }],
      }]
    }
  }

  if (disp == 'y') {
    return {
      type: "container",
      size: {
        w: 1 / 5,
        h: 35
      },
      children: [{
        type: "container",
        border: false,
        onClick: function() {
          //Do waypoint Add
          //tr.data.socket.emit(rostopic, value);
        },
        children: [{
          id: "inverse_Y",
          type: "text",
          text: "Y",
          textSize: 12,
          textFont: "noto",
          align: {
            v: "CENTER",
            h: "CENTER"
          },
        }],
      }]
    }
  }

  if (disp == 'z') {
    return {
      type: "container",
      size: {
        w: 1 / 5,
        h: 35
      },
      children: [{
        type: "container",
        border: false,
        onClick: function() {
          //Do waypoint Add
          //tr.data.socket.emit(rostopic, value);
        },
        children: [{
          id: "inverse_Z",
          type: "text",
          text: "X",
          textSize: 12,
          textFont: "noto",
          align: {
            v: "CENTER",
            h: "CENTER"
          },
        }],
      }]
    }
  }

  if (disp == 'h') {
    return {
      type: "container",
      size: {
        w: 1 / 5,
        h: 35
      },
      children: [{
        type: "container",
        border: false,
        onClick: function() {
          //Do waypoint Add
          //tr.data.socket.emit(rostopic, value);
        },
        children: [{
          id: "inverse_H",
          type: "text",
          text: "H",
          textSize: 12,
          textFont: "noto",
          align: {
            v: "CENTER",
            h: "CENTER"
          },
        }],
      }]
    }
  }
  if (disp == 'd') {
    return {
      type: "container",
      size: {
        w: 1 / 5,
        h: 35
      },
      children: [{
        type: "container",
        border: false,
        onClick: function() {
          //Do waypoint Add
          //tr.data.socket.emit(rostopic, value);
        },
        children: [{
          id: "inverse_Duration",
          type: "text",
          text: "D",
          textSize: 12,
          textFont: "noto",
          align: {
            v: "CENTER",
            h: "CENTER"
          },
        }],
      }]
    }
  }
}
