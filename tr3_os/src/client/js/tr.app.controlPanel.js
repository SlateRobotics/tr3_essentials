function label(id) {
  return {
    type: "container",
    size: {
      w: 0.111,
      h: 25
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: id,
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}

function label_state(id) {
  return {
    type: "container",
    size: {
      w: 0.111,
      h: 25
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: "",
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        onDraw: function() {
          var p = tr.data.getState(id).position;
          if (p) {
            this.text = p.toFixed(2);
          }
        }
      }],
    }]
  }
}

function slider_label(id) {
  return {
    type: "container",
    size: {
      w: 0.111,
      h: 25
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "sliderl",
        type: "text",
        text: "0",
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        onDraw: function() {}
      }],
    }]
  }
}

function button_big(text, rostopic, value, background) {
  return {
    type: "container",
    size: {
      w: 0.5,
      h: 50
    },
    background: background,
    onClick: function() {
      tr.data.socket.emit(rostopic, value);
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: text,
        textSize: 18,
        padding: 12,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}

function map_range(val, low1, high1, low2, high2) {
  return low2 + (high2 - low2) * (val - low1) / (high1 - low1);
}

function pos_Slider(id) {
  return {
    type: "container",
    size: {
      w: .334,
      h: 25
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "slider",
        type: "slider",

        onInput: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);
          var sliderVal = page.getChild(id + "slider").element.value();

          var val = 0;
          if (select.element.value() == "EFFORT") {
            val = sliderVal * 100.0;
            tr.data.socket.emit("/tr3/joints/" + id + "/control/effort", val);
          } else if (select.element.value() == "SERVO") {
            val = sliderVal * Math.PI * 2.0;
            tr.data.socket.emit("/tr3/joints/" + id + "/control/position", val);
          }

          var label = page.getChild(id + "sliderl");
          label.text = val.toFixed(2);
        },

        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);
          if (select.element.value() == "EFFORT") {
            var val = 0;

            var slider = page.getChild(id + "slider");
            slider.element.value(val);

            var label = page.getChild(id + "sliderl");
            label.text = val.toFixed(2);
          }
        },

        min: -1,
        max: 1,
        val: 0,
        step: 0.01,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}

function templabel(text, w, h) {
  var ww = 0;
  var hh = 0;
  if (w == 0) {
    ww = 1;
  } else {
    ww = w;
  }
  if (h == 0) {
    hh = 25;
  } else {
    hh = h;
  }
  return {
    type: "container",
    size: {
      w: ww,
      h: hh
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        type: "text",
        text: text,
        textSize: 18,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}

function select_mode(id) {
  return {
    type: "container",
    size: {
      w: 0.333,
      h: 25
    },

    children: [{
      type: "container",
      border: false,
      children: [{
        type: "select",
        id: "select-" + id,
        options: ["EFFORT", "BACKDRIVE", "SERVO"],
        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          if (page) {
            var slider = page.getChild(id + "slider");

            if (val == "EFFORT") {
              slider.setval(0);
            } else if (val == "BACKDRIVE") {
              slider.setval(0);
            } else if (val == "SERVO") {
              var state = tr.data.getState(id).position;
              console.log(state)
              slider.setval(state / Math.PI / 2.0);
            }

          }


          var i = 0;
          if (val == "EFFORT") {
            i = 0;
          } else if (val == "BACKDRIVE") {
            i = 1;
          } else if (val == "SERVO") {
            i = 2;
          }

          tr.data.socket.emit("/tr3/joints/" + id + "/mode", i);
        },
        Size: {
          w: 1,
          h: 25
        },
        textSize: 12,
        padding: 0,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}


app.drawing = new App({
  name: "Control Panel",
  iconUrl: "/img/icon-app-control",
  pages: [{
    pos: {
      x: 0,
      y: 0
    },
    size: {
      w: 1.0,
      h: 1.0
    },
    header: {
      text: "Control Panel V2",
    },
    children: [{
      type: "tabControl",
      labels: ["Control", "Config", "3D Render"],
      pages: [{
        type: "container",
        size: {
          w: 1.0,
          h: "fill"
        },
        padding: 10,
        background: "rgba(255, 255, 255, 0.2)",
        children: [
          label("IDs"), label("Position"), templabel("Mode Select", .333, 25), label("Target"), templabel("Position Slider", .334, 25),
          label("a0"), label_state("a0"), select_mode("a0"), slider_label("a0"), pos_Slider("a0"),
          label("a1"), label_state("a1"), select_mode("a1"), slider_label("a1"), pos_Slider("a1"),
          label("a2"), label_state("a2"), select_mode("a2"), slider_label("a2"), pos_Slider("a2"),
          label("a3"), label_state("a3"), select_mode("a3"), slider_label("a3"), pos_Slider("a3"),
          label("a4"), label_state("a4"), select_mode("a4"), slider_label("a4"), pos_Slider("a4"),
          label("h0"), label_state("h0"), select_mode("h0"), slider_label("h0"), pos_Slider("h0"),
          label("h1"), label_state("h1"), select_mode("h1"), slider_label("h1"), pos_Slider("h1"),
          {
            type: "container",
            size: {
              w: 1.0,
              h: 10
            },
            border: false
          },
          button_big("STOP", "/tr3/stop", true, "rgba(212, 40, 40, 1)"), button_big("RELEASE", "/tr3/stop", false, "rgba(43, 212, 40, 1)"),
          {
            type: "container",
            size: {
              w: 1.0,
              h: 10
            },
            border: false
          },
          button_big("SHUTDOWN", "/tr3/shutdown", true, "rgba(212, 40, 40, 1)"), button_big("POWER UP", "/tr3/powerup", true, "rgba(43, 212, 40, 1)")
        ],
      }, {
        type: "container",
        size: {
          w: 1.0,
          h: "fill"
        },
        padding: 10,
        background: "rgba(255, 255, 255, 0.2)",
      }, {
        type: "container",
        size: {
          w: 1.0,
          h: "fill"
        },
        padding: 10,
        background: "rgba(255, 255, 255, 0.2)",
        children: [{
          type: "tr2",
        }],
      }],
    }],
  }],
});
