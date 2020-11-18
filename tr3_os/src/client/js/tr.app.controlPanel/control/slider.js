if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.slider = function(type) {
  return {
    type: "container",
    size: {
      w: 1,
      h: 40
    },
    border: false,
    children: [{
      type: "container",
      border: false,
      children: [{
        id: "slider-control-" + type,
        type: "slider",
        disabled: (type != "voltage"),
        onDraw: function() {
          var id = tr.controlPanel.state.currentActuator;
          var app = this.getApp();
          var page = app.getCurrentPage();
          var slider = page.getChild("slider-control-" + type).element;
          
          if (this.disabled == true) {
            slider.elt.disabled = true;

            var state = tr.data.getState(id);
            if (type == "position") {
              slider.value(state.position / Math.PI / 2.0);
            } else if (type == "velocity") {
              slider.value(state.velocity);
            } else if (type == "torque") {
              slider.value(state.torque / 60.0);
            } else if (type == "voltage") {
              slider.value(state.effort / 100.0);
            }
          } else {
            slider.elt.disabled = false;
            var sliderVal = slider.value();
            if (type == "voltage" && abs(sliderVal) > 0.1) {
              tr.data.socket.emit("/tr3/" + id + "/control/voltage", sliderVal * 12.6);
            }
          }
        },

        onInput: function(val) {
          var id = tr.controlPanel.state.currentActuator;
          var app = this.getApp();
          var page = app.getCurrentPage();
          var sliderVal = page.getChild("slider-control-" + type).element.value();

          var val = 0;
          if (type == "voltage") {
            val = sliderVal * 12.6;
            tr.data.socket.emit("/tr3/" + id + "/control/voltage", val);
          } else if (type == "position") {
            val = sliderVal * Math.PI * 2.0;
            tr.data.socket.emit("/tr3/" + id + "/control/position", {position: val, duration: 0});
          } else if (type == "velocity") {
            val = sliderVal;
            tr.data.socket.emit("/tr3/" + id + "/control/velocity", val);
          } else if (type == "torque") {
            val = sliderVal * 60;
            tr.data.socket.emit("/tr3/" + id + "/control/torque", val);
          }
        },

        onChange: function(val) {
          var id = tr.controlPanel.state.currentActuator;
          var app = this.getApp();
          var page = app.getCurrentPage();

          var val = 0;
          if (type == "voltage" || type == "velocity") {
            var slider = page.getChild("slider-control-" + type);
            slider.element.value(val);
            tr.data.socket.emit("/tr3/" + id + "/control/" + type, val);
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
