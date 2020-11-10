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
              slider.value(state.effort / 40.0);
            }
          } else {
            slider.elt.disabled = false;
            var sliderVal = slider.value();
            if (type == "effort" && abs(sliderVal) > 0.1) {
              tr.data.socket.emit("/tr3/joints/" + id + "/control/effort", type);
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
            val = sliderVal;
            tr.data.socket.emit("/tr3/joints/" + id + "/control/voltage", val);
          } else if (type == "position") {
            val = sliderVal * Math.PI * 2.0;
            tr.data.socket.emit("/tr3/joints/" + id + "/control/position", {position: val, duration: 0});
          } else if (type == "velocity") {
            val = sliderVal;
            tr.data.socket.emit("/tr3/joints/" + id + "/control/velocity", val);
          } else if (type == "torque") {
            val = sliderVal;
            tr.data.socket.emit("/tr3/joints/" + id + "/control/torque", val);
          }
        },

        onChange: function(val) {
          var id = tr.controlPanel.state.currentActuator;
          var app = this.getApp();
          var page = app.getCurrentPage();
          if (type == "voltage" || type == "velocity") {
            var val = 0;

            var slider = page.getChild("slider-control-" + type);
            slider.element.value(val);
            tr.data.socket.emit("/tr3/joints/" + id + "/control/" + type, val);
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
