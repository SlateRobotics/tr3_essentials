if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};
var p = tr.controls.pnp2.program_Tools;

tr.controls.pnp2.slider = function(id) {
  return {
    type: "container",
    size: {
      w: 5 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "slider",
        type: "slider",

        onDraw: function() {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var sliderVal = page.getChild(id + "slider").element.value();
        },

        onInput: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var sliderVal = page.getChild(id + "slider").element.value();

          var val = 0;
          val = sliderVal * Math.PI * 2.0;
          //tr.data.socket.emit("/tr3/joints/" + id + "/control/position", val);
          var label = page.getChild(id + "sliderl");
          label.text = val.toFixed(2);
          p.sliderchanged(app.config);
        },

        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
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
