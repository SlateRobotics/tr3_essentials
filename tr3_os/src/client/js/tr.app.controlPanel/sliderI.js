if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.sliderI = function(id) {
  return {
    type: "container",
    size: {
      w: 1 / 9,
      h: 20
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: id + "slider-i",
        type: "slider",

        onInput: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);
          var sliderVal = page.getChild(id + "slider-i").element.value();

          var label = page.getChild("txt-" + id + "-i");
          label.text = val.toFixed(1);
        },

        onChange: function(val) {
          var app = this.getApp();
          var page = app.getCurrentPage();
          var select = page.getChild("select-" + id);

          var slider = page.getChild(id + "slider-i");
          var val = slider.element.value();

          var label = page.getChild("txt-" + id + "-i");
          label.text = val.toFixed(1);
        },

        min: 0,
        max: 20,
        val: 5,
        step: 0.1,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
