tr.gui.datePicker = {
  defaults: function() {
    this.dMonth = "00";
    this.dDay = "00";
    this.dYear = "0000";

    this.changed = function() {
      if (!this.initialized) return;
      if (this.onChange) {
        var d = moment(this.dYear + "-" + this.dMonth + "-" + this.dDay);
        this.onChange(d);
      }
    }
  },

  setup: function() {
    var days = [];
    for (var i = 1; i <= 31; i++) {
      if (i < 10) {
        days.push("0" + String(i));
      } else {
        days.push(String(i));
      }
    }

    var years = [(moment().year())]
    years.push(years[0] + 1);

    var container = new tr.gui.component(tr.gui.container);
    container.setup({
      border: false,
      parent: this,
      size: {
        w: 1,
        h: 40
      },
      padding: 5,
      children: [{
        type: "select",
        size: {
          w: 0.25,
          h: 1
        },
        defaultValue: moment().format("MM"),
        options: ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12"],
        textSize: 22,
        onChange: function(val) {
          this.parent.parent.dMonth = val;
          this.parent.parent.changed();
        },
      }, {
        type: "text",
        text: "/",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        size: {
          w: 0.1,
          h: 1
        },
        textSize: 22,
      }, {
        type: "select",
        size: {
          w: 0.25,
          h: 1
        },
        defaultValue: moment().format("DD"),
        options: days,
        textSize: 22,
        onChange: function(val) {
          this.parent.parent.dDay = val;
          this.parent.parent.changed();
        }
      }, {
        type: "text",
        text: "/",
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        size: {
          w: 0.1,
          h: 1
        },
        textSize: 22,
      }, {
        type: "select",
        size: {
          w: 0.3,
          h: 1
        },
        defaultValue: moment().format("YYYY"),
        options: years,
        textSize: 22,
        onChange: function(val) {
          this.parent.parent.dYear = val;
          this.parent.parent.changed();
        }
      }]
    });
    this.children.push(container);
  }
}
