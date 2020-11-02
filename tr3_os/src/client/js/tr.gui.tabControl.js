tr.gui.tabControl = {
  defaults: function() {
    this.labels = this.config.labels || [];
    this.pages = this.config.pages || [];
    this.config.currentPage = this.config.currentPage || 0;
  },

  setup: function() {
    var buttons = this.componentConfig.createButtons.bind(this)();
    var page = this.config.pages[this.config.currentPage];
    var container = new tr.gui.component(tr.gui.container);
    container.setup({
      type: "container",
      border: false,
      parent: this,
      margin: this.margin,
      padding: this.padding,
      children: [buttons, page]
    });
    this.children.push(container);
  },

  draw: function() {
    for (var i = 0; i < this.children[0].children[0].children.length; i++) {
      var b = this.children[0].children[0].children[i];
      if (i == this.config.currentPage) {
        b.background = "rgb(50, 50, 50)";
      } else {
        b.background = "rgb(100, 100, 100)";
      }
    }
  },

  createButtons: function() {
    var c = [];
    var w = 1 / this.config.pages.length;
    for (var i = 0; i < this.config.pages.length; i++) {
      var b = this.componentConfig.createButton(this.config.labels[i], w);

      b.onClick = function() {
        var i = this.index;

        var tc = this.parent.parent.parent;
        tc.config.currentPage = i;

        var c = tc.config.pages[i];
        c.parent = tc.children[0];

        var t = new tr.gui.component(tr.gui.container);
        t.setup(c);

        tc.children[0].children[1].clear();
        tc.children[0].children[1] = t;
      }
      c.push(b);
    }
    return {
      type: "container",
      background: "rgb(80, 80, 80)",
      size: {
        w: 1.0,
        h: 30
      },
      children: c,
    };
  },

  createButton: function(label, w) {
    return {
      id: "btn-tab-" + label,
      type: "container",
      size: {
        w: w,
        h: 30
      },
      children: [{
        type: "container",
        border: false,
        children: [{
          type: "text",
          text: label,
          textSize: 18,
          size: {
            w: 1,
            h: 1
          },
          padding: 4,
          align: {
            v: "TOP",
            h: "CENTER"
          },
        }],
      }]
    };
  }
}
