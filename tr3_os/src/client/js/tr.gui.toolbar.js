tr.gui.toolbar = {
  defaults: function() {
    this.border = false;
    this.size = {
      w: 1.0,
      h: 40
    };
    this.pos = this.config.pos || {
      x: 0,
      y: 0
    };
    this.align = {
      v: "TOP",
      h: "Left"
    };
    this.background = this.config.background || "rgba(255,255,0,0.5)";

    this.toolbarItems = this.config.children;
    this.config.children = [];
    this.children = [];
  },

  setup: function() {

    var toolbarcontainer = new tr.gui.component(tr.gui.container);
    var toolbarcontainersetup = {
      id: "Ctoolbar",
      border: false,
      parent: this,
      size: {
        w: 1.0,
        h: 40
      },
      align: {
        v: "TOP",
        h: "Left"
      },
      background: this.background,
      pos: this.pos,
      children: []
    };

    for (var i = 0; i < this.toolbarItems.length; i++) {
      var item = this.toolbarItems[i];
      toolbarcontainersetup.children.push({

        id: 'TB' + i,
        type: "container",
        margin: 0,
        align: {
          v: "TOP",
          h: "Left"
        },
        size: {
          w: 1 / this.toolbarItems.length,
          h: 40
        },
        background: "rgba(255,255,255,.8)",
        children: [{
          type: "image",
          size: {
            w: 1,
            h: 40
          },
          align: {
            v: "TOP",
            h: "Left"
          },
          url: item.img
        }],
        onClick: item.onClick
      })
    }
    toolbarcontainer.setup(toolbarcontainersetup);

    this.children.push(toolbarcontainer);
  },
}
