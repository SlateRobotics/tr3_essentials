tr.gui.listBox = {
    defaults: function() {
      this.itemHeight = this.config.itemHeight || 70;
      this.itemPadding = this.config.itemPadding || 15;
      this.items = this.config.items;
      this.title = this.config.title;
      this.currentItem = this.items[0];
    },

    setup: function () {
      var items = this.componentConfig.createItems.bind(this)();
      var container = new tr.gui.component(tr.gui.container);
      container.setup({
        type: "container",
        border: false,
        parent: this,
        margin: this.margin,
        padding: this.padding,
        children: [items]
      });
      this.children.push(container);
      this.componentConfig.change.bind(this)();
    },
  
    draw: function() {
      for (var i = 0; i < this.items.length; i++) {
        if (this.items[i] == this.currentItem) {
          this.children[0].children[0].children[i].background = "rgb(50, 50, 50)";
        } else {
          this.children[0].children[0].children[i].background = "rgb(100, 100, 100)";
        }
      }
    },

    change: function () {
      if (this.config.onChange) {
        this.config.onChange(this.currentItem);
      }
    },

    createItems: function () {
      var c = [];

      for (var i = 0; i < this.config.items.length; i++) {
        var item = this.componentConfig.createItem.bind(this)(this.config.items[i]);
  
        item.onClick = function() {
          var id = this.id.replace("listBox-item-", "");
          var listBox = this.parent.parent.parent;
          listBox.currentItem = id;
          listBox.componentConfig.change.bind(listBox)();
        }

        c.push(item);
      }
      return {
        type: "container",
        background: "rgb(150, 150, 150)",
        children: c,
      };
    },

    createItem: function(label) {
      return {
        id: "listBox-item-" + label,
        type: "container",
        size: {
          w: 1.0,
          h: this.itemHeight
        },
        children: [{
          type: "container",
          border: false,
          children: [{
            type: "text",
            text: label,
            textSize: 16,
            padding: this.itemPadding,
            align: {
              v: "CENTER",
              h: "LEFT"
            },
          }],
        }]
      };
    },
  }
  