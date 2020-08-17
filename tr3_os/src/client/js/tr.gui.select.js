tr.gui.select = {
  defaults: function() {
    this.border = false;
    this.options = this.config.options || [];
    this.defaultValue = this.config.defaultValue || "";
    this.onChange = this.config.onChange;
    this.element = "";
    this.textSize = this.config.textSize || 10;
    this.changed = function() {
      var val = this.element.value();
      if (this.onChange) {
        this.onChange(val);
      }
    }
  },

  setup: function() {
    this.setOptions = function(o) {
      if (this.element) {
        this.element.hide();
      }

      this.element = createSelect('');
      for (var i = 0; i < o.length; i++) {
        this.element.option(o[i]);
      }

      this.element.hide();
      this.element.changed(this.changed.bind(this));

      if (this.defaultValue) {
        this.element.value(this.defaultValue);
      }

      this.options = o
      this.changed();
    }.bind(this);

    this.setOptions(this.options);
  },

  draw: function() {
    var pos = this.getAbsolutePosition();
    this.element.style("font-size", this.textSize + "px");
    this.element.position(pos.x, pos.y);
    this.element.size(this.size.w - this.padding * 2, this.size.h - this.padding * 2);
    this.element.show();
  },

  clear: function() {
    this.element.remove();
  },

}
