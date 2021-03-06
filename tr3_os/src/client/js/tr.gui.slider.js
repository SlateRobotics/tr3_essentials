tr.gui.slider = {
  defaults: function() {
    this.border = false;
    this.min = this.config.min;
    this.max = this.config.max;
    this.val = this.config.val;
    this.step = this.config.step || 1;
    this.disabled = this.config.disabled;
    this.element = createSlider(this.min, this.max, this.val, this.step);
    this.onChange = this.config.onChange;
    this.onInput = this.config.onInput;

    this.inputed = function() {
      var val = this.element.value();
      if (this.onInput) {
        this.onInput(val);
      }
    };

    this.changed = function() {
      var val = this.element.value();
      if (this.onChange) {
        this.onChange(val);
      }
    };

    this.setval = function(val) {
      this.element.value(val);
      this.onInput(val);
      this.onChange(val);
    };
  },

  setup: function() {
    this.element.hide();
    this.element.input(this.inputed.bind(this));
    this.element.changed(this.changed.bind(this));
  },

  draw: function() {
    var pos = this.getAbsolutePosition();
    this.element.position(pos.x, pos.y);
    this.element.size(this.size.w - this.padding * 2 - 3, this.size.h - this.padding * 2);
    this.element.show();

    if (this.disabled) {
      this.element.elt.disabled = true;
    } else {
      this.element.elt.disabled = false;
    }
  },

  clear: function() {
    this.element.remove();
  },
}
