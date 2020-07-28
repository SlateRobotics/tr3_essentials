tr.gui.text = {
  defaults: function() {
    this.border = false;
    this.softAlign = false;

    if (!this.config.align) {
      this.config.align = {};
    }
    if (!this.config.align.v) {
      this.align.v = "TOP";
    }
    if (!this.config.align.h) {
      this.align.h = "LEFT";
    }

    if (!this.config.size) {
      this.config.size = {};
    }
    if (!this.config.size.w) {
      this.size.w = 1;
    }
    if (!this.config.size.h) {
      this.size.h = this.textSize + this.margin * 2
    }
  },

  draw: function() {
    noStroke();
    fill(this.textColor);
    textSize(this.textSize);
    textAlign(window[this.align.h], window[this.align.v]);
    text(this.text, this.pos.x, this.pos.y, this.size.w, this.size.h);
  }
};
