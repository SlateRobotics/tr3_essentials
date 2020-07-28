tr.gui.image = {
  defaults: function() {
    this.border = false;
    this.url = this.config.url;
    this.image = loadImage(this.url);
    this.rotation = this.config.rotation || 0;
  },

  draw: function() {
    angleMode(DEGREES);

    this.translate(this.size.w / 2, this.size.h / 2);
    rotate(this.rotation);
    this.translate(-this.size.w / 2, -this.size.h / 2);
    this.image.resize(this.size.w, this.size.h);
    image(this.image, this.pos.x, this.pos.y);
  },
}
