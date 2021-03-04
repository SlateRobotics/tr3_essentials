tr.gui.camera = {
  defaults: function() {
    this.border = false;
    this.cameraImageUrl = 'http://' + location.hostname + ':8081/stream?topic=/camera/rgb/image_rect_color&quality=15';
    this.image = '';
    this.loadingImage = false;
    this.element = '';
  },

  setup: function() {
    this.loadingImage = true;
    this.element = createElement("img", "");
    this.element.attribute("src", this.cameraImageUrl);
    this.element.style("user-select", "none");
    this.element.hide();
  },

  draw: function() {
    this.size = this.parent.size;

    var pos = this.getAbsolutePosition();
    this.element.position(pos.x, pos.y);
    this.element.size(this.size.w, this.size.h);
    this.element.show();
  },

  clear: function() {
    this.element.remove();
  },

  drawPixels: function() {
    this.image = createImage(tr.data.cameraImage.width, tr.data.cameraImage.height);
    this.image.loadPixels();

    var pxls = new Uint8Array(tr.data.cameraImage.data);
    for (var i = 0; i < this.image.height; i++) {
      for (var j = 0; j < this.image.width; j++) {
        var idx = (i * this.image.width + j) * 3;
        var r = pxls[idx];
        var g = pxls[idx + 1];
        var b = pxls[idx + 2];
        this.image.set(j, i, [r, g, b, 255]);
      }
    }

    this.image.updatePixels();
    this.image.resize(this.parent.size.w, this.parent.size.h);
    image(this.image, this.pos.x, this.pos.y);
  },
};
