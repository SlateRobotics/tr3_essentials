tr.gui.camera = {
  defaults: function() {
    this.border = false;
    this.image = '';
  },

  draw: function() {
    this.size = this.parent.size;
    if (!tr.data.cameraImage) return;

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
