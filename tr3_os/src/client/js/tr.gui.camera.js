tr.gui.camera = {
  defaults: function() {
    this.border = false;
    this.cameraImageUrl = 'http://' + location.hostname + ':8081/snapshot?topic=/camera/rgb/image_raw&type=ros_compressed';
    this.image = '';
    this.loadingImage = false;
  },

  setup: function () {
    this.loadingImage = true;
    loadImage(this.cameraImageUrl, function (img) {
      this.image = img;
      this.loadingImage = false;
    }.bind(this));
  },

  draw: function() {
    this.size = this.parent.size;

    if (!this.image) return;
    this.image.resize(this.size.w, this.size.h);
    image(this.image, 0, 0, this.image.width, this.image.height);

    if (!this.loadingImage) {
      this.loadingImage = true;
      loadImage(this.cameraImageUrl, function (img) {
        this.image = img;
        this.loadingImage = false;
      }.bind(this));
    }
  },

  drawPixels: function () {
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
