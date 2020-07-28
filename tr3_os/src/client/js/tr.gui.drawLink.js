tr.gui.drawLink = function(link, r_x, r_y, r_z, x, y, z, animate, _p5) {
  if (!_p5) _p5 = window;

  _p5.push();
  _p5.scale(0.2);

  _p5.directionalLight(250, 250, 250, 500, -1250, -500);
  _p5.noStroke();
  _p5.ambientMaterial(100);

  _p5.rotateX(r_x);
  _p5.rotateY(r_y);
  _p5.rotateZ(r_z);
  _p5.translate(x, y, z);

  if (animate) {
    animate();
  }

  _p5.model(link);

  _p5.pop();
}
