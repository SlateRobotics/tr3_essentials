tr.gui.drawLink = function(link, r_x, r_y, r_z, x, y, z, animate, _p5) {
  if (!_p5) _p5 = window;

  _p5.push();
  _p5.scale(0.2);

  _p5.noStroke();

  _p5.translate(x, y, z);
  _p5.rotateX(r_x);
  _p5.rotateY(r_y);
  _p5.rotateZ(r_z);

  if (animate) {
    animate();
  }

  _p5.directionalLight(255, 255, 255, 500, -1250, -1250);
  _p5.ambientLight(150);
  _p5.ambientMaterial(40);
  _p5.model(link);

  _p5.pop();
}
