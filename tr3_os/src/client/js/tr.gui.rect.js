tr.gui.rect = function(x, y, wx, wy, r) {
  beginShape();

  var startAngle = TWO_PI * 0.00;
  var endAngle = TWO_PI * 0.25;

  var corners = [
    [x + wx, y + wy],
    [x, y + wy],
    [x, y],
    [x + wx, y]
  ];
  for (var i = 0; i < corners.length; i++) {
    var cx = corners[i][0];
    var cy = corners[i][1];

    cx -= r * Math.sign(cx - (wx + x) + 1);
    cy -= r * Math.sign(cy - (wy + y) + 1);

    var resolution = 15;
    for (var j = startAngle; j <= endAngle; j += (TWO_PI * 0.25) / resolution) {
      var vx = cx + r * Math.cos(j);
      var vy = cy + r * Math.sin(j);
      vertex(vx, vy);
    }

    startAngle += TWO_PI * 0.25;
    endAngle += TWO_PI * 0.25;
  }

  endShape(CLOSE);
}
