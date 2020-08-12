if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.pnp2 = function() {
  var c = tr.controls.pnp2;

var columns = [];
columns.push(c.columnLeft());
columns.push(c.columnRight());

  return new App({
    id: 4,
    name: "P.N.P. V2",
    iconUrl: "/img/icon-app-pnp",
    pages: [{
      pos: {
        x: 0,
        y: 0
      },
      header: {
        text: "P.N.P v2",
      },
      children: columns,
  }],
  });
};
