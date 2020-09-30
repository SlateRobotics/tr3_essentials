if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.tabConfig = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  children.push.apply(children, c.configHeader());
  children.push.apply(children, c.configRow("a0"));
  children.push.apply(children, c.configRow("a1"));
  children.push.apply(children, c.configRow("a2"));
  children.push.apply(children, c.configRow("a3"));
  children.push.apply(children, c.configRow("a4"));
  children.push.apply(children, c.configRow("g0"));
  children.push.apply(children, c.configRow("h0"));
  children.push.apply(children, c.configRow("h1"));
  children.push.apply(children, c.configRow("b0"));
  children.push.apply(children, c.configRow("b1"));

  return {
    type: "container",
    size: {
      w: 1.0,
      h: "fill"
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: children,
  }
}
