if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.tabControl = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  children.push.apply(children, c.controlHeader());
  children.push.apply(children, c.controlRow("a0"));
  children.push.apply(children, c.controlRow("a1"));
  children.push.apply(children, c.controlRow("a2"));
  children.push.apply(children, c.controlRow("a3"));
  children.push.apply(children, c.controlRow("a4"));
  children.push.apply(children, c.controlRow("g0"));
  children.push.apply(children, c.controlRow("h0"));
  children.push.apply(children, c.controlRow("h1"));

  children.push(c.spacer());
  children.push(c.btnBig("STOP", "/tr3/stop", true, "red"));
  children.push(c.btnBig("RELEASE", "/tr3/stop", false, "green"));
  children.push(c.btnBig("SHUTDOWN", "/tr3/shutdown", true, "red"));
  children.push(c.btnBig("POWER UP", "/tr3/powerup", true, "green"));

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
