if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.controlPanel) tr.controls.controlPanel = {};

tr.controls.controlPanel.playbackBlock = function() {
  var c = tr.controls.controlPanel;

  var children = [];
  children.push(c.btnPlay());
  children.push(c.btnPause());
  children.push(c.btnAdd());
  children.push(c.btnRemove());
  children.push(c.btnWaypointLeft());
  children.push(c.labelProgram());
  children.push(c.btnWaypointRight());
  //children.push(c.btnPlaybackImg("/img/pnp-control0.png"));

  return {
    type: "container",
    size: {
      w: 3 / 9,
      h: 120,
    },
    padding: 10,
    background: "rgba(255, 255, 255, 0.2)",
    children: children,
  }
}
