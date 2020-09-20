if (!tr) tr = {};
if (!tr.controls) tr.controls = {};
if (!tr.controls.pnp2) tr.controls.pnp2 = {};

tr.controls.pnp2.cell = function(opts) {
  if (!opts.width) opts.width = 1;
  if (!opts.id) opts.id = "lbl-cell-" + opts.label;
  return {
    type: "container",
    size: {
      w: opts.width / 5,
      h: 1 / 6,
    },
    children: [{
      type: "container",
      border: false,
      children: [{
        id: opts.id,
        type: "text",
        text: opts.label,
        textSize: 12,
        textFont: "noto",
        onClick: opts.onClick,
        align: {
          v: "CENTER",
          h: "CENTER"
        },
      }],
    }]
  }
}
