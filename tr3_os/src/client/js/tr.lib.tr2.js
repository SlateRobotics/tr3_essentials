if (!tr) tr = {};
if (!tr.lib) tr.lib = {};

tr.lib.link = function(config) {
  this.id = config.id || "j0";
  this.meshId = config.meshId || this.id;
  this.offset = config.offset || 0;
  this.axis = config.axis || "Y";
  this.flip = config.flip || false;
  this.fixed = config.fixed || false;

  this.meshOffset = config.meshOffset || {
    x: 0,
    y: 0,
    z: 0
  }

  this.rotate = config.rotate || {
    x: 0,
    y: 0,
    z: 0
  }

  this.translate = config.translate || {
    x: 0,
    y: 0,
    z: 0
  }
}

tr.lib.tr2 = function() {
  this.head = [];
  this.arm = [];
  this.base = [];

  this.state = {};
  this.state.pos = {
    x: 0,
    y: 0,
    z: 0
  };
  this.state.a0 = 0;
  this.state.a1 = 0;
  this.state.a2 = 0;
  this.state.a3 = 0;
  this.state.a4 = 0;
  this.state.g0 = 0;
  this.state.h0 = 0;
  this.state.h1 = 0;

  this.setup = function() {
    // HEAD
    this.head.push(new tr.lib.link({
      id: "h0",
      meshId: "h0",
      axis: "Z",
      flip: true,
      rotate: {
        x: 0,
        y: 0,
        z: 1.5708
      },
      translate: {
        x: 0,
        y: 357.9,
        z: 703.25
      },
    }));

    this.head.push(new tr.lib.link({
      id: "h1",
      meshId: "h1",
      axis: "Z",
      flip: true,
      rotate: {
        x: -1.5708,
        y: 3.1415,
        z: -1.5708
      },
      translate: {
        x: -166.17,
        y: 67,
        z: 249.17
      },
    }));

    // ARM
    this.arm.push(new tr.lib.link({
      id: "a0",
      meshId: "a0",
      axis: "Z",
      rotate: {
        x: 0,
        y: 0,
        z: 0
      },
      translate: {
        x: 101.6,
        y: 122.9,
        z: 459.937
      },
      offset: 1.5708,
    }));

    this.arm.push(new tr.lib.link({
      id: "a1",
      meshId: "a1",
      axis: "Z",
      rotate: {
        x: -1.5708,
        y: 0,
        z: 0
      },
      translate: {
        x: 0,
        y: 70,
        z: 76
      },
      offset: 0,
    }));

    this.arm.push(new tr.lib.link({
      id: "a1_fixed",
      meshId: "a2",
      fixed: true,
      rotate: {
        x: 0,
        y: 0,
        z: 0
      },
      translate: {
        x: 0,
        y: 190,
        z: 0
      },
    }));

    this.arm.push(new tr.lib.link({
      id: "a2",
      meshId: "a3",
      axis: "Z",
      meshOffset: {
        x: 8,
        y: 300,
        z: 0
      },
      rotate: {
        x: 3.1415,
        y: 0,
        z: 1.5708
      },
      translate: {
        x: 0,
        y: 0,
        z: 0
      },
      offset: -0.698132,
    }));

    this.arm.push(new tr.lib.link({
      id: "a3",
      meshId: "a4",
      axis: "Z",
      flip: true,
      rotate: {
        x: 0,
        y: 3.1415,
        z: -1.5708
      },
      translate: {
        x: 8,
        y: 300,
        z: 0
      },
      offset: 0.698132,
    }));

    this.arm.push(new tr.lib.link({
      id: "a4",
      meshId: "g0",
      axis: "Z",
      flip: true,
      rotate: {
        x: 1.5708,
        y: 0,
        z: 0
      },
      translate: {
        x: 0,
        y: 67,
        z: 67
      },
    }));
  }

  this.setup();
}
