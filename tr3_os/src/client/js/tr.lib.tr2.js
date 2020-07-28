if (!tr) tr = {};
if (!tr.lib) tr.lib = {};

tr.lib.link = function (config) {
  this.id = config.id || "j0";
  this.meshId = config.meshId || this.id;
  this.offset = config.offset || 0;

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

tr.lib.tr2 = function () {
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

  this.setup = function () {
    this.arm.push(new tr.lib.link({
      id: "a0",
      meshId: "a0",
      rotate: {x: 90, y: 0, z: 0},
      translate: {x: 93.3, y: 729.2, z: 129.2},
      offset: -90,
    }));

    this.arm.push(new tr.lib.link({
      id: "a1",
      meshId: "a1",
      rotate: {x: 180, y: 0, z: -90},
      translate: {x: 89.7, y: 80.0, z: -214.6},
      offset: -90,
    }));

    this.arm.push(new tr.lib.link({
      id: "a2",
      meshId: "a2",
      rotate: {x: 90, y: -90, z: 0},
      translate: {x: -109.7, y: 166.9, z: 0},
    }));

    this.arm.push(new tr.lib.link({
      id: "a3",
      meshId: "a1",
      rotate: {x: 180, y: 0, z: -90},
      translate: {x: 154.4, y: 80.0, z: 0},
      offset: 20,
    }));
    
    this.arm.push(new tr.lib.link({
      id: "a4",
      meshId: "g0",
      rotate: {x: 90, y: 90, z: 0},
      translate: {x: 109.7, y: 166.9, z: 0},
    }));
  }

  this.setup();
}
