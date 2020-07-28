if (!tr) tr = {}
if (!tr.app) tr.app = {}
if (!tr.app.pnp) tr.app.pnp = {};
//boop//
tr.app.pnp.pages = [{
  id: "PNP01",
 
  pos: {
    x: 0,
    y: 0
  },
  size: {
    w: 1.0,
    h: 1.0
  },
  header: {
    text: "P.N.P.",
  },
  children: [{
    id: "colLeft",
    type: "container",
    align: {
      v: "CENTER",
      h: "CENTER"
    },
    size: {
      w: 0.5,
      h: 1
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 10,
    padding: 5, 
    children: [{
      id: "ProgramBar",
      type: "container",
      size: {
        w: 1.0,
        h: 1 / 8
      },
      background: "rgba(255, 255, 255, 0.5)",
      children: [{
        type: "container",
        size: {
          w: 1 / 4,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          id: "progselect",
          type: "select",
          size: {
            w: 1,
            h: 40
          },
          padding: 5,
          options: ["Did Not Load", ],
          textSize: 22,
          onChange: function(val) {
            var app = this.getApp();
            app.config.changeProgram(val);
          },
        }],

      }, {
        type: "container",
        size: {
          w: 1 / 4,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-del0.png",
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 4,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-new0.png",
          onClick: function() {
            console.log("New Program");
            var app = this.getApp();
            app.config.addProgram();
          },
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 4,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-set0.png",
        }],
      }],



    }, {
      type: "container",
      size: {
        w: 1,
        h: 1 / 10
      },
      background: "rgba(255, 255, 255, 0.5)",
      children: [{
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-bw0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.getCurrentProgram().incrementWaypoint(-1);
          },
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          id: "dwaypoint",
          type: "text",
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          text: "",
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-fw0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.getCurrentProgram().incrementWaypoint(1);
          },
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-del1.png",
          onClick: function() {
            var app = this.getApp();
            var prog = app.config.getCurrentProgram();
            prog.removeWaypoint();
          },
        }],
      }, {
        type: "container",
        size: {
          w: 1 / 5,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-add0.png",
          onClick: function() {
            var app = this.getApp();
            var prog = app.config.getCurrentProgram();
            prog.insertWaypoint();
          },
        }],


      }],
    }, {
      id: "DisplayBar",
      type: "container",
      size: {
        w: 1,
        h: 1 / 3
      },
      background: "rgba(255, 255, 255, 0.5)",
      children: [{
        type: "container",
        size: {
          w: 1 / 2,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [{
			type: "container",
			border: true,
			size: {
				w: 1,
			h: 1/5},
			children: [{
				type: "text",
				text: "Goal Position"
			}],
		},{
			type: "container",
			border: true,
			size: {
				w: 1,
				h: 1/5
			},
				children: [{
					type: "container",
			border: true,
			size: {
				w: 1/5,
				h: 1
			},
			children: [{
				type: "text",
				text: "X"
			}],
			}],
		},{
			type: "container",
			border: true,
			size: {
				w: 1,
				h: 1/5
			},
				children: [{
					type: "container",
			border: true,
			size: {
				w: 1/5,
				h: 1
			},
			children: [{
				type: "text",
				text: "Y"
			}],
			}],
			
		},{
			type: "container",
			border: true,
			size: {
				w: 1,
				h: 1/5
			},
				children: [{
					type: "container",
			border: true,
			size: {
				w: 1/5,
				h: 1
			},
			children: [{
				type: "text",
				text: "Z"
			}],
			}],
			
		},{
			type: "container",
			border: true,
			size: {
				w: 1,
				h: 1/5
			},
			
			
			
			
			
			
		}],
      }, {
        type: "container",
        size: {
          w: 1 / 2,
          h: "fill"
        },
        background: "rgba(255, 255, 255, 0.5)",
        children: [],
      }],

    }, {
      // 1st //
      id: "ControlBar",
      type: "container",

      background: "rgba(255, 255, 255, 0.5)",
      size: {
        w: 1,
        h: 1 / 2.3
      },
      children: [{
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          type: "container",
          border: false,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],

        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {

          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control3.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control3.png",
			size: {
          w: 26,
          h: 35
          },
		  }],

        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control2.png",
			size: {
          w: 26,
          h: 35
          },
		  }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control0.png",
			size: {
          w: 26,
          h: 35
        },
          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control1.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "Z",
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
             align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control4.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
             align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control4.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],




        }],
      }, {



        // 2nd //
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          border: false,
          type: "container",
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [], //
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
             align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control5.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control5.png",
			size: {
          w: 26,
          h: 35
		  },

          }],
        }, {
          type: "container",

          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
             align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control5.png",
			size: {
          w: 26,
          h: 35
		  },

          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "rotationdisplay0",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "X",

          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "rotationdisplay1",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "Y",

          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "rotationdisplay2",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "Z",

          }],
		  onClick: function(){
			  console.log(this.size.w + 'y: ' +this.size.h)
		  },
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{

            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control6.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",

          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control6.png",
			size: {
          w: 26,
          h: 35
		  },

          }],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },

          children: [{
             align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control6.png",
			size: {
          w: 26,
          h: 35
		  },

          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],
        }],
      }, {
        // 3rd //
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          border: false,
          type: "container",
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
             align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control3.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
             align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control3.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "speeddisplay",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "0",
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            id: "incrimentdisplay",
            align: {
              v: "CENTER",
              h: "CENTER"
            },
            type: "text",
            text: "1",

          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control4.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [],
        }, {
          type: "container",
          size: {
            w: 1 / 5,
            h: 1 / 5
          },
          children: [{
            align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control4.png",
			size: {
          w: 26,
          h: 35
		  },
          }],
        }, {
          type: "container",
          border: false,
          size: {
            w: 1,
            h: 1 / 5
          },
          children: [],

        }],
      }]

    }],
  }, {
    id: "colRight",
    type: "container",
    size: {
      w: 0.5,
      h: 1
    },
    background: "rgba(255, 255, 255, 0.2)",
    margin: 10,
    padding: 5,
    children: [{
      type: "container",
      background: "rgba(255, 255, 255, 0.5)",
      size: {
        w: 1,
        h: 0.9
      },
      children: [{
        id: "tr",
        cameraPos: {
          x: 90,
          y: 0,
          z: 275
        },
        type: "tr2",
      }],
    }, {
      type: "container",
      background: "rgba(255, 255, 255, 0.5)",
      size: {
        w: 1,
        h: "fill"
      },
      children: [{
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-play0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.programStartFrom();
          },
        }],
      }, {
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-pause0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.programStop();
          },
        }],
      }, {
        type: "container",
        background: "rgba(255, 255, 255, 0.5)",
        size: {
          w: 1 / 3,
          h: 1
        },
        children: [{
          align: {
              v: "CENTER",
              h: "CENTER"
            },type: "image",
            url: "/img/pnp-control-stop0.png",
          onClick: function() {
            var app = this.getApp();
            app.config.programStop();
          },

        }],



      }],
    }],
  }], // Page01 Close

}]
