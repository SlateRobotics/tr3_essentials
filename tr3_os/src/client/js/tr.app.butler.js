if (!tr) tr = {};
if (!tr.app) tr.app = {};

tr.app.butler = new App({
  id: 0,
  name: "Butler",
  desc: "Schedule Domestic Services and Chores",
  iconUrl: "/img/icon-app-butler",
  state: {

  },
  pages: [{
    id: "main",
    pos: {
      x: 0,
      y: 0
    },
    size: {
      w: 1.0,
      h: 1.0
    },
    header: {
      text: "Butler"
    },
    children: [{
      id: "colLeft",
      type: "container",
      size: {
        w: 0.5,
        h: 1
      },
      background: "rgba(255, 255, 255, 0.2)",
      radius: 15,
      margin: 10,
      padding: 15,
      children: [{
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: "false",
        children: [{
          type: "text",
          text: "Plan:",
          textSize: 20,
        }, {
          type: "text",
          text: "Time Remaining:",
          textSize: 20,
        }],
      }, {
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: "false",
        children: [{
          type: "text",
          align: {
            v: "TOP",
            h: "RIGHT"
          },
          text: "Basic",
          textSize: 20,
        }, {
          type: "text",
          align: {
            v: "TOP",
            h: "RIGHT"
          },
          text: "4 hrs 15 min",
          textSize: 20,
        }],
      }],
    }, {
      id: "scheduleservice",
      type: "container",
      size: {
        w: 0.5,
        h: 0.75
      },
      margin: 10,
      padding: 15,
      background: "rgba(37, 183, 250, 0.8)",
      radius: 15,
      align: {
        v: "CENTER",
        h: "CENTER"
      },
      onClick: function() {
        this.getApp().setPage(1);
      },
      children: [{
        type: "container",
        size: {
          w: 1.0,
          h: 60
        },
        align: {
          v: "CENTER",
          h: "CENTER"
        },
        border: false,
        children: [{
          type: "text",
          text: "SCHEDULE",
          textSize: 28,
          size: {
            w: 1,
            h: 30
          },
          align: {
            v: "TOP",
            h: "CENTER"
          },
        }, {
          id: "lblService",
          type: "text",
          text: "SERVICE",
          textSize: 28,
          size: {
            w: 1,
            h: 30
          },
          align: {
            v: "TOP",
            h: "CENTER"
          },
        }],
      }]
    }],
  }, {
    id: "schedule",
    size: {
      w: 1.0,
      h: 1.0
    },
    header: {
      text: "Butler | Schedule Service"
    },
    children: [{
      type: "container",
      size: {
        w: 1,
        h: 0.8
      },
      margin: 10,
      padding: 15,
      background: "rgba(255,255,255,0.2)",
      radius: 15,
      children: [{
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: false,
        align: {
          v: "TOP",
          h: "LEFT"
        },
        children: [{
          type: "text",
          size: {
            w: 1,
            h: 40
          },
          text: "Date of Service:",
          align: {
            v: "CENTER",
            h: "LEFT"
          },
        }, {
          type: "text",
          size: {
            w: 1,
            h: 40
          },
          text: "Service Type:",
          align: {
            v: "CENTER",
            h: "LEFT"
          },
        }],
      }, {
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: false,
        children: [{
          type: "datePicker",
          border: false,
          size: {
            w: 1,
            h: 40
          },
          onChange: function(d) {
            console.log(d);
          },
        }, {
          type: "select",
          size: {
            w: 1,
            h: 40
          },
          padding: 5,
          options: ["Laundry", "Kitchen Cleanup", "House Pickup"],
          textSize: 22,
        }],
      }],
    }, {
      id: "buttons",
      type: "container",
      size: {
        w: 1.0,
        h: 0.2
      },
      border: "false",
      children: [{
        type: "container",
        size: {
          w: 0.5,
          h: 1
        },
        border: false,
      }, {
        type: "container",
        size: {
          w: 0.25,
          h: 1
        },
        background: "rgba(255, 255, 255, 0.2)",
        radius: 15,
        margin: 10,
        padding: 15,
        onClick: function() {
          this.getApp().setPage(0);
        },
        children: [{
          type: "text",
          size: {
            w: 1,
            h: 1
          },
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          text: "Close",
          textSize: 22,
        }],
      }, {
        type: "container",
        size: {
          w: 0.25,
          h: 1
        },
        background: "rgba(37, 183, 250, 0.8)",
        radius: 15,
        margin: 10,
        padding: 15,
        onClick: function() {
          this.getApp().setPage(0);
        },
        children: [{
          type: "text",
          size: {
            w: 1,
            h: 1
          },
          align: {
            v: "CENTER",
            h: "CENTER"
          },
          text: "Submit",
          textSize: 22,
        }],
      }],
    }],
  }],
});
