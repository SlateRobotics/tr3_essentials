if (!tr) tr = {};
if (!tr.utils) tr.utils = {};

tr.utils.quaternionToEuler = function(q) {
  var q0 = q.w;
  var q1 = q.x;
  var q2 = q.y;
  var q3 = q.z;

  var yaw = Math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
  var pitch = Math.asin(-2.0*(q.x*q.z - q.w*q.y));
  var roll = Math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

  return({
    x: roll,
    y: pitch,
    z: yaw
  });
};
