#!/usr/bin/env node

var path = require('path');
var fs = require('fs');

var buildFileName = "tr.build.js";
var root = path.join(__dirname, '../client/js/');

var build = "";

var files = fs.readdirSync(root);

files.forEach(function (file) {
  var stat = fs.statSync(path.join(root, file));
  if (stat && stat.isDirectory()) {
    var dir_files = fs.readdirSync(path.join(root, file));
    dir_files.forEach(function (dir_file) {
      if (file.split('.')[0] == "tr" && file != buildFileName) {
        build += fs.readFileSync(path.join(root, file, dir_file), 'utf8');
      }
    });
  } else {
    if (file.split('.')[0] == "tr" && file != buildFileName) {
      build += fs.readFileSync(path.join(root, file), 'utf8');
    }
  }
});

fs.writeFileSync(path.join(root, buildFileName), build);
