#!/usr/bin/env node

var path = require('path');
var fs = require('fs');

var buildFileName = "tr.build.js";

var build = "";

function buildFiles (root, files) {
  files.forEach(function (file) {
    var stat = fs.statSync(path.join(root, file));
    if (stat && stat.isDirectory()) {
      var dir_files = fs.readdirSync(path.join(root, file));
      buildFiles(path.join(root, file), dir_files);
    } else {
      if (path.join(root, file).includes("tr.") && file != buildFileName) {
        build += fs.readFileSync(path.join(root, file), 'utf8');
      }
    }
  });
}

var root = path.join(__dirname, '../client/js/');
var files = fs.readdirSync(root);
buildFiles(root, files);

fs.writeFileSync(path.join(root, buildFileName), build);
