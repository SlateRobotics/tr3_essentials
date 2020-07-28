
var express = require('express');
var router = express.Router();
var path = require('path');

var _root = '/../../../client/views/';

router.use('/', function (req, res, next) {
	var filePath = path.join(__dirname, _root, '/index.html');
	res.sendFile(filePath);
});

module.exports = router;
