var net = require('net');
var fs = require('fs');
var request = require('request');
var moment = require('moment');
var jsonParser=require('body-parser').json();

var HTTP_PORT = 2345;

// http server
var express = require('express');
var app = express(),
  server = require('http').Server(app);  
app.use(jsonParser);
server.listen(HTTP_PORT);
console.log("start");

var loc = {x:0, y:0};
// ble location interface
app.post('/ble/location', function (req, res) {
	console.log(req.body);
	
	loc = req.body;
	
	var d = {errcode:-1};
	res.send( JSON.stringify(loc) );
});

app.get('/bleloc', function (req, res) {
	//console.log(req.body);
	
	var d = loc;
	res.send( JSON.stringify(d) );
});

