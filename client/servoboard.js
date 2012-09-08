// Load external dependencies
var EventEmitter = require('events').EventEmitter,
	SerialPort = require('serialport');


// ServoBoard constructor
var ServoBoard = module.exports = function ServoBoard() {
	this.device = '/dev/arduino';
	this.baudrate = 115200;
	this.servos = [];
	for(var i = 0; i < 16; i++) {
		this.servos.push(new Servo(i, this));
	}
	this.connection = null;
};

// Interit from EventEmitter
ServoBoard.prototype.__proto__ = EventEmitter.prototype;

// ServoBoard methods
ServoBoard.prototype.connect = function() {
	var self = this;

	self.connection = new SerialPort.SerialPort(self.device, {
		baudrate: self.baudrate,
		parser: SerialPort.parsers.readline('\r\n')
	});
//~ console.log(self);
	self.connection.on('data', function(data) {
		self._dataHandler(data);
	});

	self.connection.on('error', function(error) {
		self._errorHandler(error);
	});

	self.connection.on('close', function() {
		self.emit('disconnect');
	});
};

ServoBoard.prototype.disconnect = function() {
	if(this.connection) {
		this.connection.close();
		this.connection = null;
	}
};

ServoBoard.prototype.debugEnable = function() {
	this.send('DE');
};

ServoBoard.prototype.debugDisable = function() {
	this.send('DD');
};

ServoBoard.prototype.send = function(instruction) {
	// TODO: add better instruction validation
	if(!this.connection) {
		this.emit('error', {
			name: 511,
			message: 'No connection to servo board.'
		});
		return false;
	}

	if(!instruction) {
		this.emit('error', {
			name: 400,
			message: 'No instruction given'
		});
		return false;
	}

	this.connection.write(instruction + '\n');
};

ServoBoard.prototype._dataHandler = function(data) {
	data = data.split(',');

	if(data[0] === 'IN') { // The board returned an informative message
		this.emit('info', data[1]);
	}
	else if(data[0] === 'ER') { // The board returned an error
		this.emit('error', {
			name: parseInt(data[1]),
			message: data[2]
		});
	}
	else if(data[0] === 'RY') { // The board is ready to recieve input
		this.emit('connect');
	}
	else { // The board returned an unknown message
		this.emit('error', {
			name: 500,
			message: 'Unknown message from board'
		});
	}
};

ServoBoard.prototype._errorHandler = function(error) { // Connection errors
	this.emit('error', {
		name: 510,
		message: error.message,
		full: error
	});
};


// Servo constructor
var Servo = function Servo(index, servoBoard) {
	this.index = index;
	this.servoBoard = servoBoard;
	this._goal = 0;
	//this._position = 0;
	//this._load = 0;
	this._isEnabled = false;
};

// Interit from EventEmitter
Servo.prototype.__proto__ = EventEmitter.prototype;

// Servo methods
Servo.prototype.setGoal = function(goal) {
	goal = parseInt(goal);
	if(typeof goal == 'number' && goal >= 0 && goal <= 999) {
		this.servoBoard.send('SG,' + this.index + ',' + goal);
		this._goal = goal;
	}
	else {
		throw Error('Goal must be between 0 and 999');
	}
};

Servo.prototype.getGoal = function() {
	return this._goal;
};
