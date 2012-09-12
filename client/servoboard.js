// Load external dependencies
var	Buffer = require('buffer').Buffer
	EventEmitter = require('events').EventEmitter,
	SerialPort = require('serialport');


// ServoBoard constructor
var ServoBoard = module.exports = function ServoBoard() {

	// Default configuration
	this.device = '~/dev/arduino'; // The system path to the servo board serial port
	this.baudrate = 115200;
	this.reconnectDelay = 3000; // Millisecond to automatic reconnect attempt
	this.sendRetryDelay = 20; // Milliseconds to check if there is anything to send
	this.servoUpdateDelay = 20 // Milliseconds to enqueue new servo goals/enable/disable/calibrate

	// Setup
	this._id = 0; // Must be 0-15
	this._queue = [];
	this._boardReady = false;
	this._attemptReconnect = true;
	this.servos = [];
	for(var i = 0; i < 16; i++) {
		this.servos.push(new Servo(i, this));
	}
	this._connection = null;

	this.latestInstructions = {
		SERVOS_GOAL: [],
		SERVOS_ENABLED: 0,
		SERVOS_CALIBRATE: 0
	};

	this._servoUpdate(); // Start the servo update timer
	this._send(); // Start the send timer
};

// Interit from EventEmitter
ServoBoard.prototype.__proto__ = EventEmitter.prototype;

// ServoBoard input/output instructions
ServoBoard.prototype.instructions = {
	END_OF_INSTRUCTION: 255,
	input: {
		DEBUG_ENABLE:			0,	// Enables debug output from the board
		DEBUG_DISABLE:			1,	// Disables debug output from the board
		SET_ID:					2,	// Set the servo board id
		SERVOS_GOAL:			3,	// Set goals for up to all 16 servos, needs a 1-16 bytes argument
		SERVOS_ENABLED:			4,	// Enable/disable multiple servos by setting 1/0 in a 1-16-bit argument
		SERVOS_CALIBRATE:		5,	// Run calibration for all servos that has a 1 in a 1-16-bit argument
	},
	output: {
		READY:					0,	// Servo board is ready to recieve instructions
		INFO:					1,	// Info message
		DEBUG:					2,	// Debug message
		ERROR:					3,	// Error code one byte then message
		ALL_SERVOS_POSITION:	4,	// 16 bytes with servo positions
		ALL_SERVOS_LOAD:		5	// 16 bytes with servo loads
	},
	error: {
		UNKNOWN: {
			code: 0,
			message: 'Unknown error'
		},
		NO_CONNECTION: {
			code: 1,
			message: 'No connection'
		},
		INSTRUCTION_ILLEGAL: {
			code: 2,
			message: 'Illegal instruction'
		},
		RESPONSE_ILLEGAL: {
			code: 3,
			message: 'Servo board returned an illegal response'
		}
	}
};

ServoBoard.prototype.connect = function() {
	var self = this;

	self.attemptReconnect = true;

	self._connection = new SerialPort.SerialPort(self.device, {
		baudrate: self.baudrate,
		parser: SerialPort.parsers.readline(self.instructions.END_OF_INSTRUCTION)
	});

	self._connection.on('data', function(data) {
		self._dataHandler(data);
	});

	self._connection.on('error', function(error) {
		self._errorHandler(error);
	});

	self._connection.on('close', function() {
		self.emit('disconnect');
		self._boardReady = false;
		self._connection = null;
		if(self.attemptReconnect) {
			self.connect();
		}
	});

	setTimeout(function() {
		if(!self._boardReady && self.attemptReconnect) {
			self.connect();
		}
	}, self.reconnectDelay);
};

ServoBoard.prototype.disconnect = function() {
	this._attemptReconnect = false;

	if(this._connection) {
		this._connection.close();
		this._connection = null;
	}

	this._boardReady = false;
};

ServoBoard.prototype.map = function(x, inMin, inMax, outMin, outMax, round) {
	var result = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

	if(round) {
		return Math.round(result);
	}

	return result;
};

ServoBoard.prototype._servoUpdate = function() {
	var	self = this,
		differentThanLatestGoal = false,
		SERVOS_GOAL = [self.instructions.input.SERVOS_GOAL],
		SERVOS_ENABLED = 0,
		SERVOS_CALIBRATE = 0,
		instruction;

	self.servos.forEach(function(servo, i) {
		if(typeof servo.goal !== 'number') {
			servo.goal = 0;
		}
		else if(servo.goal < 0) {
			servo.goal = 0;
		}
		else if(servo.goal > 1) {
			servo.goal = 1;
		}

		SERVOS_GOAL.push(self.map(servo.goal, 0, 1, 0, 254, true));

		if(servo.goal !== self.latestInstructions.SERVOS_GOAL[i]) {
			differentThanLatestGoal = true;
		}

		if(servo.isEnabled) {
			SERVOS_ENABLED += 1 << i;
		}

		if(servo._calibrate) {
			SERVOS_CALIBRATE += 1 << i;
			servo._calibrate = false;
		}

	});

	if(
		differentThanLatestGoal ||
		SERVOS_GOAL.length !== self.latestInstructions.SERVOS_GOAL.length
	) {
		self._clearInstructionsFromQueueByType(self.instructions.input.SERVOS_GOAL);
		self._enqueue(SERVOS_GOAL);
	}

	if(SERVOS_ENABLED !== self.latestInstructions.SERVOS_ENABLED) {
		instruction = self.instructions.input.SERVOS_ENABLED;
		self._clearInstructionsFromQueueByType(instruction);
		self._enqueue([instruction, SERVOS_ENABLED]);
	}

	if(SERVOS_CALIBRATE) {
		instruction = self.instructions.input.SERVOS_CALIBRATE;
		self._clearInstructionsFromQueueByType(instruction);
		self._enqueue([instruction, SERVOS_CALIBRATE]);
	}

	setTimeout(function() {
		self._servoUpdate();
	}, self.servoUpdateDelay);
};

ServoBoard.prototype._clearInstructionsFromQueueByType = function(type) {
	var i,
		instruction = type + this._id << 4;

	for(i = this._queue.length - 1; i >= 0; i--) {
		if(this._queue[i][0] === instruction) {
			this._queue.splice(i, 1);
		}
	}
};

ServoBoard.prototype.getInputNameByCode = function(code) {
	var instructions = this.instructions.input, i;

	for(i in instructions) {
		if(
			instructions.hasOwnProperty(i) &&
			instructions[i] == code
		) {
			return i;
		}
	}

	return null;
};

ServoBoard.prototype.getOutputNameByCode = function(code) {
	var instructions = this.instructions.output, i;

	for(i in instructions) {
		if(
			instructions.hasOwnProperty(i) &&
			instructions[i] == code
		) {
			return i;
		}
	}

	return null;
};

ServoBoard.prototype.getErrorByCode = function(code) {
	var errors = this.instructions.error, i;

	for(i in errors) {
		if(
			errors.hasOwnProperty(i) &&
			errors[i].code == code
		) {
			return errors[i];
		}
	}

	return errors.UNKNOWN;
};

ServoBoard.prototype.getId = function() {
	return this._id;
};

ServoBoard.prototype.setId = function(id) {
	if(id < 0 || id > 15) {
		throw 'Servo Board id must be in the range 0-15';
	}

	this._id = id;
	this._enqueue(this.instructions.SET_ID, id);
};

ServoBoard.prototype.debugEnable = function() {
	this._enqueue(this.instructions.DEBUG_ENABLE);
};

ServoBoard.prototype.debugDisable = function() {
	this._enqueue(this.instructions.DEBUG_DISABLE);
};

ServoBoard.prototype._enqueue = function() {
	arguments[0] += this._id << 4; // Add the servo board id to the first argument
	arguments[arguments.length] = this.instructions.END_OF_INSTRUCTION;
	this._queue.push(new Buffer(arguments));
};

ServoBoard.prototype._send = function() {
	var self = this;

	if(self._boardReady && self._queue.length > 0) {
		self._connection.write(self._queue.shift(), function() {
			// TODO: figure out what to do on error here
			self._send();
		});
	}
	else {
		setTimeout(function() {
			self._send();
		}, self.sendRetryDelay);
	}
};

ServoBoard.prototype._dataHandler = function(data) {
	var	self = this,
		id = data[0] >> 4,
		instruction = data[0] & 0x0f,
		instructionName = self.getOutputNameByCode(instruction),
		outputInstructions = self.instructions.output,
		temp;

	data.shift();

	if(id === self._id && data.length === 16 && instructionName) {
		switch(instruction) {
			case outputInstructions.READY:
				self.emit('connect');
				self._boardReady = true;
				break;
			case outputInstructions.INFO:
				this.emit('info', data);
				break;
			case outputInstructions.DEBUG:
				this.emit('debug', data);
				break;
			case outputInstructions.ERROR:
				self.emit('error', self.getErrorByCode(data[0]));
				break;
			case outputInstructions.ALL_SERVOS_POSITION:
				data.forEach(function(position, i) {
					temp = self.servos[i];
					if(temp instanceof Servo) {
						temp.position = self.map(position, 0, 254, 0, 1);
					}
				});
				break;
			case outputInstructions.ALL_SERVOS_LOAD:
				data.forEach(function(load, i) {
					temp = self.servos[i];
					if(temp instanceof Servo) {
						temp.load = self.map(load, 0, 254, 0, 1);
					}
				});
				break;
			default:
				self.emit('error', self.instructions.error.RESPONSE_ILLEGAL);
		}
	}
};

ServoBoard.prototype._errorHandler = function(error) { // Connection errors
	this.emit('error', this.instructions.error.NO_CONNECTION);
};


// Servo constructor
var Servo = function Servo(index, servoBoard) {
	this.index = index;
	this.servoBoard = servoBoard;
	this.goal = 0;
	this.position = 0;
	this.load = 0;
	this.isEnabled = false;
	this._calibrate = false;
};

// Interit from EventEmitter
Servo.prototype.__proto__ = EventEmitter.prototype;

// Servo methods
Servo.prototype.calibrate = function() {
	return this._calibrate = true;
};
