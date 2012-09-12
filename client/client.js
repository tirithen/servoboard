var ServoBoard = require('./servoboard.js'),
	servoBoard = new ServoBoard();

servoBoard.on('error', function(error) {
	console.error('Servo Board:', error);
});

servoBoard.on('info', function(data) {
	console.log('Servo Board:', data);
});

servoBoard.on('send', function(data) {
	console.log('Client:', data);
});

servoBoard.on('connect', function() {
	var pos = 0;

	console.log('Servo Board:', 'Connected!');

	servoBoard.debugEnable();
	for(var i = 0, l = servoBoard.servos.length; i < l; i++) {
		servoBoard.servos[i].isEnabled = true;
	}

	setInterval(function() {
		pos += .1;
		if(pos > 1) {
			pos = 0;
		}
		for(var i = 0, l = servoBoard.servos.length; i < l; i++) {
			servoBoard.servos[i].goal = pos;
		}
	}, 5000);
});

servoBoard.on('disconnect', function() {
	console.log('Servo Board:', 'Disconnected!');
});

servoBoard.device = '/dev/ttyACM3';
servoBoard.baudrate = 115200;
servoBoard.connect();
