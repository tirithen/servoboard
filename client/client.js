var ServoBoard = require('./servoboard.js'),
	servoBoard = new ServoBoard();

servoBoard._id = 9;

servoBoard.on('error', function (error) {
	console.error('Servo Board ERROR:', error);
});

servoBoard.on('info', function (data) {
	console.log('Servo Board INFO:', data);
});

servoBoard.on('debug', function (data) {
	console.log('Servo Board DEBUG:', data);
});

servoBoard.on('send', function (data) {
	console.log('Client:', data);
});

servoBoard.on('connect', function () {
	var pos = 0, i;

	console.log('Servo Board:', 'Connected!');

	servoBoard.debugEnable();
	servoBoard.debugEnable();
	servoBoard.debugEnable();
	for(i = 0, l = servoBoard.servos.length; i < l; i++) {
		servoBoard.servos[i].isEnabled = true;
	}

	setInterval(function () {
		pos += .1;
		if(pos > 1) {
			pos = 0;
		}

		for(i = 0, l = servoBoard.servos.length; i < l; i++) {
			servoBoard.servos[i].goal = pos;
		}
	}, 5000);
});

servoBoard.on('disconnect', function () {
	console.log('Servo Board:', 'Disconnected!');
});

//~ servoBoard.device = '/dev/arduino';
servoBoard.device = '/dev/ttyACM0';
servoBoard.baudrate = 115200;
servoBoard.connect();
