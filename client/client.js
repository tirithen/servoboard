var ServoBoard = require('./servoboard.js'),
	servoBoard = new ServoBoard();

servoBoard.on('error', function(error) {
	console.error('Servo Board:', error);
});

servoBoard.on('info', function(data) {
	console.log('Servo Board:', data);
});

servoBoard.on('connect', function() {
	var pos = 0;

	console.log('Servo Board:', 'Connected!');

	servoBoard.debugEnable();

	setInterval(function() {
		pos += 100;
		if(pos > 999) {
			pos = 0;
		}
		servoBoard.servos[4].setGoal(pos);
	}, 2000);
});

servoBoard.on('disconnect', function() {
	console.log('Servo Board:', 'Disconnected!');
});

servoBoard.device = '/dev/ttyACM3';
servoBoard.baudrate = 115200;
servoBoard.connect();
