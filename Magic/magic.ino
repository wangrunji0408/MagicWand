#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Arduino.h"

typedef uint8_t pin_t;

class LEDBar
{
	pin_t pin;

  public:
	LEDBar(pin_t pin) : pin(pin)
	{
		pinMode(pin, OUTPUT);
	}
	void setValue(int value)
	{
		// assert(value >= 0 && value <= 10);
		analogWrite(pin, 255 / 10 * value);
	}
};

class Button
{
	pin_t pin;

  public:
	Button(pin_t pin) : pin(pin)
	{
		pinMode(pin, INPUT);
	}
	bool isPressed() const
	{
		return digitalRead(pin) == HIGH;
	}
};

class LED
{
	pin_t pin;

  public:
	LED(pin_t pin) : pin(pin)
	{
		pinMode(pin, OUTPUT);
	}
	void set(bool x = HIGH)
	{
		digitalWrite(pin, x);
	}
	void reset()
	{
		digitalWrite(pin, LOW);
	}
};

class MP3Player
{
	const int PAUSE = 0xFFF1;
	const int PLAY = 0xFFF0;
	const int NEXT = 0xFFF4;

	pin_t pin;
	byte _crol_(byte data, int num)
	{
		byte d;
		d = (data << num) | (data >> (8 - num));
		return d;
	}
	void write_command(int COM_data)
	{
		byte high_data; //高八位
		byte low_data;  //低八位
		low_data = COM_data & 0x00ff;
		high_data = (COM_data >> 8) & 0x00ff;
		digitalWrite(pin, LOW);
		delay(5);					//拉低数据线 5 毫秒
		for (int i = 0; i < 8; i++) //发送高八位
		{
			if (high_data & 0x80) //数据位为 1
			{
				digitalWrite(pin, HIGH);
				delayMicroseconds(600); //延时 600us
				digitalWrite(pin, LOW);
				delayMicroseconds(200); //延时 200us
			}
			else
			{
				digitalWrite(pin, HIGH);
				delayMicroseconds(200); //延时 200us
				digitalWrite(pin, LOW);
				delayMicroseconds(600); //延时 600us
			}
			high_data = _crol_(high_data, 1); //循环左移一位
		}
		for (int i = 0; i < 8; i++) // 发送低八位
		{
			if (low_data & 0x80) //数据位为 1
			{
				digitalWrite(pin, HIGH);
				delayMicroseconds(600); //延时 600us
				digitalWrite(pin, LOW);
				delayMicroseconds(200); //延时 200us
			}
			else
			{
				digitalWrite(pin, HIGH);
				delayMicroseconds(200); //延时 200us
				digitalWrite(pin, LOW);
				delayMicroseconds(600); //延时 600us
			}
			low_data = _crol_(low_data, 1);
		}
		digitalWrite(pin, HIGH);
		delay(1);
	}

  public:
	MP3Player(pin_t pin) : pin(pin)
	{
		pinMode(pin, OUTPUT);
		write_command(0xFFE7);
		write_command(0xFFF3);
	}
	void pause()
	{
		write_command(PAUSE);
	}
	void play()
	{
		write_command(PLAY);
	}
	void next()
	{
		write_command(NEXT);
	}
	void begin(int id)
	{
		write_command(id);
	}
};

class Gyro
{
	MPU6050 mpu;

	// const int GYRO_OFFSET[3] = {-803, 0, 280};
	// const int ACCEL_OFFSET[3] = {-6350, 280, 3114};
	const int GYRO_OFFSET[3] = {220, 76, -85};
	const int ACCEL_OFFSET[3] = {0, 0, 1788};

	// MPU control/status vars
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;		// count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

  public:
	// orientation/motion vars
	Quaternion q;		 // [w, x, y, z]         quaternion container
	VectorInt16 gyro;
	VectorInt16 aa;		 // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity; // [x, y, z]            gravity vector
	float euler[3];		 // [psi, theta, phi]    Euler angle container
	float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	float yrp[3];		 // [yaw, roll, pitch]

	uint8_t dmpGetYawRollPitch(float *data, Quaternion *q, VectorFloat *gravity) {
		// yaw: (about Z axis)
		data[0] = atan2(2*q->x*q->y - 2*q->w*q->z, 2*q->w*q->w + 2*q->y*q->y - 1);
		// roll: (tilt left/right, about X axis)
		data[1] = atan(gravity->y / sqrt(gravity->x*gravity->x + gravity->z*gravity->z));
		// pitch: (nose up/down, about Y axis)
		data[2] = atan(gravity->x / sqrt(gravity->y*gravity->y + gravity->z*gravity->z));
		return 0;
	}

	Gyro()
	{
		// initialize device
		Serial.println(F("Initializing I2C devices..."));
		mpu.initialize();

		// verify connection
		Serial.println(F("Testing device connections..."));
		Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

		// wait for ready
		// Serial.println(F("\nSend any character to begin DMP programming and demo: "));
		// while (Serial.available() && Serial.read())
		// 	; // empty buffer
		// while (!Serial.available())
		// 	; // wait for data
		// while (Serial.available() && Serial.read())
		// 	; // empty buffer again

		// load and configure the DMP
		Serial.println(F("Initializing DMP..."));
		devStatus = mpu.dmpInitialize();

		// supply your own gyro offsets here, scaled for min sensitivity
		mpu.setXGyroOffset(GYRO_OFFSET[0]);
		mpu.setYGyroOffset(GYRO_OFFSET[1]);
		mpu.setZGyroOffset(GYRO_OFFSET[2]);
		// mpu.setXAccelOffset(ACCEL_OFFSET[0]); // don't!!
		// mpu.setYAccelOffset(ACCEL_OFFSET[1]); // don't!!
		mpu.setZAccelOffset(ACCEL_OFFSET[2]);

		// make sure it worked (returns 0 if so)
		if (devStatus == 0)
		{
			// turn on the DMP, now that it's ready
			Serial.println(F("Enabling DMP..."));
			mpu.setDMPEnabled(true);

			// enable Arduino interrupt detection
			Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
			// attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
			mpuIntStatus = mpu.getIntStatus();

			// set our DMP Ready flag so the main loop() function knows it's okay to use it
			Serial.println(F("DMP ready! Waiting for first interrupt..."));
			dmpReady = true;

			// get expected DMP packet size for later comparison
			packetSize = mpu.dmpGetFIFOPacketSize();
		}
		else
		{
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
			Serial.print(F("DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.println(F(")"));
		}
	}
	void loop()
	{
		// if programming failed, don't try to do anything
		if (!dmpReady)
			return;

		// reset interrupt flag and get INT_STATUS byte
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & 0x10) || fifoCount == 1024)
		{
			// reset so we can continue cleanly
			mpu.resetFIFO();
			Serial.println(F("FIFO overflow!"));

			// otherwise, check for DMP data ready interrupt (this should happen frequently)
		}
		else
		{
			// wait for correct available data length, should be a VERY short wait
			while (fifoCount < packetSize)
				fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);

			// track FIFO count here in case there is > 1 packet available
			// (this lets us immediately read more without waiting for an interrupt)
			fifoCount -= packetSize;
			
			// mpu.dmpGetGyro(&gyro, fifoBuffer);
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			// mpu.dmpGetEuler(euler, &q);
			// mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			// mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			dmpGetYawRollPitch(yrp, &q, &gravity);
			// mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			// mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		}
	}
};

struct Device
{
	LEDBar ledBar = LEDBar(2);
	LED led = LED(3);
	LED status_led = LED(13);
	Button button = Button(8);
	MP3Player mp3Player = MP3Player(A4);
	Gyro gyro = Gyro();	// must 5V

	Device()
	{
	}
	void loop()
	{
		gyro.loop();
		status_led.set((millis() / 500) & 1);
	}
};

class Controller
{
	Device* device;
	String cmd;
	bool last_pressed = false;
	int last_clock = 0;
	#define Serial1 Serial
  public:
	Controller(Device* device) : device(device) {}
	void loop()
	{		
		int clock = millis() / 10;
		if(clock != last_clock) {
			// send yrp
			Serial1.print("yrp ");
			Serial1.print(device->gyro.yrp[0]); Serial1.print(" ");
			Serial1.print(device->gyro.yrp[1]); Serial1.print(" ");
			Serial1.print(device->gyro.yrp[2]); Serial1.print(" ");
			Serial1.println("");
		}
		last_clock = clock;

		// button event
		bool pressed = device->button.isPressed();
		if(last_pressed ^ pressed) {
			Serial1.print("button ");
			Serial1.println(pressed);
		}
		last_pressed = pressed;

		while(Serial1.available()) {
			char c = Serial1.read();
			cmd.concat(c);
			if(c != '\n')
				continue;
			Serial1.print("ack " + cmd);
			if(cmd.startsWith("play")) {
				int id = cmd.charAt(5) - '0';
				device->mp3Player.begin(id);
			} else if (cmd.startsWith("stop")) {
				device->mp3Player.pause();
			} else if (cmd.startsWith("next")) {
				device->mp3Player.next();
			}
			cmd = "";
		}
	}
};

Device* device;
Controller* ctrl;

void setup()
{
	Wire.begin();
	Wire.setClock(400000);
	Serial.begin(9600);
	// Serial1.begin(115200);

	device = new Device();
	ctrl = new Controller(device);
}

void loop()
{
	device->loop();
	ctrl->loop();
	delay(1);
}
