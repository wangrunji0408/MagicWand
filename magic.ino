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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

class Gyro
{
	MPU6050 mpu;

	const int INTERRUPT_PIN = 2;
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
  public:
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
		pinMode(INTERRUPT_PIN, INPUT);

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
			attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

		// wait for MPU interrupt or extra packet(s) available
		// while (!mpuInterrupt && fifoCount < packetSize)
		// {
		// 	// other program behavior stuff here
		// 	// .
		// 	// .
		// 	// .
		// 	// if you are really paranoid you can frequently test in between other
		// 	// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// 	// while() loop to immediately process the MPU data
		// 	// .
		// 	// .
		// 	// .
		// }

		if (!mpuInterrupt && fifoCount < packetSize)
			return;

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
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
		else if (mpuIntStatus & 0x02)
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
const int N = 20;
class RecentStatistic
{
	int value[N] = {0};
	int z = 0;
	public:
	void push(int x)
	{
		value[z++] = x;
		if(z == N)	z = 0;
	}
	int getMean() const
	{
		int s = 0;
		for(int i=0; i<N; ++i)
			s += value[i];
		return s / N;
	}
	int getS() const
	{
		int max = 0, min = 1 << 28;
		for(int i=0; i<N; ++i)
			if(value[i] > max)
				max = value[i];
			else if(value[i] < min)
				min = value[i];
		return max - min;
		float v = 0;
		int mean = getMean();
		for(int i=0; i<N; ++i)
			v += (value[i] - mean) * (value[i] - mean);
		return sqrt(v / N);
	}
};

class Counter
{
	int n, x;
	public:
	Counter(int n): n(n), x(0) { }
	bool next() 
	{
		bool ret = ++x == n;
		if(ret)	x = 0;
		return ret;
	}
};

enum MotionEvent {
	None, Circle, Star, Square, Push
};

class GyroMotionRecognizer
{
	public:
	Gyro *gyro;
	MotionEvent event = None;
	Quaternion lastq, diff;
	int z = 0;
	Counter counter, counter10;
	RecentStatistic rs_a, rs_az, rs_gx;
	bool orientation_moving, moving, pushing;
	int a0;

	GyroMotionRecognizer(Gyro* gyro): gyro(gyro), counter(10), counter10(10) {}
	void loop() 
	{
		if(counter10.next()) 
		{
			rs_az.push(gyro->aaReal.z);
			pushing = rs_az.getS() > 10000;
		}
		if(!counter.next())
			return;
		Quaternion const& q = gyro->q;
		diff = q.getProduct(lastq.getReverse());
		lastq = q;

		a0 = abs(gyro->aaWorld.x) + abs(gyro->aaWorld.y) + abs(gyro->aaWorld.z - 10000);
		rs_a.push(a0);
		rs_gx.push(gyro->gyro.x);
		orientation_moving = rs_gx.getS() > 200;
		moving = rs_a.getS() > 10000;
	}
	MotionEvent getEvent() 
	{
		return event;
	}
};

struct Device
{
	LEDBar ledBar = LEDBar(2);
	LED led = LED(3);
	LED status_led = LED(13);
	Button playButton = Button(8);
	Button pauseButton = Button(9);
	MP3Player mp3Player = MP3Player(A4);
	Gyro gyro = Gyro();	// must 5V
	#define bluetoothSerial Serial1

	Device()
	{
		Wire.begin();
		Wire.setClock(400000);
		Serial.begin(9600);
		bluetoothSerial.begin(9600);
	}
	void loop()
	{
		gyro.loop();
		status_led.set((millis() / 500) & 1);
	}
};

inline int f2i(float x)
{
	return (int)(x * 1000);
}

class Controller
{
	Device* device;
	GyroMotionRecognizer motion;
	Counter counter;
  public:
	Controller(Device* device) : device(device), motion(&device->gyro), counter(1) {}
	void loop()
	{
		motion.loop();
		if(counter.next())
		{
			uint8_t buf[512];
			buf[0] = '$';
			memcpy(buf+4, motion.gyro->yrp, sizeof(float) * 3);
			buf[16] = '\r';
			buf[17] = '\n';
			Serial.write(buf, 18);

			// char str[512];
			// sprintf(str, "%d, %d, %d", f2i(motion.gyro->yrp[0]), f2i(motion.gyro->yrp[1]), f2i(motion.gyro->yrp[2]));			
			
			// sprintf(str, "%d", (int)(acos(motion.diff.w) * 2 * 180 / PI));
			// sprintf(str, "%d, %d, %d, %d", device->gyro.gyro.x, device->gyro.gyro.y, device->gyro.gyro.z, motion.rs_gx.getS());
			// sprintf(str, "%d, %d", motion.gyro->aaReal.z, motion.rs_az.getS());					
			// sprintf(str, "%d, %d, %d", motion.pushing, motion.moving, motion.orientation_moving);			
			// sprintf(str, "%d, %d, %d", motion.rs_a.getS(), motion.rs_a.getMean(), );
			// if(device->playButton.isPressed())
			// 	sprintf(str, "%d, %d, %d", device->gyro.gyro.x, device->gyro.gyro.y, device->gyro.gyro.z);
			// else
			// 	sprintf(str, "%d, %d, %d", device->gyro.aaWorld.x, device->gyro.aaWorld.y, device->gyro.aaWorld.z);			
			// Serial.println(str);
		}

		// device.bluetoothSerial.println(str);
		// switch(motion.getEvent()) 
		// {
		// 	case None:
		// 		Serial.println("None");
		// 		break;
		// 	case Square:
		// 		Serial.println("Square");
		// 		break;
		// 	case Star:
		// 		Serial.println("Star");
		// 		break;
		// 	case Circle:
		// 		Serial.println("Circle");
		// 		break;
		// 	default:
		// 		break;
		// }
		
		// if (device.button.isPressed())
		// 	device.led.set();
		// else
		// 	device.led.reset();
		// int t = millis() / 1000 % 11;
		// device.ledBar.setValue(t);

		// if (device.playButton.isPressed())
		// 	device.mp3Player.play();
		// else if (device.pauseButton.isPressed())
		// 	device.mp3Player.pause();
	}
};

Device* device;
Controller* ctrl;

void setup()
{
	device = new Device();
	ctrl = new Controller(device);
}

void loop()
{
	device->loop();
	ctrl->loop();
	delay(1);
}
