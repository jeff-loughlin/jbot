#define PI 3.14159265

#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>
#include "pid.h"
#include "filter.h"
#include <pthread.h>
#include "geocoords.h"

//#define _POSIX_SOURCE 1 /* POSIX compliant source */

//#define PI 3.14159265
#define RADTODEG (180 / PI);
#define DEGTORAD (PI / 180)

#define BAUDRATE B115200

#define FULL_SPEED 255
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define SUBSUMPTION_INTERVAL 100
#define LED_BLINK_INTERVAL 1000
#define CALCULATE_GPS_HEADING_INTERVAL 10000

// Sensor values (read from serial link)
// Each sensor value has a corresponing _t that will contain the timestamp for the last update to that value.
// This allows us to detect stale data and ignore it if necessary.
float heading = 0;
unsigned long heading_t;
double latitude = 0;
unsigned long latitude_t;
double longitude = 0;
unsigned long longitude_t;
int distanceLeft = 0;
unsigned long distanceLeft_t;
int distanceCenter = 0;
unsigned long distanceCenter_t;
int distanceRight = 0;
unsigned long distanceRight_t;
int switch1 = 0;
unsigned long switch1_t;
int switch2 = 0;
unsigned long switch2_t;
int magX, magY, magZ;
unsigned long magX_t, magY_t, magZ_t;
double accelX, accelY, accelZ;
unsigned long accelX_t, accelY_t, accelZ_t;
int gyroX, gyroY, gyroZ;
unsigned long gyroX_t, gyroY_t, gyroZ_t;
int gyroDeltaT;
unsigned long gyroDeltaT_t;

// Actuator values (sent to serial link)
int leftPower;
int rightPower;

// File descriptor for serial device
int fdSerial = 0;

// File descriptor for the manual control FIFO
int fdFifo = 0;

// Magnetometer calibration values.
int xMax, yMax, zMax, xMin, yMin, zMin;

// Gyro calibration values
int gyroXCal = 0, gyroYCal = 0, gyroZCal = 0;

// Accelerometer calibration values
int accelXCal = 0, accelYCal = 0, accelZCal = 980;

// Subsumption task control class
class ControlMode
{
public:
    bool active;
    int leftMotorPower;
    int rightMotorPower;

    ControlMode() {active = false; leftMotorPower = 0; rightMotorPower = 0; };
};


// Filters for smoothing magnetometerm, accelerometer, and calculated heading data
Filter headingFilter(0.125, 4, 1, 0);
Filter xFilter(0.125, 4, 1, 0);
Filter yFilter(0.125, 4, 1, 0);
Filter zFilter(0.125, 4, 1, 0);

Filter xAccFilter(0.125, 4, 1, 0);
Filter yAccFilter(0.125, 4, 1, 0);
Filter zAccFilter(0.125, 4, 1, 0);



// PID controller variables for heading
double SetHeading, headingPIDInput, headingPIDOutput;
PID headingPID(&headingPIDInput, &headingPIDOutput, &SetHeading,1,0,0, DIRECT);

// PID controller variables for wall follower (distance from wall)
double SetDistance, wallPIDInput, wallPIDOutput;
PID wallFollowerPID(&wallPIDInput, &wallPIDOutput, &SetDistance, 5, .1, .5, DIRECT);


// Constants for the pan/tilt camera servos
#define PAN_SERVO 1
#define TILT_SERVO 2
#define PAN_MIN 0
#define PAN_MAX 180
#define TILT_MIN 20
#define TILT_MAX 120
#define PAN_MID 90
#define TILT_MID 90

// Servo positions for pan/tilt camera servos
int panServo = PAN_MID;   // defaults to mid point
int tiltServo = TILT_MID;  // defaults to mid point


// Manual control settings - these get set by manual commands coming in through the FIFO.  Start them with sensible defaults.
int manualPowerLeft = 0;
int manualPowerRight = 0;
int manualServoPan = PAN_MID;
int manualServoTilt = TILT_MID;

// Autonomous control or manual control?  This gets set to false when a manual command comes in through the FIFO
bool autonomous = true;



// Quick and dirty function to get elapsed time in milliseconds.  This will wrap at 32 bits (unsigned long), so
// it's not an absolute time-since-boot indication.  It is useful for measuring short time intervals in constructs such
// as 'if (lastMillis - millis() > 1000)'.  Just watch out for the wrapping issue, which will happen every 4,294,967,295
// milliseconds - unless you account for this, I don't recommend using this function for anything that will cause death or
// disembowelment when it suddenly wraps around to zero (e.g. avionics control on an aircraft)...
unsigned long millis()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	unsigned long count = tv.tv_sec * 1000000 + tv.tv_usec;
	return count / 1000;
}

// Parse a message from the serial link or FIFO.  Message format is generally:  msgType:val1|,val2,val3...|
// This will parse the command string into msgType and value (or a comma separated list of values).  It is
// up to the caller to interpret the values (int, float, list of ints, etc)
void ParseMessage(const char *msg, char *msgType, char *val)
{
    int c = 0;
    while (msg[c] != ':' && msg[c] != 0)
    {
        msgType[c] = msg[c];
        c++;
    }
    msgType[c] = '\0';

    int idx = 0;
    c++;
    while (msg[c] != 0x0d && msg[c] != 0x0a && msg[c] != 0x00)
    {
        val[idx] = msg[c];
        c++;
        idx++;
    }
    val[idx] = '\0';
}

// Process a message received over the serial link
void ProcessSerialMsg(const char *msg)
{
    char msgType[8];
    char val[256];

    ParseMessage(msg, msgType, val);

    if (!strcmp(msgType, "LA"))
	{
		latitude = strtod(val, 0);
		latitude_t = millis();
	}
    else if (!strcmp(msgType, "LO"))
	{
		longitude = strtod(val, 0);
		longitude_t = millis();
	}
    else if (!strcmp(msgType, "DL"))
	{
		distanceLeft = strtol(val, 0, 10);
		distanceLeft_t = millis();
	}
    else if (!strcmp(msgType, "DC"))
	{
		distanceCenter = strtol(val, 0, 10);
		distanceCenter_t = millis();
	}
    else if (!strcmp(msgType, "DR"))
	{
		distanceRight = strtol(val, 0, 10);
		distanceRight_t = millis();
	}
	else if (!strcmp(msgType, "MX"))
	{
	    magX = strtol(val, 0, 10);
	    magX_t = millis();
	}
	else if (!strcmp(msgType, "MY"))
	{
	    magY = strtol(val, 0, 10);
	    magY_t = millis();
	}
	else if (!strcmp(msgType, "MZ"))
	{
	    magZ = strtol(val, 0, 10);
	    magZ_t = millis();
	}
	else if (!strcmp(msgType, "AX"))
	{
	    accelX = (float)(strtol(val, 0, 10));
	    accelX_t = millis();

        // Invert the X axis because it is mounted upside-down
        accelX = -accelX;

        // Apply the calibration offset to correct for constant offset
        accelX -= accelXCal;
	}
	else if (!strcmp(msgType, "AY"))
	{
	    accelY = (float)(strtol(val, 0, 10));
	    accelY_t = millis();

        // Invert the Y axis becasue it is mounted upside-down
        accelY = -accelY;

        // Apply the calibration offset to correct for constant offset
        accelY -= accelYCal;
	}
	else if (!strcmp(msgType, "AZ"))
	{
	    accelZ = (float)(strtol(val, 0, 10));
	    accelZ_t = millis();

        // Invert the Z axis because it is mounted upside-down
        accelZ = -accelZ;

        // Apply the calibration offset to correct for constant offset.  (NOTE: Level setting for ZAxis is 980)
        accelZ -= (accelZCal - 980);
	}
	else if (!strcmp(msgType, "GX"))
	{
	    gyroX = strtol(val, 0, 10);
	    gyroX_t = millis();

        gyroX -= gyroXCal;
	}
	else if (!strcmp(msgType, "GY"))
	{
	    gyroY = strtol(val, 0, 10);
	    gyroY_t = millis();

        gyroY -= gyroYCal;
	}
	else if (!strcmp(msgType, "GZ"))
	{
	    gyroZ = strtol(val, 0, 10);
	    gyroZ_t = millis();

        gyroZ -= gyroZCal;
	}
	else if (!strcmp(msgType, "GDT"))
	{
	    gyroDeltaT = strtol(val, 0, 10);
	    gyroDeltaT_t = millis();
	}
    else if (!strcmp(msgType, "S1"))
	{
		switch1 = strtol(val, 0, 10);
		switch1_t = millis();
	}
    else if (!strcmp(msgType, "S2"))
	{
		switch2 = strtol(val, 0, 10);
		switch2_t = millis();
	}
}




float rollAngle = 0;
float pitchAngle = 0;
float yawAngle = 0;
//float CFAngleX1 = 0;
//float CFAngleY1 = 0;



float CFAngleX2 = 0;
float CFAngleY2 = 0;

// Calculate compass heading from the raw X, Y, Z magnetometer and accelerometer data received over the serial link.
// This assumes the magnetometer has been calibrated by sending a CAL command to the sensor module first, which will
// cause it to send back max and min values for each of its X, Y, and Z coordinates.
float getHeading()
{
    if (xMax == xMin || yMax == yMin || zMax == zMin)
        return 0.0F;

    // Normalize raw magnetometer data to a unit circle with the calibraton limits
    float cx = (float)(magX-xMin)/(float)(xMax-xMin) - 0.5;
    float cy = (float)(magY-yMin)/(float)(yMax-yMin) - 0.5;
    float cz = (float)(magZ-zMin)/(float)(zMax-zMin) - 0.5;


    // Convert accelerometer data from mm/s^2 to g
    float rx = -(float)accelX / 980;
    float ry = -(float)accelY / 980;
    float rz = -(float)accelZ / 980;


    // Calculate roll and pitch angles from the accelerometer readings
    float rollAngle = atan2(ry, rz);
    float pitchAngle = atan(-rx / ((ry * sin(rollAngle)) + (rz * cos(rollAngle))));
    printf("RollAngle: %f\n",(rollAngle + PI) * 180 / 3.14);
    printf("PitchAngle: %f\n", -pitchAngle * 180 / 3.14);


    // Then factor in the gyros to smooth out any short-term movements
//    float gx = gyroX * 0.00875;
//    float gy = gyroY * 0.00875;
//    float gz = gyroZ * 0.00875;

    CFAngleX2 = rollAngle; //0.95 * (CFAngleX2 + (gx * 3.14 / 180) * ((float)gyroDeltaT / 1000.0)) + 0.05 * rollAngle;
    CFAngleY2 = pitchAngle; //0.95 * (CFAngleY2 + (gy * 3.14 / 180) * ((float)gyroDeltaT / 1000.0)) + 0.05 * pitchAngle;

    // Now use trigonometry to project the magnetic unit circle onto a 2D X-Y plane oriented at the calculated angles.
    // X and Y will be the point on the unit circle that represents our compass heading.
    float y = (cz * sin(CFAngleX2)) - (cy * cos(CFAngleX2));
    float x = (cx * cos(CFAngleY2))
                     + (cy * sin(CFAngleY2) * sin(CFAngleX2))
                     + (cz * sin(CFAngleY2) * cos(CFAngleX2));


    // Find the compass heading for the calculated x and y
    float yawAngle = atan2(y, x);

    // Factor in the declination angle for this location  (from http://www.ngdc.noaa.gov/geomag-web/#declination)
    float declinationAngle = 0.20780084;  // should this be negative?
    yawAngle += declinationAngle;

    // Convert to degrees
    yawAngle = yawAngle * RADTODEG; //180 / PI;

    // Convert to range (0, 360)
    yawAngle = (yawAngle > 0.0 ? yawAngle : (360.0 + yawAngle));

    return fabs(yawAngle - 360);
}


// Initialization stuff - open and configure the serial device, etc.
void Setup(int *fdSerial, const char *serialDev)
{
    struct termios oldtio, newtio;

    //   Open serial device for reading and writing and not as controlling tty
    //   because we don't want to get killed if linenoise sends CTRL-C.
    printf("Opening serial device\n");
    *fdSerial = open(serialDev, O_RDWR | O_NOCTTY );
    if (*fdSerial < 0)
    {
		printf("Failed to open tty device\n");
        perror(serialDev);
        exit(-1);
    }

	printf("Setting serial device attributes\n");

    tcgetattr(*fdSerial, &oldtio); /* save current serial port settings */
	memset(&newtio, 0, sizeof(newtio));
    /*
          BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
          CRTSCTS : output hardware flow control (only used if the cable has
                    all necessary lines. See sect. 7 of Serial-HOWTO)
          CS8     : 8n1 (8bit,no parity,1 stopbit)
          CLOCAL  : local connection, no modem contol
          CREAD   : enable receiving characters
    */
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

    /*
          IGNPAR  : ignore bytes with parity errors
          ICRNL   : map CR to NL (otherwise a CR input on the other computer
                    will not terminate input)
          otherwise make device raw (no other input processing)
    */
    newtio.c_iflag = IGNPAR | ICRNL;

    /*
         Raw output.
    */
    newtio.c_oflag = 0;

    /*
          ICANON  : enable canonical input
          disable all echo functionality, and don't send signals to calling program
    */
    newtio.c_lflag = ICANON;

    /*
          initialize all control characters
          default values can be found in /usr/include/termios.h, and are given
          in the comments, but we don't need them here
    */
    newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */
    newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
    newtio.c_cc[VERASE]   = 0;     /* del */
    newtio.c_cc[VKILL]    = 0;     /* @ */
    newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
    newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
    newtio.c_cc[VSWTC]    = 0;     /* '\0' */
    newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */
    newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
    newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
    newtio.c_cc[VEOL]     = 0;     /* '\0' */
    newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
    newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
    newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
    newtio.c_cc[VEOL2]    = 0;     /* '\0' */

    /*
          now clean the modem line and activate the settings for the port
    */
    tcflush(*fdSerial, TCIFLUSH);
    tcsetattr(*fdSerial,TCSANOW,&newtio);



    // Set up the PID controllers for heading and wall following
//    SetHeading = 137.0;
    headingPIDInput = 0;
    headingPID.SetOutputLimits(-FULL_SPEED + 2, FULL_SPEED-2);
    headingPID.SetMode(AUTOMATIC);

    SetDistance = 40.0;
    wallFollowerPID.SetOutputLimits(-FULL_SPEED + 2, FULL_SPEED-2);
    wallFollowerPID.SetMode(AUTOMATIC);
}


// Send a command to the motor controller to set the speed of a motor
void SetMotorSpeed(int motor, int speed)
{
    // Make sure we stay in the range -255 < speed < 255
    if (speed < -FULL_SPEED)
        speed = -FULL_SPEED;
    if (speed > FULL_SPEED)
        speed = FULL_SPEED;

    char outMsg[256];
    switch (motor)
    {
        case LEFT_MOTOR:    sprintf(outMsg, "LM:%d\r", speed);
                            printf("Setting Left motor speed to \033[1m%d\033[0m\n", speed);
                            break;
        case RIGHT_MOTOR:   sprintf(outMsg, "RM:%d\r", speed);
                            printf("Setting Right motor speed to \033[1m%d\033[0m\n", speed);
                            break;
    }
    write(fdSerial, outMsg, strlen(outMsg));
}

// Send a command to the servo controller to set the angle of a servo
void SetServoAngle(int servo, int angle)
{
    // Make sure we stay in the range 0 < angle < 180
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    char outMsg[256];
    switch (servo)
    {
        case PAN_SERVO:     sprintf(outMsg, "S2:%d\r", angle);
                            printf("Setting Pan servo to %d\n", angle);
                            break;
        case TILT_SERVO:    sprintf(outMsg, "S3:%d\r", angle);
                            printf("Setting Tilt servo to %d\n", angle);
                            break;
    }
    write(fdSerial, outMsg, strlen(outMsg));
}


// Steer to heading subsumption task.  If active and not subsumed by a higher priority task, this will set the motor speeds
// to steer to the given heading (SetHeading)
void SteerToHeading(ControlMode *steerToHeadingControl)
{
    // Filter the mag data to eliminate noise
    xFilter.update(magX);
    magX = xFilter.GetValue();
    yFilter.update(magY);
    magY = yFilter.GetValue();
    zFilter.update(magZ);
    magZ = zFilter.GetValue();

    // Do the same with the accelerometer data
    xAccFilter.update(accelX);
    accelX = xAccFilter.GetValue();
    yAccFilter.update(accelY);
    accelY = yAccFilter.GetValue();
    zAccFilter.update(accelZ);
    accelZ = zAccFilter.GetValue();

    float heading = getHeading();
    printf("Heading:         %f (default calibration)\n", heading);

    float filteredHeading = heading;

//   if (SetHeading - filteredHeading > 180)
//       filteredHeading += 360;
//   else if (SetHeading - filteredHeading < -180)
//       filteredHeading -= 360;


    headingFilter.update(filteredHeading);
    filteredHeading = headingFilter.GetValue();
    headingPIDInput = filteredHeading;
    headingPID.Compute();

    printf("Raw Heading:         %f\n", heading);
    printf("\033[1mSet Heading:         %f\033[0m \n", SetHeading);
    printf("\033[1mFiltered Heading:    %f\033[0m \n", filteredHeading);
    printf("PID error:           %f\n", headingPIDOutput);
    steerToHeadingControl->leftMotorPower = FULL_SPEED + headingPIDOutput;
    steerToHeadingControl->rightMotorPower = FULL_SPEED - headingPIDOutput;
    steerToHeadingControl->active = true;
}


// Wall follower subsumption task.  If active and not subsumed by a higher priority task, this will set the motor speeds
// to follow a wall at distance WALL_FOLLOWER_DISTANCE
void WallFollower(ControlMode *wallFollowerControl)
{
    // Turn distanceServo to 45 degrees...
    // distanceServo.SetAngle(45); // or something like that
    wallPIDInput = distanceLeft;
    wallFollowerPID.Compute();
    wallFollowerControl->leftMotorPower = FULL_SPEED + wallPIDOutput;
    wallFollowerControl->rightMotorPower = FULL_SPEED - wallPIDOutput;
}


// Detect obstacles subsumption task.  If an onstacle is detected, set active flag to subsume all other tasks.  This
// will generally be the highest priority task (except for manual control), since we always want to avoid obstacles
// regardless of what other tasks are active.
void DetectObstacles(ControlMode *detectObstaclesControl)
{
    int distance = distanceCenter;
    if (distance > 4 && distance < 40 ) // cm
    {
        detectObstaclesControl->active = true;
    }
    else
        detectObstaclesControl->active = false;
}


void CalculateHeadingToWaypoint()
{
    // Get current waypoint.  TODO: change this from hard-coded values to something useful
    float wpLatitude = 39.972090;
    float wpLongitude = -75.735969;

    GeoCoordinate current(latitude, longitude);
    GeoCoordinate waypoint(wpLatitude, wpLongitude);

    // SetHeading is the vlaue used by the heading PID controller.  By changing this, we change the heading
    // to which the SteerToHeading subsumption task will try to steer us.
    SetHeading = getBearing(current, waypoint);

    return;
}

void CalibrateSensors()
{
        printf("\nEntering calibration mode\n");
        printf("Place on a flat level surface\n\n");

        // Now calibrate the gyros and accelerometers
        int xg = 0;
        int yg = 0;
        int zg = 0;
        int ax = 0;
        int ay = 0;
        int az = 0;
        gyroXCal = 0;
        gyroYCal = 0;
        gyroZCal = 0;
        accelXCal = 0;
        accelYCal = 0;
        accelZCal = 980;
        for (int i = 0; i < 100; i++)
        {
            xg += gyroX;
            yg += gyroY;
            zg += gyroZ;
            ax += accelX;
            ay += accelY;
            az += accelZ;
            usleep(100000);
        }

        gyroXCal = xg / 100;
        gyroYCal = yg / 100;
        gyroZCal = zg / 100;

#ifdef JUNK_FOR_NOW  // Commenting out for now so can calibrate other sensors without messing up the mag calibration
        accelXCal = ax / 100;
        accelYCal = ay / 100;
        accelZCal = az / 100;

        printf("Rotate through all axes X-Y-Z\n");
        xMin = 9999;
        xMax = -9999;
        yMin = 9999;
        yMax = -9999;
        zMin = 9999;
        zMax = -9999;
        for (int i = 0; i < 100; i++)
        {
            if (magX > xMax)
	            xMax = magX;
	        if (magX < xMin)
	            xMin = magX;
            if (magY > yMax)
                yMax = magY;
            if (magY < yMin)
                yMin = magY;
            if (magZ > zMax)
                zMax = magZ;
            if (magZ < zMin)
                zMin = magZ;
            usleep(100000);
        }
#endif

        printf("Done calibration sequence.  Writing to disk.\n");
        FILE *f = fopen("./.calibration", "w");
        fprintf(f, "AX:%d\n",accelXCal);
        fprintf(f, "AY:%d\n",accelYCal);
        fprintf(f, "AZ:%d\n",accelZCal);
        fprintf(f, "GX:%d\n",gyroXCal);
        fprintf(f, "GY:%d\n",gyroYCal);
        fprintf(f, "GZ:%d\n",gyroZCal);
        fprintf(f, "MXMin:%d\n",xMin);
        fprintf(f, "MXMax:%d\n",xMax);
        fprintf(f, "MYMin:%d\n",yMin);
        fprintf(f, "MYMax:%d\n",yMax);
        fprintf(f, "MZMin:%d\n",zMin);
        fprintf(f, "MZMax:%d\n",zMax);
        fclose(f);
}

// Manual control subsumption task.  If a command comes in over the FIFO, it will override all other tasks.
void ManualControl(ControlMode *manualControl)
{
    // Read from FIFO (non-blocking)
    // If command present, set active flag for manualControl and use it.  Active flag remains in effect
    // until "A" command comes in to deactivate it.
    //
    // Valid commands are:
    // M:left,right          - Motor power (left = 0-255, right = 0-255)
    // PT:panAngle,tiltAngle - Set angle for pan and tilt servos
    // A:0                   - return to autonomous mode

    char buf[256];
    int res = read(fdFifo, buf, 255);
    if (res <= 0)
    {
        manualControl->active = !autonomous;
        manualControl->leftMotorPower = manualPowerLeft;
        manualControl->rightMotorPower = manualPowerRight;
        return;
    }

    char msgType[8];
    char val[256];
    ParseMessage(buf, msgType, val);

    if (!strcmp(msgType, "M"))
    {
        char *leftVal = strtok(val, ",");
        char *rightVal = strtok(0, ",");
        int left = strtol(leftVal, 0, 10);
        int right = strtol(rightVal, 0, 10);
        manualPowerLeft = left;
        manualPowerRight = right;
        panServo = manualServoPan + ((manualPowerLeft - manualPowerRight) * 0.10);
        if (panServo < PAN_MIN)
            panServo = PAN_MIN;
        if (panServo > PAN_MAX)
            panServo = PAN_MAX;
        autonomous = false;
    }
    else if (!strcmp(msgType, "A"))
    {
        manualPowerLeft = 0;
        manualPowerRight = 0;
        autonomous = true;
    }
    else if (!strcmp(msgType, "PT"))
    {
        char *panVal = strtok(val, ",");
        char *tiltVal = strtok(0, ",");
        panServo = strtol(panVal, 0, 10);
        if (panServo < PAN_MIN)
            panServo = PAN_MIN;
        if (panServo > PAN_MAX)
            panServo = PAN_MAX;

        tiltServo = 180 - strtol(tiltVal, 0, 10);
        if (tiltServo < TILT_MIN)
            tiltServo = TILT_MIN;
        if (tiltServo > TILT_MAX)
            tiltServo = TILT_MAX;

        manualServoPan = panServo;
        manualServoTilt = tiltServo;
        SetServoAngle(PAN_SERVO, panServo);
        SetServoAngle(TILT_SERVO, tiltServo);
    }
    else if (!strcmp(msgType, "H"))
    {
        SetHeading = strtof(val, 0);
        autonomous = true;
    }
    else if (!strcmp(msgType, "CAL"))
    {
        CalibrateSensors();
    }
    manualControl->active = !autonomous;
    manualControl->leftMotorPower = manualPowerLeft;
    manualControl->rightMotorPower = manualPowerRight;
}


// Called every SUBSUMPTION_INTERVAL milliseconds, this will run each of the subsumption tasks in succession and then
// use the one with the highest priority to control the motors.
void ProcessSubsumptionTasks()
{
    ControlMode steerToHeadingControl;
    ControlMode wallFollowerControl;
    ControlMode detectObstaclesControl;
    ControlMode manualControl;

    int leftMotorPower;
    int rightMotorPower;

    // Run each subsumption task.
    DetectObstacles(&detectObstaclesControl);
    SteerToHeading(&steerToHeadingControl);
    WallFollower(&wallFollowerControl);
    ManualControl(&manualControl);


    // Go through the tasks from lowest to highest priority.  If a task's active flag is set, set the motor speeds
    // to that task's calculated values.  At the end, we'll have the motor speeds for the highest priority task in
    // leftMotorPower and rightMotorPower - these are the values that will get sent to the motor controller.
    leftMotorPower = FULL_SPEED;
    rightMotorPower = FULL_SPEED;
    if (steerToHeadingControl.active)
    {
        leftMotorPower = steerToHeadingControl.leftMotorPower;
        rightMotorPower = steerToHeadingControl.rightMotorPower;
        printf("SteerToHeadingControl Active\n");
    }
    if (wallFollowerControl.active)
    {
        // See if we're heading back in the desired direction.  If we are, we've followed the wall
        // or obstacle around to a point where we can now continue on our desired course.
        if (fabs(SetHeading - heading) < 10)
        {
            wallFollowerControl.active = false;
        }
        else
        {
            // Otherwise, keep following the wall or obstacle
            leftMotorPower = wallFollowerControl.leftMotorPower;
            rightMotorPower = wallFollowerControl.rightMotorPower;
            printf("WallFollowerControl Active\n");
        }
    }
    if (detectObstaclesControl.active)
    {
        printf("Obstacle detected.  Turning and activating Wall Follower mode\n");
        SetMotorSpeed(LEFT_MOTOR, FULL_SPEED);
        SetMotorSpeed(RIGHT_MOTOR, -FULL_SPEED);
        sleep(2);
        leftMotorPower = FULL_SPEED;
        rightMotorPower = FULL_SPEED;
        wallFollowerControl.active = true;
    }
    if (manualControl.active)
    {
        leftMotorPower = manualControl.leftMotorPower;
        rightMotorPower = manualControl.rightMotorPower;
        printf("ManualControl Active\n");
    }
    if (0)
    {
        // Insert higher priority motor controls here
    }
    SetMotorSpeed(LEFT_MOTOR, leftMotorPower);
    SetMotorSpeed(RIGHT_MOTOR, rightMotorPower);
}


// Read from the serial link.  This runs in its own thread so it will continue regardless of what else is going
// on.  As we read telemetry and other sensor values from the link, update their values for other tasks to see
static void *ReadSerialThread(void *)
{
    int res;
    char buf[255];

    printf("ReadSerial thread started.\n");
    while (1)
    {
        // Read will block until characters are available on the serial line
        res = read(fdSerial, buf, 255);
        if (res > 1)
        {
            buf[res - 1] = '\0';
            ProcessSerialMsg(buf);
        }
    }
    return NULL;
}


void ReadCalibrationFile()
{
    FILE *f = fopen("./.calibration", "r");
    if (f == NULL)
        return;

    while (!feof(f))
    {
        char line[256];
        fgets(line, 256, f);
        char *t = strtok(line, ":");
        char *v = strtok(NULL, ":");
        if (t == NULL || v == NULL || strlen(t) == 0 || strlen(v) == 0)
            break;
        if (!strcmp(t, "AX"))
        {
            accelXCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "AY"))
        {
            accelYCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "AZ"))
        {
            accelZCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "GX"))
        {
            gyroXCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "GY"))
        {
            gyroYCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "GZ"))
        {
            gyroZCal = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MXMin"))
        {
            xMin = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MXMax"))
        {
            xMax = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MYMin"))
        {
            yMin = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MYMax"))
        {
            yMax = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MZMin"))
        {
            zMin = strtol(v, 0, 10);
        }
        if (!strcmp(t, "MZMax"))
        {
            zMax = strtol(v, 0, 10);
        }
    }
    fclose(f);
}


int calFlag = 0;
int waypointFlag = 1;  // default
int headingFlag = 0;
int led = 0;
int main(int argc, char **argv)
{
    char serialDev[256];
    strcpy(serialDev, DEFAULTDEVICE);

    opterr = 0;
    int c;
    while ((c = getopt (argc, argv, "cwh:d:")) != -1)
    {
        switch (c)
        {
            case 'c':
                calFlag = 1;
                break;
            case 'h':
                headingFlag = 1;
                SetHeading = strtol(optarg, 0, 10);
                waypointFlag = 0;
                break;
            case 'w':
                waypointFlag = 1;
                break;
            case 'd':
                strcpy(serialDev, optarg);
                break;
            case '?':
                if (optopt == 'h')
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
                return 1;
            default:
                abort ();
        }
    }




//    if (argc == 2)
//        strcpy(serialDev, argv[1]);
//    else
//        strcpy(serialDev, DEFAULTDEVICE);


    Setup(&fdSerial, serialDev);
    fdFifo = open("./RobotFifo", O_RDONLY | O_NONBLOCK);

    // Default calibration.  We'll use these until the complex heading calculation routine has been called with enough
    // variation to do a real calibration
    // Commented out.  Read from .calibration file on disk now (ReadCalibrationFile()).
//    xMin = -735;
//    xMax = 420;
//    yMin = -700;
//    yMax = 380;
//    zMin = -450;
//    zMax = 100;
    ReadCalibrationFile();


    printf("Starting ReadSerial thread\n");
    pthread_t threadId;
    pthread_create(&threadId, NULL, ReadSerialThread, NULL);
    sleep(2);

    if (calFlag)
    {
        CalibrateSensors();
    }

    printf("Starting main loop\n");
    unsigned long lastLEDMillis = 0;
    unsigned long lastSubMillis = 0;
    unsigned long lastGPSMillis = 0;

    // The main loop.  Run forever.
    while (1)
    {
        unsigned long loopTime = millis();


        if ((millis() - lastSubMillis > SUBSUMPTION_INTERVAL))
        {
            ProcessSubsumptionTasks();

            printf("Pan Servo: %d\n", panServo);
            printf("Tilt Servo: %d\n", tiltServo);
            printf("MagX: %d\n", magX);
            printf("MagY: %d\n", magY);
            printf("MagZ: %d\n", magZ);
            printf("AccelX: %f\n", accelX);
            printf("AccelY: %f\n", accelY);
            printf("AccelZ: %f\n", accelZ);
            printf("GyroX: %d\n", gyroX);
            printf("GyroY: %d\n", gyroY);
            printf("GyroZ: %d\n", gyroZ);
            printf("gyroDeltaT: %d\n", gyroDeltaT);
            printf("Latitude: %f\n", latitude);
            printf("Longitude: %f\n", longitude);
            printf("LED: %s\n\n", led ? "ON" : "OFF");

            lastSubMillis = millis();
        }

        if (waypointFlag && millis() - lastGPSMillis > CALCULATE_GPS_HEADING_INTERVAL)
        {
            CalculateHeadingToWaypoint();
            lastGPSMillis = millis();
        }

        char msg[256];
        if (led == 0 && millis() - lastLEDMillis > LED_BLINK_INTERVAL)
        {
            sprintf(msg, "LED:1\r");
            write(fdSerial, msg, strlen(msg));
            lastLEDMillis = millis();
	        led = 1;
        }
        if (led == 1 && millis() - lastLEDMillis > LED_BLINK_INTERVAL)
        {
            sprintf(msg, "LED:0\r");
            write(fdSerial, msg, strlen(msg));
            lastLEDMillis = millis();
	        led = 0;
        }
        unsigned long now = millis();
    	if (now - loopTime < 1)
            usleep((1 - (now - loopTime)) * 1000);
    }
}



