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
#include "PID_v1.h"
#include "kalman.h"
#include <pthread.h>
#include "include.h"
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


// Kalman filters for smoothing magnetometerm, accelerometer, and calculated heading data
Kalman headingFilter(0.125, 4, 1, 0);
Kalman xFilter(0.125, 4, 1, 0);
Kalman yFilter(0.125, 4, 1, 0);
Kalman zFilter(0.125, 4, 1, 0);

Kalman xAccFilter(0.125, 4, 1, 0);
Kalman yAccFilter(0.125, 4, 1, 0);
Kalman zAccFilter(0.125, 4, 1, 0);



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




// extract the NED angles in degrees from the NED rotation matrix (used by getHeadingNew() - remove this if we decide not
// to use that function
void GetAnglesDegFromRotationMatrix(float R[3][3], float *pfPhiDeg, float *pfTheDeg, float *pfPsiDeg, float *pfRhoDeg)
{
    // calculate the pitch angle -90.0 <= Theta <= 90.0 deg
    if (R[0][2] >= 1.0F)
        *pfTheDeg = -90.0F;
    else if (R[0][2] <= -1.0F)
        *pfTheDeg = 90.0F;
    else
        *pfTheDeg = (float) asin(-R[0][2]) * RADTODEG;

    // calculate the roll angle range -180.0 <= Phi < 180.0 deg
    *pfPhiDeg = (float)atan2(R[1][2], R[2][2]) * RADTODEG;

    // map +180 roll onto the functionally equivalent -180 deg roll
    if (*pfPhiDeg == 180.0F)
        *pfPhiDeg = -180.0F;

    // calculate the yaw and compass angle 0.0 <= Psi < 360.0 deg
    *pfPsiDeg = (float)atan2(R[0][1], R[0][0]) * RADTODEG;
    if (*pfPsiDeg < 0.0F)
        *pfPsiDeg += 360.0F;

    // check for rounding errors mapping small negative angle to 360 deg
    if (*pfPsiDeg >= 360.0F)
        *pfPsiDeg = 0.0F;

    // for NED, the compass heading Rho equals the yaw angle Psi
    *pfRhoDeg = *pfPsiDeg;
    return;
}


// Used by getHeadingNew() - remove this if we decide not to use that function
float PerformSensorFusion(float cx, float cy, float cz, float nx, float ny, float nz, float *theta, float *phi, float *psi, float *rho)
{
    float matrix [3][3];

    // place the un-normalized gravity and geomagnetic vectors into the rotation matrix z and x axes
    matrix[0][2] = nx;
    matrix[1][2] = ny;
    matrix[2][2] = nz;
    matrix[0][0] = cx;
    matrix[1][0] = cy;
    matrix[2][0] = cz;

    // set y vector to vector product of z and x and g vectors
    matrix[0][1] = matrix[1][2] * matrix[2][0] - matrix[2][2] * matrix[1][0];
    matrix[1][1] = matrix[2][2] * matrix[0][0] - matrix[0][2] * matrix[2][0];
    matrix[2][1] = matrix[0][2] * matrix[1][0] - matrix[1][2] * matrix[0][0];

    // set x vector to vector product of y and z and g vectos
    matrix[0][0] = matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2];
    matrix[1][0] = matrix[2][1] * matrix[0][2] - matrix[0][1] * matrix[2][2];
    matrix[2][0] = matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2];

    // calculate the vector moduli
//    float fmodB;
    float fmodG, fmodx, fmody;
//    float fsinDelta;

    fmodG =  (float) sqrt(nx * nx + ny * ny + nz * nz);
//    fmodB =  (float) sqrt(cx * cx + cy * cy + cz * cz);
    fmodx = (float) sqrt(matrix[0][0] * matrix[0][0] + matrix[1][0] * matrix[1][0] + matrix[2][0] * matrix[2][0]);
    fmody = (float) sqrt(matrix[0][1] * matrix[0][1] + matrix[1][1] * matrix[1][1] + matrix[2][1] * matrix[2][1]);

    // normalize the rotation matrix
    if (!((fmodx == 0.0F) || (fmody == 0.0F) || (fmodG == 0.0F)))
    {
        // normalize x axis
        matrix[0][0] /= fmodx;
        matrix[1][0] /= fmodx;
        matrix[2][0] /= fmodx;

        // normalize y axis
        matrix[0][1] /= fmody;
        matrix[1][1] /= fmody;
        matrix[2][1] /= fmody;

        // normalize z axis
        matrix[0][2] /= fmodG;
        matrix[1][2] /= fmodG;
        matrix[2][2] /= fmodG;

        // estimate the declination angle Delta (90 minus angle between the vectors)
//        fsinDelta = (nx * cx + ny * cy + nz * cz) / fmodG / fmodB;

//        pthisSV6DOF->fDelta6DOFn = (float) asin(fsinDelta) * RADTODEG;
    }
    else
    {
        // no solution is possible so set rotation to identity matrix and delta to 0 degrees
//        fmatrixAeqI(matrix, 3);
//        pthisSV6DOF->fDelta6DOFn = 0.0F;
        // Can't calculate.  Just return the current heading instead of garbage.  We'll get it right next time.
        return heading;
    }

    // extract the roll, pitch and yaw angles from the rotation matrix
//    float phi, theta, psi, rho;  // phi = roll angle, theta = pitch angle, psi = yaw angle, rho = compass heading
    GetAnglesDegFromRotationMatrix(matrix, phi, theta, psi, rho);

    // get the instantaneous orientation quaternion
//    fQuaternionFromRotationMatrix(matrix, &(pthisSV6DOF->fq6DOFn));

    float compassHeading =  (float)atan2(matrix[0][1], matrix[0][0]) * (180 / PI);
    if (compassHeading < 0.0F)
        compassHeading += 360.0F;

    return compassHeading;
}



void fSixDOFSensorDrivers(struct AccelSensor *thisAccel, struct MagSensor *thisMag)
{
    thisMag->iBpx = magX;
    thisMag->iBpy = magY;
    thisMag->iBpz = magZ;

    thisMag->fBpx = magX / FCOUNTSPERUT;
    thisMag->fBpy = magY / FCOUNTSPERUT;
    thisMag->fBpz = magZ / FCOUNTSPERUT;


    thisAccel->iGpx = accelX;
    thisAccel->iGpy = accelY;
    thisAccel->iGpz = accelZ;

    thisAccel->fGpx = accelX / FCOUNTSPERG;
    thisAccel->fGpy = accelY / FCOUNTSPERG;
    thisAccel->fGpz = accelZ / FCOUNTSPERG;
}



int loopcounter = 0;
struct SV6DOF thisSV6DOF;                   // 6DOF state vector
struct MagCalibration thisMagCal;           // hard and soft iron magnetic calibration
struct MagneticBuffer thisMagneticBuffer;   // magnetometer measurement buffer

// low pass filter parameters
float fb0, fa1, fa2;

int iSolutionSize = 7;

// general purpose arrays
float xftmpA10x10[10][10], *ftmpA10x10[10];             // scratch 10x10 matrix 
float xftmpB10x10[10][10], *ftmpB10x10[10];             // scratch 10x10 matrix 
float xftmpA10x1[10][1], *ftmpA10x1[10];                // scratch 10x1 matrix
float xftmpA7x7[7][7], *ftmpA7x7[7];                    // scratch 7x7 matrix
float xftmpB7x7[7][7], *ftmpB7x7[7];                    // scratch 7x7 matrix 
float xftmpA7x1[7][1], *ftmpA7x1[7];                    // scratch 7x1 matrix 
float xftmpA4x4[4][4], *ftmpA4x4[4];                    // scratch 4x4 matrix
float xftmpB4x4[4][4], *ftmpB4x4[4];                    // scratch 4x4 matrix 
float xftmpA3x3[3][3], *ftmpA3x3[3];                    // scratch 3x3 matrix 
float xftmpA4x1[4][1], *ftmpA4x1[4];                    // scratch 4x1 matrix 
float xftmpB4x1[4][1], *ftmpB4x1[4];                    // scratch 4x1 matrix 
float xftmpA3x1[3][1], *ftmpA3x1[3];                    // scratch 3x1 vector 


float getHeadingNewNew()
{
    struct AccelSensor thisAccel;
    struct MagSensor thisMag;
    fSixDOFSensorDrivers(&thisAccel, &thisMag);

    // update the magnetometer measurement buffer integer magnetometer data
    fUpdateMagnetometerBuffer(&thisMagneticBuffer, &thisMag, &thisAccel, loopcounter);

    // remove hard and soft iron terms from Bp (uT) to get calibrated data Bc (uT)
    fInvertMagCal(&thisMag, &thisMagCal);

    // pass the accel and calibrated mag data to the eCompass
    feCompassDirectNED(&thisSV6DOF, &thisMag, &thisAccel);

    // low pass filter the orientation matrix and get low pass quaternion and Euler angles
    int iCoordSystem = 0;
    fLPFOrientationMatrix(&thisSV6DOF, iCoordSystem, loopcounter, fb0, fa1, fa2);

    // shuffle the rotation matrix low pass filter delay lines
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            thisSV6DOF.fLPR6DOFnm2[i][j] = thisSV6DOF.fLPR6DOFnm1[i][j];
            thisSV6DOF.fLPR6DOFnm1[i][j] = thisSV6DOF.fLPR6DOFn[i][j];
            thisSV6DOF.fR6DOFnm2[i][j] = thisSV6DOF.fR6DOFnm1[i][j];
            thisSV6DOF.fR6DOFnm1[i][j] = thisSV6DOF.fR6DOFn[i][j];
        }

    // update the inclination angle low pass filter delay lines
    thisSV6DOF.fLPDelta6DOFnm2 = thisSV6DOF.fLPDelta6DOFnm1;
    thisSV6DOF.fLPDelta6DOFnm1 = thisSV6DOF.fLPDelta6DOFn;
    thisSV6DOF.fDelta6DOFnm2 = thisSV6DOF.fDelta6DOFnm1;
    thisSV6DOF.fDelta6DOFnm1 = thisSV6DOF.fDelta6DOFn;

    // the following section of code executes the calibration algorithms
    // and decides if the new calibration should be accepted. this code
    // is compute-intensive and is best implemented under an RTOS as a low priority
    // task which might execute every minute or so. here it is implemented
    // in a single process which is a simpler way to demonstrate.

    // decision as to whether to calibrate is the logical OR of these conditions
    // line 1: if we don't yet have a calibration
    // line 2: if we have limited measurements (MEDEQUATIONS or less) and INITIALCALINTERVAL iterations have passed
    // line 3: every FINALCALINTERVAL iterations
    if (thisMagneticBuffer.iMagBufferCount >= MINEQUATIONS)
    {
        // calibrate if this will be the first calibration
        // or initially ever INITIALCALINTERVAL iterations or
        // ultimately every FINALCALINTERVAL iterations
        if ((!thisMagCal.iValidMagCal) ||
            (thisMagCal.iValidMagCal && !(loopcounter % INITIALCALINTERVAL) && (thisMagneticBuffer.iMagBufferCount <= MEDEQUATIONS)) ||
            (thisMagCal.iValidMagCal && !(loopcounter % FINALCALINTERVAL)))
        {
                // 10 point eigenpair calibration
                if (iSolutionSize == 10)
                {
                    printf("\n\nCalling 10 element EIG calibration at iteration %d with %lu in Magnetic Buffer", loopcounter, thisMagneticBuffer.iMagBufferCount);
                    fUpdateCalibration10EIG(&thisMagCal, &thisMagneticBuffer, ftmpA10x10, ftmpB10x10, ftmpA10x1, ftmpA3x3, ftmpA3x1);
                }
                // 7 point eigenpair calibration
                else if (iSolutionSize == 7)
                {
                    printf("\n\nCalling 7 element EIG calibration at iteration %d with %lu in Magnetic Buffer", loopcounter, thisMagneticBuffer.iMagBufferCount);
                    fUpdateCalibration7EIG(&thisMagCal, &thisMagneticBuffer, ftmpA7x7, ftmpB7x7, ftmpA7x1);
                }
                // 4 point INV calibration
                else
                {
//                    printf("\n\nCalling 4 element INV calibration at iteration %d with %d in Magnetic Buffer", loopcounter, thisMagneticBuffer.iMagBufferCount);
//                    fUpdateCalibration4INV(&thisMagCal, &thisMagneticBuffer, ftmpA4x4, ftmpB4x4,
//                                             ftmpA4x1, ftmpB4x1, icolind, irowind, ipivot);
                }
                // for convenience show the original optimal invW
//                printf("\n\nFor comparison: Simulation inverse soft iron matrix invW (normalized)");
//                fmatrixPrintA(thisSimulation.finvW, 0, 2, 0, 2);

                // accept new calibration if:
                // a) number of measurements is MEDEQUATIONS or less or
                // b) fit error is reduced and geomagnetic field is in range
                // (actual range of geomagnetic field strength B on earth varies 22uT to 67uT)
                if ((thisMagneticBuffer.iMagBufferCount <= MEDEQUATIONS) ||
                  (  (thisMagCal.ftrFitErrorpc <= thisMagCal.fFitErrorpc) &&
                    (thisMagCal.ftrB >= MINBFIT) && (thisMagCal.ftrB <= MAXBFIT)) ) // *jdl* added parens around && clause (spaced parens)
                {
                    printf("\n\nAccepting new calibration solution");
                    thisMagCal.fFitErrorpc = thisMagCal.ftrFitErrorpc;
                    thisMagCal.fB = thisMagCal.ftrB;
                    thisMagCal.fVx = thisMagCal.ftrVx;
                    thisMagCal.fVy = thisMagCal.ftrVy;
                    thisMagCal.fVz = thisMagCal.ftrVz;
                    fmatrixAeqB(thisMagCal.finvW, thisMagCal.ftrinvW, 3, 3);
                }
                else
                {
                    printf("\n\nRejecting new calibration solution");
                }

                // age (increase) the calibration fit to avoid a good calibration preventing future updates
                // FITERRORAGING is the reciprocal of the time (s) for the fit error to increase by e=2.718
                // FINALCALINTERVAL * DELTAT is the interval in seconds between each aging update of fFitErrorpc
                // (1 + FITERRORAGING * FINALCALINTERVAL * DELTAT)^n=e defines n, the number of updates for e fold increase
                // approx n * FINALCALINTERVAL * DELTAT = 1. / FITERRORAGING
                // so as required FITERRORAGING is the reciprocal of the time in secs for e fold increase
                thisMagCal.fFitErrorpc += thisMagCal.fFitErrorpc * FITERRORAGING * (float) FINALCALINTERVAL * DELTAT;
         } // end of test whether to call calibration functions
    }
  else // still too few entries in magnetic buffer for calibration
      printf("%lu entries in magnetometer buffer is insufficient for calibration\n", thisMagneticBuffer.iMagBufferCount);

    loopcounter++;
    return thisSV6DOF.fPsi6DOF;
}

float rollAngle = 0;
float pitchAngle = 0;
float yawAngle = 0;
float CFAngleX1 = 0;
float CFAngleY1 = 0;

float getHeadingNew()
{
//    float fmodB;
    float fmodG, fmodx, fmody;
//    float fsinDelta;
    float matrix[3][3];

    // If we don't have calibration data yet then we have no business being here
    if (xMax == xMin || yMax == yMin || zMax == zMin)
        return 0;


    // Normalize raw magnetometer data to a unit circle with the calibraton limits
    float cx = (float)(magX-xMin)/(float)(xMax-xMin) - 0.5;
    float cy = (float)(magY-yMin)/(float)(yMax-yMin) - 0.5;
    float cz = (float)(magZ-zMin)/(float)(zMax-zMin) - 0.5;

    // Convert accelerometer data from m/s^2 to G
    float rx = -(float)accelX;// / 980;  // 9.8
    float ry = -(float)accelY;// / 980;  // 9.8
    float rz = -(float)accelZ;// / 980;  // 9.8

/*
    // Convert gyro data to instantaneous angular measurement
    float gx = (float)gyroX * 0.00875 * (PI / 180);
    float gy = (float)gyroY * 0.00875 * (PI / 180);
    float gz = (float)gyroZ * 0.00875 * (PI / 180);
*/
    float accelTheta, accelPhi, accelPsi, accelRho;
    /*float headingAccel = */PerformSensorFusion(cx, cy, cz, rx, ry, rz, &accelTheta, &accelPhi, &accelPsi, &accelRho);
//    printf("Tilt-compensated heading: %f\n", headingAccel + (0.20780084 * 180 / PI));



    // Now we have in headingAccel the heading calculated from the magnetometer, tilt-compensated based on the orientation
    // reported by the accelerometers.  We'll use this as a long-term indication of heading.  That is, when siting still, this
    // is what we would consider our heading to be.  Now use the gyros to deal with short-term changes in the orientation.
    // If we suddenly pitch upwards or roll left or right, the gyros will tell us by how much our orientation changed over
    // the last 100ms.  Integrate that over time to give us the current heading, offset from the long-term calculation in
    // headingAccel.  This way, when we're sitting still or moving in a straight line, the gyros will read near zero and
    // most of the heading calculation comes from the magnetometer (tilt-compensated by the accelerometer).  When we're
    // turning or bouncing around, the gyros will represent a larger portion of the calculation, and the heading comes
    // mainly from the integration of gyro readings over time.
    //

    // Factor in the gyros to smooth out any short-term movements
    float gx = gyroX * 0.00875;
    float gy = gyroY * 0.00875;
//    float gz = gyroZ * 0.00875;

    printf("accelPhi: %f\n", accelPhi);
    printf("accelTheta: %f\n", accelTheta);

    CFAngleX1 = 0.5 * (CFAngleX1 + (gx * 3.14 / 180) * ((float)gyroDeltaT / 1000.0)) + 0.5 * accelPhi * PI / 180;
    CFAngleY1 = 0.5 * (CFAngleY1 + (gy * 3.14 / 180) * ((float)gyroDeltaT / 1000.0)) + 0.5 * accelTheta * PI / 180;


    // Now use trigonometry to project the magnetic unit circle onto a 2D X-Y plane oriented at the calculated angles.
    // X and Y will be the point on the unit circle that represents our compass heading.
    float y = (cz * sin(CFAngleX1)) - (cy * cos(CFAngleX1));
    float x = (cx * cos(CFAngleY1))
                     + (cy * sin(CFAngleY1) * sin(CFAngleX1))
                     + (cz * sin(CFAngleY1) * cos(CFAngleX1));


    // Find the compass heading for the calculated x and y
    float yawAngle = atan2(y, x);

    // Factor in the declination angle for this location  (from http://www.ngdc.noaa.gov/geomag-web/#declination)
    float declinationAngle = 0.20780084;  // should this be negative?
    yawAngle += declinationAngle;

    // Convert to degrees
    yawAngle = yawAngle * 180 / 3.14;

    // Convert to range (0, 360)
    yawAngle = (yawAngle > 0.0 ? yawAngle : (360.0 + yawAngle));

    return fabs(yawAngle - 360);






/*
    float gyroTheta, gyroPhi, gyroPsi, gyroRho;
    PerformSensorFusion(cx, cy, cz, gx, gy, gz, &gyroTheta, &gyroPhi, &gyroPsi, &gyroRho);

    rollAngle = 0.95 * (rollAngle + gyroPhi * gyroDeltaT / 1000.0F) + (0.05 * accelPhi * PI / 180);
    pitchAngle = 0.95 * (pitchAngle + gyroTheta * gyroDeltaT / 1000.0F) + (0.05 * accelTheta * PI / 180);
    yawAngle = 0.95 * (yawAngle + gyroRho * gyroDeltaT / 1000.0F) + (0.05 * accelRho * PI / 180);

    float headingGyro = yawAngle / 2;

    printf("Heading by accel: %f\n", headingAccel);
    printf("Heading by gyro: %f\n", headingGyro);

    return (int)(headingAccel * 0.05 + headingGyro * 0.95);
*/
    // place the un-normalized gravity and geomagnetic vectors into the rotation matrix z and x axes
    matrix[0][2] = rx;
    matrix[1][2] = ry;
    matrix[2][2] = rz;
    matrix[0][0] = cx;
    matrix[1][0] = cy;
    matrix[2][0] = cz;

    // set y vector to vector product of z and x and g vectors
    matrix[0][1] = matrix[1][2] * matrix[2][0] - matrix[2][2] * matrix[1][0];
    matrix[1][1] = matrix[2][2] * matrix[0][0] - matrix[0][2] * matrix[2][0];
    matrix[2][1] = matrix[0][2] * matrix[1][0] - matrix[1][2] * matrix[0][0];

    // set x vector to vector product of y and z and g vectos
    matrix[0][0] = matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2];
    matrix[1][0] = matrix[2][1] * matrix[0][2] - matrix[0][1] * matrix[2][2];
    matrix[2][0] = matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2];

    // calculate the vector moduli
    fmodG =  (float) sqrt(rx * rx + ry * ry + rz * rz);
//    fmodB =  (float) sqrt(cx * cx + cy * cy + cz * cz);
    fmodx = (float) sqrt(matrix[0][0] * matrix[0][0] + matrix[1][0] * matrix[1][0] + matrix[2][0] * matrix[2][0]);
    fmody = (float) sqrt(matrix[0][1] * matrix[0][1] + matrix[1][1] * matrix[1][1] + matrix[2][1] * matrix[2][1]);

    // normalize the rotation matrix
    if (!((fmodx == 0.0F) || (fmody == 0.0F) || (fmodG == 0.0F)))
    {
        // normalize x axis
        matrix[0][0] /= fmodx;
        matrix[1][0] /= fmodx;
        matrix[2][0] /= fmodx;

        // normalize y axis
        matrix[0][1] /= fmody;
        matrix[1][1] /= fmody;
        matrix[2][1] /= fmody;

        // normalize z axis
        matrix[0][2] /= fmodG;
        matrix[1][2] /= fmodG;
        matrix[2][2] /= fmodG;

        // estimate the declination angle Delta (90 minus angle between the vectors)
//        fsinDelta = (rx * cx + ry * cy + rz * cz) / fmodG / fmodB;

//        pthisSV6DOF->fDelta6DOFn = (float) asin(fsinDelta) * RADTODEG;
    }
    else
    {
        // no solution is possible so set rotation to identity matrix and delta to 0 degrees
//        fmatrixAeqI(matrix, 3);
//        pthisSV6DOF->fDelta6DOFn = 0.0F;
    }

    // extract the roll, pitch and yaw angles from the rotation matrix
//    float phi, theta, psi, rho;  // phi = roll angle, theta = pitch angle, psi = yaw angle, rho = compass heading
//    GetAnglesDegFromRotationMatrix(matrix, &phi, &theta, &psi, &rho);

    // get the instantaneous orientation quaternion
//    fQuaternionFromRotationMatrix(matrix, &(pthisSV6DOF->fq6DOFn));

    float compassHeading =  (float)atan2(matrix[0][1], matrix[0][0]) * (180 / PI);
    if (compassHeading < 0.0F)
        compassHeading += 360.0F;

    return compassHeading;

#ifdef JUNK
    // Now figure in the gyro rotation
    float normValues[3];

    // Calculate the angular speed of the sample
    float omegaMagnitude = (float)sqrt(gyroValues[0] * gyroValues[0] +
                                         gyroValues[1] * gyroValues[1] +
                                         gyroValues[2] * gyroValues[2]);

    // Normalize the rotation vector if it's big enough to get the axis
    if(omegaMagnitude > EPSILON)
    {
        normValues[0] = gyroValues[0] / omegaMagnitude;
        normValues[1] = gyroValues[1] / omegaMagnitude;
        normValues[2] = gyroValues[2] / omegaMagnitude;
    }

    // Integrate around this axis with the angular speed by the timestep
    // in order to get a delta rotation from this sample over the timestep
    // We will convert this axis-angle representation of the delta rotation
    // into a quaternion before turning it into the rotation matrix.
    float dT = (float)gyroDeltaTime / 1000.0;
    float thetaOverTwo = omegaMagnitude * dT;
    float sinThetaOverTwo = sin(thetaOverTwo);
    float cosThetaOverTwo = cos(thetaOverTwo);
    deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
    deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
    deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
    deltaRotationVector[3] = cosThetaOverTwo;
    return (int)rho;
#endif
}



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

    // Collect timing data for each of the 3 heading calculation methods.  Ultimately we'll choose one of these and timing
    // may or may not be a consideration
    unsigned long startTime[3], endTime[3];
    startTime[0] = millis();
    float complexHeading = getHeadingNewNew();
    endTime[0] = millis();
//    startTime[1] = millis();
//    float simpleHeading = getHeadingNew();
//    endTime[1] = millis();
    startTime[2] = millis();
    float reallySimpleHeading = getHeading();
    endTime[2] = millis();
    printf("Heading (complex method - %lums): %f (%scalibrated)\n", endTime[0] - startTime[0], complexHeading, thisMagCal.iValidMagCal ? "" : "not ");
//    printf("Heading (matrix + gyro - %lums):  %f (default calibration)\n", endTime[1] - startTime[1], simpleHeading);
    printf("Heading (simple - %lums):         %f (default calibration)\n", endTime[2] - startTime[2], reallySimpleHeading);

    if (thisMagCal.iValidMagCal)
    {
        printf("Using complex heading\n");
        heading = complexHeading;
    }
    else
    {
//        printf("Using matrix + gyro heading\n");
//        heading = simpleHeading;
        printf("Using simple heading\n");
        heading = reallySimpleHeading;
    }


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


// Initialize various arrays and values used by the matrix math routines for heading calculations.
void InitMagCalibrationData()
{
    // initialize the pointers to arrays as workaround to C function limitations with variable size arrays
    // keep in main since C functions require compile-time knowledge of the number of matrix rows
    // 3 row arrays
    for (int i = 0; i < 3; i++)
    {
        thisMagCal.finvW[i] = thisMagCal.xfinvW[i];
        thisMagCal.ftrinvW[i] = thisMagCal.xftrinvW[i];
        thisMagCal.fA[i] = thisMagCal.xfA[i];
        thisMagCal.finvA[i] = thisMagCal.xinvA[i];
        thisSV6DOF.fR6DOFn[i] = thisSV6DOF.xfR6DOFn[i];
        thisSV6DOF.fR6DOFnm1[i] = thisSV6DOF.xfR6DOFnm1[i];
        thisSV6DOF.fR6DOFnm2[i] = thisSV6DOF.xfR6DOFnm2[i];
        thisSV6DOF.fLPR6DOFn[i] = thisSV6DOF.xfLPR6DOFn[i];
        thisSV6DOF.fLPR6DOFnm1[i] = thisSV6DOF.xfLPR6DOFnm1[i];
        thisSV6DOF.fLPR6DOFnm2[i] = thisSV6DOF.xfLPR6DOFnm2[i];
        ftmpA3x3[i] = xftmpA3x3[i];
        ftmpA3x1[i] = xftmpA3x1[i];
    }
    // 4 row arrays
    for (int i = 0; i < 4; i++)
    {
        ftmpA4x4[i] = xftmpA4x4[i];
        ftmpB4x4[i] = xftmpB4x4[i];
        ftmpA4x1[i] = xftmpA4x1[i];
        ftmpB4x1[i] = xftmpB4x1[i];
    }

    // 7 row arrays
    for (int i = 0; i < 7; i++)
    {
        ftmpA7x7[i] = xftmpA7x7[i];
        ftmpB7x7[i] = xftmpB7x7[i];
        ftmpA7x1[i] = xftmpA7x1[i];
    }
    // 10 row arrays
    for (int i = 0; i < 10; i++)
    {
        ftmpA10x10[i] = xftmpA10x10[i];
        ftmpB10x10[i] = xftmpB10x10[i];
        ftmpA10x1[i] = xftmpA10x1[i];
    }
    // MAXMATINV row arrays
//    for (int i = 0; i < MAXMATINV; i++)
//    {
//        icolind[i] = xicolind[i];
//        irowind[i] = xirowind[i];
//        ipivot[i] = xipivot[i];
//    }

    // for safety (ref option 0), initialize the geomagnetic field to something safe: 50uT and 50 deg
//    thisSimulation.fB = 50.0F;
//    thisSimulation.fDeltaDeg = 50.0F;

    // initialize the magnetometer simulation calibration parameters to null
//    fmatrixAeqI(thisSimulation.finvW, 3);   // null (identity matrix) inverse soft iron
//    fmatrixAeqI(thisSimulation.fW, 3);      // null (identity matrix) forward soft iron
//    thisSimulation.fVx = thisSimulation.fVy = thisSimulation.fVz = 0.0F;

    // reset computed magnetic calibration and magnetometer data buffer
    ResetMagCalibration(&thisMagCal, &thisMagneticBuffer);



    // initialize the low pass filters for 6DOF orientation
    fInitLPFOrientationMatrix(&fb0, &fa1, &fa2);
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
    InitMagCalibrationData();
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



