#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

struct termios orig_termios;
int fdFifo = 0;

#define MODE_MOTOR 0
#define MODE_SERVO 1
int panServo = 90;
int tiltServo = 90;

void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0)
    {
        return r;
    }
    else
    {
        return c;
    }
}

void SendCommand(const char *cmd)
{
    printf("Sending: %s\n", cmd);
    write(fdFifo, cmd, strlen(cmd));
}

int main(int argc, char *argv[])
{
    set_conio_terminal_mode();

    fdFifo = open("./RobotFifo", O_WRONLY);

    bool autoMode = true;
    char c = 0;
    int leftPower = 0;
    int rightPower = 0;
    int turn = 0;
    int speed = 0;

    int mode = MODE_MOTOR;

    while (c != 'q')
    {
        while (!kbhit())
        {
            usleep(1000);
        }
        c = getch();

        if (c == 'q')
            exit(0);
            
        if (c == 'h')
        {
            reset_terminal_mode();
            printf("\n\nNew heading: ");
            char hdg[255];
            fgets(hdg, 255, stdin);
            int newHeading = strtol(hdg, 0, 10);
            char cmd[255];
            sprintf(cmd, "H:%d\r", newHeading);
            SendCommand(cmd);
            set_conio_terminal_mode();
            continue;
        }
        if (c == 'c')
        {
            reset_terminal_mode();
            printf("\n\nCalibrating.  Rotate 360 degrees over 10 seconds...\n\n");
            char cmd[255];
            sprintf(cmd, "CAL:0\r");
            SendCommand(cmd);
            set_conio_terminal_mode();
            continue;
        }

        if (mode == MODE_MOTOR)
        {
            switch (c)
            {
            case '+':
            case '8':   speed++; autoMode = false; break;
            case '-':
            case '2':   speed--; autoMode = false; break;
            case '4':   turn--; autoMode = false; break;
            case '6':   turn++; autoMode = false; break;
            case '5':   turn = 0; autoMode = false; break;
            case '0':   speed = 0; turn = 0; autoMode = false; break;
            case 's':   mode = MODE_SERVO; break;
            case 'a':   speed = 0;
                        turn = 0;
                        autoMode = true;
                        SendCommand("A:0\r");
                        break;
            }

            if (!autoMode)
            {
                if (speed > 255)
                    speed = 255;
                if (speed < -255)
                    speed = -255;
                if (turn > 255)
                    turn = 255;
                if (turn < -255)
                    turn = -255;

                leftPower = speed + turn;
                rightPower = speed - turn;

                if (leftPower > 255)
                    leftPower = 255;
                if (leftPower < -255)
                    leftPower = -255;
                if (rightPower > 255)
                    rightPower = 255;
                if (rightPower < -255)
                    rightPower = -255;

                char cmd[256];
                sprintf(cmd, "M:%d,%d\r", leftPower, rightPower);
                SendCommand(cmd);
            }
        }
        else
        {
            switch (c)
            {
                case '4': panServo--; break;
                case '6': panServo++; break;
                case '8': tiltServo++; break;
                case '2': tiltServo--; break;
                case '5': panServo = 90; tiltServo = 90; break;
                case 'm': mode = MODE_MOTOR; break;
                case 'a':   speed = 0;
                            turn = 0;
                            autoMode = true;
                            SendCommand("A:0\r");
                            break;
            }
            if (panServo > 180)
                panServo = 180;
            if (panServo < 0)
                panServo = 0;
            if (tiltServo > 180)
                tiltServo = 180;
            if (tiltServo < 0)
                tiltServo = 0;

                char cmd[256];
                sprintf(cmd, "PT:%d,%d\r", panServo, tiltServo);
                SendCommand(cmd);
        }
    }
}
