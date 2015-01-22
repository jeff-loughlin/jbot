# the compiler: gcc for C program, define as g++ for C++
CPP = g++

#OBJS = robot.o filter.o PID_v1.o magnetic.o matrix.o orientation.o geocoords.o
OBJS = robot.o filter.o pid.o geocoords.o

# compiler flags:
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CPPFLAGS  = -g -Wall -DDEFAULTDEVICE=\"/dev/ttyACM0\"


# the build target executable:
TARGET = robot

all: $(TARGET) ctrl

$(TARGET): $(OBJS)
	$(CPP) -g -lm -lc -pthread -o $@ $^

%.o : %.cpp include.h
	$(CPP) $(CPPFLAGS) -o $@ -c $<

ctrl: ctrl.cpp
	$(CPP) -o ctrl ctrl.cpp

clean:
	$(RM) $(TARGET) ctrl ctrl.o $(OBJS)
