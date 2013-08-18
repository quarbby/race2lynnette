#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <math.h>

#define WHEEL_RADIUS 10.00
#define click 0.0872222     //1 click=0.0872222 cm
#define defaultSpeed 10.0		//Calibrated robot for default speed of 10

char buf[80];
int motorValues[2];

void straight(double distance, int speed);
void straight2(double distance, int speedLeft, int speedRight);
void straightback(double distance, int speedLeft, int speedRight);
void printTrail();
void readMotors();
void turnRight(double degrees, int speed);
void turnLeft(double degrees, int speed);
int distSideSensor(char* sensorName);
int distFrontSensor(char* sensorName);
void motorSpeed(int speedLeft, int speedRight);
int main();
