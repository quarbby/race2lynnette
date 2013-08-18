//Code for straight and turns robot movements
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <math.h>
#include "robotMovement.h"
#include "picomms.h"

/*
int main()
{
    int mysock;
    mysock = connect_to_robot();

    straight(55.0, +20);
	//turn(90);

}
*/

void printTrail()
{
    sprintf(buf, "C TRAIL\n");
    send_msg(buf, strlen(buf));
    recv_msg(buf, strlen(buf));
}   


void motorSpeed(int speedLeft, int speedRight)
{
    char buf[80];
    sprintf(buf, "M LR %d %d\n", speedLeft, speedRight);
    send_msg(buf, strlen(buf));
    recv_msg(buf, strlen(buf));
}

void straight(double distance, int speed)
{
    double numOfClicks = (distance*360.0)/ (2.0*M_PI* WHEEL_RADIUS);

    int motorvalue, returnvalue;
    one_sensor_read("MEL", &motorvalue);
    int clicksToMove = motorvalue + numOfClicks + 1;

    while (motorvalue < clicksToMove)
    {
    	sprintf(buf, "M LR %d %d\n", speed, speed);
    	send_msg(buf, strlen(buf));
    	recv_msg(buf, strlen(buf));

    	//printTrail();
             
        returnvalue = one_sensor_read("MEL", &motorvalue);
    }
}


void straight2(double distance, int speedLeft, int speedRight)
{
    double numOfClicks = (distance*360.0)/ (2.0*M_PI* WHEEL_RADIUS);

    int motorvalue, returnvalue;
    one_sensor_read("MEL", &motorvalue);
    int clicksToMove = motorvalue + numOfClicks + 1;

    while (motorvalue < clicksToMove)
    {
    	sprintf(buf, "M LR %d %d\n", speedLeft, speedRight);
    	send_msg(buf, strlen(buf));
    	recv_msg(buf, strlen(buf));
             
        returnvalue = one_sensor_read("MEL", &motorvalue);
    }
}

void straightback(double distance, int speedLeft, int speedRight)
{
    double numOfClicks = (distance*360.0)/ (2.0*M_PI* WHEEL_RADIUS);

    int motorvalue, returnvalue;
    one_sensor_read("MER", &motorvalue);
    int clicksToMove = motorvalue + numOfClicks + 1;
	int lbump, rbump;
	
    while (motorvalue < clicksToMove)
    {
		
	
    	sprintf(buf, "M LR %d %d\n", speedLeft, speedRight);
    	send_msg(buf, strlen(buf));
    	recv_msg(buf, strlen(buf));
             
        returnvalue = one_sensor_read("MER", &motorvalue);
    }
}

void turnRight (double degrees, int speed)
{
	readMotors();
    
    double calibratedDeg = degrees + (speed*1.0/ defaultSpeed);
    double distance = (47.0 * M_PI) * (calibratedDeg/ 360.0);
    double movement = (distance / click) / 2;
    int leftToMove = motorValues[0] + movement + 1;
    int rightToMove = motorValues[1] - movement + 1;

    while (motorValues[0] < leftToMove && motorValues[1] > rightToMove)
    {
        sprintf(buf, "M LR +10 -10\n");
    	send_msg(buf, strlen(buf));
    	recv_msg(buf, strlen(buf));
        
        readMotors();
    }
}

void turnLeft (double degrees, int speed)
{
    readMotors();
    
    double calibratedDeg = degrees + (speed*1.0/ defaultSpeed);
    double distance = (47.0 * M_PI) * (calibratedDeg/ 360.0);
    double movement = (distance / click) / 2;
    int leftToMove = motorValues[0] - movement + 1;
    int rightToMove = motorValues[1] + movement + 1;
       
    while (motorValues[0] > leftToMove && motorValues[1] < rightToMove)
    {
        sprintf(buf, "M LR -10 +10\n");
        send_msg(buf, strlen(buf));
        recv_msg(buf, strlen(buf));
        
        readMotors();
    }
}

void readMotors()
{
    one_sensor_read("MEL", motorValues);
    one_sensor_read("MER", motorValues+1);
}

//Reads side sensor (gp2d120)
int distSideSensor(char* sensorName){
    int irValue, dist;
    one_sensor_read(sensorName, &irValue);
    dist = (2914/(irValue +4) -1);
    return dist;
}

//Reads front sensors (gp2d12)
int distFrontSensor(char* sensorName)
{
    int irValue, dist;
    one_sensor_read(sensorName, &irValue);
    dist = (6787/ (irValue - 3) - 4);
    return dist;
}

void stop()
{
	char buf[80];
	sprintf(buf, "M LR 0 0\n");
	send_msg(buf, strlen(buf));
	recv_msg(buf, strlen(buf));
}
