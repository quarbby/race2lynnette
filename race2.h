#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <assert.h>
#include <sys/select.h>
#include <math.h>

int main();
void wallFollowing();
void setAngles();
void calculateXY();
void inputMotors();
void turnLeft(double degrees);
void turnRight (double degrees);
void calculateTheta();
void wallFollowingR();