#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <assert.h>
#include <sys/select.h>
#include <math.h>
#include "picomms.h"
#include "robotMovement.h"



void wallFollowingL();
void wallFollowingR();
int main();