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

#define SPEED 40

// Basic Right wall following code
void wallFollowingR()
{
	int rightSideIR, leftSideIR, rightFrontIR, leftFrontIR, distAhead, i;
	while (1)
	{

		get_front_ir_dists(&leftFrontIR, &rightFrontIR);
		get_side_ir_dists(&leftSideIR, &rightSideIR);
		distAhead = get_us_dist();

		if (distAhead < 45){	
			if (leftFrontIR - rightFrontIR > 0){
				set_motors(-SPEED, SPEED);
			}

			else {
				set_motors(20, -20);
			}
		}

		else if (rightFrontIR > 30 && rightSideIR > 25)
			{	set_motors(SPEED+30, SPEED-8); } //turn right

		else if (rightFrontIR < 20 )
			{	set_motors (SPEED-10, SPEED+25); } //turn left

		else
			{set_motors(SPEED+8, SPEED+8); } // Straight

	}


}

// Basic Left wall following code
void wallFollowingL()
{
	int rightSideIR, leftSideIR, rightFrontIR, leftFrontIR, distAhead, i;
	while (1)
	{
		get_front_ir_dists(&leftFrontIR, &rightFrontIR);
		get_side_ir_dists(&leftSideIR, &rightSideIR);
		distAhead = get_us_dist();

		if (distAhead < 40){		
			if (leftFrontIR - rightFrontIR > 0){
				set_motors(-SPEED, SPEED);
			}

			else {
				set_motors(SPEED, -SPEED);
			}
		}


		if (leftFrontIR > 30 )
			{	set_motors(SPEED-10, SPEED+10); } //turn left

		else if (leftFrontIR < 15 )
			{	set_motors (SPEED+10, SPEED-10); } //turn right

		else
			{set_motors(SPEED, SPEED); } // Straight

	}
}

void setAnglesStart()
{
	set_ir_angle(RIGHT, 35);
  set_ir_angle(LEFT, -35);
}


int main ()
{	
	connect_to_robot();
	initialize_robot();
	setAnglesStart();
	//wallFollowingR();	
	wallFollowingL();
}


