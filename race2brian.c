//Race track 2 - unseen
#include "race2.h"

#define SPEED 20

int main ()
{	
	//setAngles();
	printf("follow right wall");
	wallFollowingR();
}

// Basic Right wall following code
void wallFollowingR()
{
	int rightSideIR, leftSideIR, rightFrontIR, leftFrontIR, distAhead, i;
	while (1)
	//for(i = 0; i < 174; i++)
	{
		get_front_ir_dists(&leftFrontIR, &rightFrontIR);
		get_side_ir_dists(&leftSideIR, &rightSideIR);
		distAhead = get_us_dist();
		
		printf("i = %i\n", i);
		
		if ( leftFrontIR < 30 )
		{ turnLeft (90.0, 5);} // super turn left
		
		else if (rightFrontIR > 30 && rightSideIR > 26)
		{ set_motors(SPEED+50, SPEED-15);} // super turn right
		
		else if (rightFrontIR > 30)
		{ set_motors(30, SPEED);} // too far, turn Right
		
		else if ( rightFrontIR < 18 )
		{ set_motors(SPEED, SPEED * 1.5 );}  // turn left
		
		else
		{set_motors(SPEED+5, SPEED+5); } // Straight
	}
}

// Basic Left wall following code
void wallFollowingL()
{
	int rightSideIR, leftSideIR, rightFrontIR, leftFrontIR, distAhead, i;
	while (1)
	//for(i = 0; i < 250; i++)
	{
		get_front_ir_dists(&leftFrontIR, &rightFrontIR);
		get_side_ir_dists(&leftSideIR, &rightSideIR);
		distAhead = get_us_dist(); 
		
		printf("i = %i\n", i);
		
		if ( rightFrontIR < 30 )
		{ turnRight (90.0, 5);} // super turn right
		
		else if (leftFrontIR > 30 && leftSideIR > 25)
		{ set_motors(SPEED-15, SPEED+50);} // super turn left
		
		else if (leftFrontIR > 38)
		{ set_motors(SPEED, 30);} // too far, turn left
		
		else if ( leftFrontIR < 18 )
		{ set_motors(SPEED*1.5, SPEED);}  // turn right
		
		else
		{set_motors(SPEED+5, SPEED+5); } // Straight
	}
}
