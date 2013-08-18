#include <stdlib.h>
#include <stdio.h>
#include "picomms.h"

const double ROBOT_RADIUS = ROBOT_WIDTH / 2.0 + 10.0;
const double ROBOT_WIDTH_DEG = ROBOT_WIDTH / (2.0 * M_PI * WHEEL_RADIUS) * 360.0;
const int SPEED = 50;//50
const int TURNSPEED = 25;//25
const int ULTRASOUNDTURNSPEED = 40;
int currentAngle = 0;

double degToRad(double a) {
    return a / 360.0 * 2.0 * M_PI;
}
// to store robot position
typedef struct RobotPosition {
	int speedL, speedR;

	double currentX, currentY, currentAngle;
	double previousX, previousY, previousAngle;

	double leftWheel, rightWheel;
	double prevLeftWheel, prevRightWheel;
} RobotPosition;

void turn(float angle) {
    const int SPEED = 10;

    char buf[80];
    float cmDistance = (ROBOT_WIDTH * M_PI) * abs(angle) / 360.0f;
    int ticksToGo = (int)((cmDistance / (2.0f * M_PI * WHEEL_RADIUS) * 360.0f)+0.5);
    
    int left_encoder;
    int right_encoder;
    
    int startValue, currentValue;
    one_sensor_read( "MEL", &startValue );
    currentValue = startValue;
    
    //printf("ticksToGo %d\n", ticksToGo);
    
    while( (abs(currentValue - startValue) < ticksToGo) &&(ticksToGo-abs(currentValue - startValue)>0)) {
        one_sensor_read( "MEL", &currentValue );
        //printf(" Sensor> left_encoder %d\n", abs(currentValue-startValue));
        
        if(angle < 0.0f)
            set_motors(SPEED, -SPEED);
        else
            set_motors(-SPEED, SPEED);
    }
    
    set_motors(0, 0);
}

// calculate the position of the robot
void calculatePosition(double *resultX, double *resultY, double *resultAngle, double diffL, double diffR, double angle) {
    double distance = diffL;
    double alpha, innerRadius;
    double angle_change = 0.0;

    if(diffL < diffR) {
		innerRadius = ROBOT_WIDTH_DEG * diffL / (diffR-diffL);
		alpha = diffR / 2.0 / M_PI / (ROBOT_WIDTH_DEG+innerRadius) * 360.0;
		angle_change = alpha;

		distance = 2.0 * cos(degToRad(90.0 - alpha / 2.0)) * (ROBOT_WIDTH_DEG / 2.0 + innerRadius);
    }

    if(diffL > diffR) {
		innerRadius = ROBOT_WIDTH_DEG * diffR / (diffL-diffR);
		alpha = diffL / 2.0 / M_PI / (ROBOT_WIDTH_DEG+innerRadius) * 360.0;
		angle_change = -alpha;

		distance = 2.0 * cos(degToRad(90.0 - alpha / 2.0)) * (ROBOT_WIDTH_DEG / 2.0 + innerRadius);
    }

    (*resultX) = cos(degToRad(angle+angle_change)) * distance / (180.0 / M_PI / WHEEL_RADIUS);
    (*resultY) = sin(degToRad(angle+angle_change)) * distance / (180.0 / M_PI / WHEEL_RADIUS);
	(*resultAngle) = angle_change;
}

//update the position of the robot
void updatePosition(RobotPosition* r) {
	double changeX, changeY, changeAngle;
	calculatePosition(&changeX, &changeY, &changeAngle, r->leftWheel - r->prevLeftWheel, r->rightWheel - r->prevRightWheel, r->previousAngle);
	r->currentX = r->previousX + changeX;
	r->currentY = r->previousY + changeY;
	r->currentAngle = r->previousAngle + changeAngle;
}

void updatePrevious(RobotPosition* r) {
	r->previousX = r->currentX;
	r->previousY = r->currentY;
	r->previousAngle = r->currentAngle;
	r->prevLeftWheel = r->leftWheel;
	r->prevRightWheel = r->rightWheel;
}

//reset the robot
void resetRobotPosition(RobotPosition* r, int leftW, int rightW) {
	r->speedL = r->speedR = 0;
	//r->currentX = r->currentY = r->currentAngle = 180.0;
	r->currentX = r->currentY =0;
	
	r->currentAngle = 0.0;
	r->previousAngle = 0.0;

	r->previousX = r->previousY =0;
	//r->previousX = r->previousY = r->previousAngle = 180.0;
	r->prevLeftWheel = r->leftWheel = (double)leftW;
	r->prevRightWheel = r->rightWheel = (double)rightW;
}

void updateRobotPosition(RobotPosition* r, int leftW, int rightW, int leftSpeed, int rightSpeed) {
	r->leftWheel = leftW;
	r->rightWheel = rightW;
	updatePosition(r);

	if(r->speedL != leftSpeed || r->speedR != rightSpeed) {
		updatePrevious(r);
	}
	
	r->speedL = leftSpeed;
	r->speedR = rightSpeed;
}

void follow_left_wall()
{
	int leftFrontIR, rightFrontIR;
	int leftSideIR, rightSideIR;
	int leftEncoder,rightEncoder;

	int ultraSound;

	RobotPosition robotPosition;

	set_ir_angle(LEFT,-35);
	set_ir_angle(RIGHT,35);

	two_sensor_read("MELR",&leftEncoder,&rightEncoder);
	resetRobotPosition(&robotPosition, leftEncoder, rightEncoder);
	//follow left wall
	while(1)
	{
		get_front_ir_dists(&leftFrontIR,&rightFrontIR);
		get_side_ir_dists(&leftSideIR,&rightSideIR);
		ultraSound = get_us_dist();

		if(ultraSound<30)
		{
			if(leftFrontIR>rightFrontIR)
			{
				set_motors(-ULTRASOUNDTURNSPEED,ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -ULTRASOUNDTURNSPEED, ULTRASOUNDTURNSPEED);
			}
			else
			{
				set_motors(ULTRASOUNDTURNSPEED,-ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, ULTRASOUNDTURNSPEED, -ULTRASOUNDTURNSPEED);
			}
		}
        else if(rightFrontIR<15)
        {
            set_motors(SPEED-TURNSPEED,SPEED+TURNSPEED);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED-TURNSPEED, SPEED+TURNSPEED);
        }
		else if(leftFrontIR>30)
		{
			set_motors(SPEED-TURNSPEED,SPEED+TURNSPEED);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED-TURNSPEED, SPEED+TURNSPEED);
		}
		else if(leftFrontIR<15)
		{
			set_motors(SPEED+TURNSPEED,SPEED-TURNSPEED);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+TURNSPEED, SPEED-TURNSPEED);
		}
		else
		{
			set_motors(SPEED,SPEED);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED, SPEED);
		}

		if(check_bump(LEFT)||check_bump(RIGHT))
		{
			if(check_bump(LEFT))
			{
				set_motors(-SPEED-20,-SPEED-40);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-20, -SPEED-40);
			}
			else if(check_bump(RIGHT))
			{
				set_motors(-SPEED-40,-SPEED-20);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-40, -SPEED-20);
			}
			else
			{
				set_motors(-SPEED-20,-SPEED-20);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-20, -SPEED-20);
			}
			
		}

	printf("CURRENT ANGLE: %lf POS: %lf %lf\n", robotPosition.currentAngle,robotPosition.currentX,robotPosition.currentY);

	}
	
}


void follow_right_wall()
{
	int leftFrontIR, rightFrontIR;
	int leftSideIR, rightSideIR;
	int leftEncoder,rightEncoder;

	int ultraSound;

	RobotPosition robotPosition;

	set_ir_angle(LEFT,-35);
	set_ir_angle(RIGHT,35);

	two_sensor_read("MELR",&leftEncoder,&rightEncoder);
	resetRobotPosition(&robotPosition, leftEncoder, rightEncoder);
	//follow right wall
	while(1)
	{
		get_front_ir_dists(&leftFrontIR,&rightFrontIR);
		get_side_ir_dists(&leftSideIR,&rightSideIR);
		ultraSound = get_us_dist();

		if(ultraSound<30)
		{
			if(leftFrontIR>rightFrontIR)
			{
				set_motors(-ULTRASOUNDTURNSPEED,ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -ULTRASOUNDTURNSPEED, ULTRASOUNDTURNSPEED);
			}
			else
			{
				set_motors(ULTRASOUNDTURNSPEED,-ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, ULTRASOUNDTURNSPEED, -ULTRASOUNDTURNSPEED);
			}
		}
		else if(rightFrontIR>30)
		{
			set_motors(SPEED+25,SPEED-25);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+25, SPEED-25);
		}
		else if(rightFrontIR<20)
		{
			set_motors(SPEED-25,SPEED+25);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED-25, SPEED+25);
		}
		else
		{
			if(abs(leftFrontIR-rightFrontIR)<5)
			{
				set_motors(SPEED+20,SPEED+20);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+20, SPEED+20);
			}
			else
			{
				set_motors(SPEED,SPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED, SPEED);
			}
			
		}

	printf("CURRENT ANGLE: %lf POS: %lf %lf\n", robotPosition.currentAngle,robotPosition.currentX,robotPosition.currentY);

	}
}

void follow_wall()
{
    int leftFrontIR, rightFrontIR;
	int leftSideIR, rightSideIR;
	int leftEncoder,rightEncoder;
    
	int ultraSound;
    int i=1;
	RobotPosition robotPosition;
    
	set_ir_angle(LEFT,-35);
	set_ir_angle(RIGHT,35);
    
	two_sensor_read("MELR",&leftEncoder,&rightEncoder);
	resetRobotPosition(&robotPosition, leftEncoder, rightEncoder);
	while(1)
	{

	//follow left wall
	printf("follow left wall--------/n");
	while(1)
	{
		get_front_ir_dists(&leftFrontIR,&rightFrontIR);
		get_side_ir_dists(&leftSideIR,&rightSideIR);
		ultraSound = get_us_dist();
        
		if(ultraSound<30)
		{
			if(leftFrontIR>rightFrontIR)
			{
				set_motors(-ULTRASOUNDTURNSPEED,ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -ULTRASOUNDTURNSPEED, ULTRASOUNDTURNSPEED);
			}
			else
			{
				set_motors(ULTRASOUNDTURNSPEED,-ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, ULTRASOUNDTURNSPEED, -ULTRASOUNDTURNSPEED);
			}
		}
        else if(rightFrontIR<15)
        {
            set_motors(SPEED-TURNSPEED-i*5,SPEED+TURNSPEED+i*5);
            //set_motors(-SPEED,SPEED);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED-TURNSPEED-i*5,SPEED+TURNSPEED+i*5);
        }
		else if(leftFrontIR>30)
		{
			set_motors(SPEED-TURNSPEED-i*5,SPEED+TURNSPEED+i*5);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED-TURNSPEED-i*5, SPEED+TURNSPEED+i*5);
		}
		else if(leftFrontIR<15)
		{
			set_motors(SPEED+TURNSPEED+i*5,SPEED-TURNSPEED-i*5);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+TURNSPEED+i*5, SPEED-TURNSPEED-i*5);
		}
		else
		{
			set_motors(SPEED+i*5,SPEED+i*5);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+i*5, SPEED+i*5);
		}
        
		if(check_bump(LEFT)||check_bump(RIGHT))
		{
			if(check_bump(LEFT))
			{
				set_motors(-SPEED-20,-SPEED-40);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-20, -SPEED-40);
			}
			else if(check_bump(RIGHT))
			{
				set_motors(-SPEED-40,-SPEED-20);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-40, -SPEED-20);
			}
			else
			{
				set_motors(-SPEED-20,-SPEED-20);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-20, -SPEED-20);
			}
			
		}
        if(robotPosition.currentAngle>190)
        {
            break;
        }
        
        printf("CURRENT ANGLE: %lf leftIR: %d rightIR %d\n", robotPosition.currentAngle,leftSideIR, rightSideIR);
        
	}
    
    while(robotPosition.currentAngle>170)
    {
        set_motors(20,-20);
        two_sensor_read("MELR", &leftEncoder, &rightEncoder);
        updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, 20, -20);
        
    }
    /*set_motors(SPEED+20+i*5,SPEED+20+i*5);
    two_sensor_read("MELR", &leftEncoder, &rightEncoder);
    updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+i*5, SPEED+i*5);*/
    get_front_ir_dists(&leftFrontIR,&rightFrontIR);
    ultraSound = get_us_dist();

    while(rightFrontIR>30||ultraSound>30)
    {
        set_motors(30+i*5,30+i*5);
        two_sensor_read("MELR", &leftEncoder, &rightEncoder);
        updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+i*5, SPEED+i*5);
        get_front_ir_dists(&leftFrontIR,&rightFrontIR);
        ultraSound = get_us_dist();
    }
    //follow right wall

    while(1)
	{
		printf("follow right wall--------/n");

		get_front_ir_dists(&leftFrontIR,&rightFrontIR);
		get_side_ir_dists(&leftSideIR,&rightSideIR);
		ultraSound = get_us_dist();
        
		if(ultraSound<30)
		{
			if(leftFrontIR>rightFrontIR)
			{
				set_motors(-ULTRASOUNDTURNSPEED,ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -ULTRASOUNDTURNSPEED, ULTRASOUNDTURNSPEED);
			}
			else
			{
				set_motors(ULTRASOUNDTURNSPEED,-ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, ULTRASOUNDTURNSPEED, -ULTRASOUNDTURNSPEED);
			}
		}
        else if(leftFrontIR<15)
        {
            set_motors(SPEED+TURNSPEED+i*5,SPEED-TURNSPEED-i*5);
			//set_motors(SPEED,-SPEED);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+TURNSPEED+i*5,SPEED-TURNSPEED-i*5);
            
        }
		else if(rightFrontIR>30)
		{
			set_motors(SPEED+TURNSPEED+i*5,SPEED-TURNSPEED-i*5);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+TURNSPEED+i*5, SPEED-TURNSPEED-i*5);
		}
		else if(rightFrontIR<15)
		{
			set_motors(SPEED-TURNSPEED-i*5,SPEED+TURNSPEED+i*5);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED-TURNSPEED-i*5, SPEED+TURNSPEED+i*5);
		}
		else
		{
			set_motors(SPEED+i*5,SPEED+i*5);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+i*5, SPEED+i*5);
		}
        
		if(check_bump(LEFT)||check_bump(RIGHT))
		{
			if(check_bump(LEFT))
			{
				set_motors(-SPEED,-SPEED-20);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED, -SPEED-20);
			}
			else if(check_bump(RIGHT))
			{
				set_motors(-SPEED-20,-SPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-20, -SPEED);
			}
			else
			{
				set_motors(-SPEED,-SPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED, -SPEED);
			}
			
		}
		if(leftSideIR<30&&rightSideIR<30)
		{
			break;
		}
        
        printf("CURRENT ANGLE: %lf leftIR: %d rightIR %d\n", robotPosition.currentAngle,leftSideIR, rightSideIR);
        
	}
	//follow left wall
    while(1)
	{
		printf("follow left wall--------/n");

		get_front_ir_dists(&leftFrontIR,&rightFrontIR);
		get_side_ir_dists(&leftSideIR,&rightSideIR);
		ultraSound = get_us_dist();
        
		if(ultraSound<30)
		{
			if(leftFrontIR>rightFrontIR)
			{
				set_motors(-ULTRASOUNDTURNSPEED,ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -ULTRASOUNDTURNSPEED, ULTRASOUNDTURNSPEED);
			}
			else
			{
				set_motors(ULTRASOUNDTURNSPEED,-ULTRASOUNDTURNSPEED);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, ULTRASOUNDTURNSPEED, -ULTRASOUNDTURNSPEED);
			}
		}
        else if(rightFrontIR<15)
        {
            set_motors(SPEED-TURNSPEED-i*5,SPEED+TURNSPEED+i*5);
            //set_motors(-SPEED,SPEED);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED-TURNSPEED-i*5,SPEED+TURNSPEED+i*5);
        }
		else if(leftFrontIR>30)
		{
			set_motors(SPEED-TURNSPEED-i*5,SPEED+TURNSPEED+i*5);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED-TURNSPEED-i*5, SPEED+TURNSPEED+i*5);
		}
		else if(leftFrontIR<15)
		{
			set_motors(SPEED+TURNSPEED+i*5,SPEED-TURNSPEED-i*5);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+TURNSPEED+i*5, SPEED-TURNSPEED-i*5);
		}
		else
		{
			set_motors(SPEED+i*5,SPEED+i*5);
			two_sensor_read("MELR", &leftEncoder, &rightEncoder);
			updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, SPEED+i*5, SPEED+i*5);
		}
        
		if(check_bump(LEFT)||check_bump(RIGHT))
		{
			if(check_bump(LEFT))
			{
				set_motors(-SPEED-20,-SPEED-40);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-20, -SPEED-40);
			}
			else if(check_bump(RIGHT))
			{
				set_motors(-SPEED-40,-SPEED-20);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-40, -SPEED-20);
			}
			else
			{
				set_motors(-SPEED-20,-SPEED-20);
				two_sensor_read("MELR", &leftEncoder, &rightEncoder);
				updateRobotPosition(&robotPosition, leftEncoder, rightEncoder, -SPEED-20, -SPEED-20);
			}
			
		}
        if(robotPosition.currentAngle>360)
        {
            break;
        }
        
        printf("CURRENT ANGLE: %lf POS: %lf %lf\n", robotPosition.currentAngle,robotPosition.currentX,robotPosition.currentY);
        
	}
    
    robotPosition.currentAngle = robotPosition.currentAngle-360;
    robotPosition.previousAngle = robotPosition.currentAngle;
    
	}
}

int main()
{
	connect_to_robot();
	initialize_robot();
	follow_wall();
}
