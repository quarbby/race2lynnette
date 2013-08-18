#include "race2.h"
#include "picomms.h"

#define DIAMETER 22.5
#define WHEEL_RADIUS 10.0

int SPEED = 42;

double thetaAcc = 0.0;
double moveClick = (M_PI * DIAMETER) / 360.0;
double rad = M_PI*(WHEEL_RADIUS*2/DIAMETER)/360.0;
int currL, currR; 
int prevL=0, prevR=0;

int main()
{
    connect_to_robot();
    initialize_robot();
	setAngles();
    inputMotors();
    //wallFollowing();

    int i;
    for (i=0; i<4; i++){
        printf("%d\n", i);
        thetaAcc = 0.0;
        SPEED = SPEED + 8;
        //wallFollowing();
        wallFollowingR();
    }

}

void setAngles(){
    set_ir_angle(RIGHT, 20);
    set_ir_angle(LEFT, -20);
}

void inputMotors(){
    one_sensor_read("MEL", &currL);
    one_sensor_read("MER", &currR);
}

void calculateTheta(){
    inputMotors();

    if (prevL != 0 && prevR != 0){
        double diffL = currL - prevL;
        double diffR = currR - prevR;
        double ddist = 0.5 * (diffL + diffR)*moveClick;
        double dtheta = abs((diffR - diffL)) * rad;
        thetaAcc += dtheta;
    }
    
    prevL = currL;
    prevR = currR;

    printf("%f\n", thetaAcc);
}

//Basic wall following code
void wallFollowing()
{
    //Sensor readings 
    int rightSideIR, leftSideIR, rightFrontIR, leftFrontIR;
    int ultrasound;
    get_front_ir_dists(&leftFrontIR, &rightFrontIR);
    get_side_ir_dists(&leftSideIR, &rightSideIR);
    inputMotors();

    while (1)
    {
        if (thetaAcc > 40)
            { break; }

        int lbump, rbump;
        check_bumpers(&lbump, &rbump);
        if (lbump){
            set_motors(SPEED+20, SPEED-20);
        }
        if (rbump){
            set_motors(SPEED-20, SPEED+20);
        }

        get_front_ir_dists(&leftFrontIR, &rightFrontIR);
        get_side_ir_dists(&leftSideIR, &rightSideIR);
        ultrasound = get_us_dist(); 


        //Hit wall
        if (ultrasound < 30){
            int i;
/*
           for(i=0; i<20; i++){
                set_motors(-SPEED, -SPEED);
                ultrasound = get_us_dist();
            }
*/
            //set_ir_angle(RIGHT, 45);
            //set_ir_angle(LEFT, -45);

        get_front_ir_dists(&leftFrontIR, &rightFrontIR);
            //for (i=0; i<15; i++){
        if (leftFrontIR - rightFrontIR > 0){
        set_motors(-SPEED, -SPEED);
        //set_motors(-SPEED, SPEED);
        set_motors(-25,25);
        }
           else{
        set_motors(-SPEED, -SPEED);         
                 set_motors(SPEED, -SPEED);
       }
    get_front_ir_dists(&leftFrontIR, &rightFrontIR);
    //}
                   
                   setAngles();
        }


/*
        //Deadlock
        if (ultrasound < 30 && leftFrontIR < 40 && rightFrontIR < 40)
        {   
            int i;

            //while ((ultrasound < 80 && leftFrontIR < 45) || (ultrasound < 80 && rightFrontIR < 45))
            
            while (ultrasound < 87)
                { set_motors(-SPEED, -SPEED); 
                   //get_side_ir_dists(&leftFrontIR, &rightSideIR); 
                   ultrasound = get_us_dist();
               }
            
               
               set_ir_angle(RIGHT, 45);
               set_ir_angle(LEFT, -45);
               
               get_front_ir_dists(&leftFrontIR, &rightFrontIR);
               //printf("%d %d\n", leftFrontIR, rightFrontIR);
               
                   if (leftFrontIR - rightFrontIR > 0){
                    //for (i=0; i<20; i++)
                        set_motors(-SPEED-20, -SPEED+20);
                    //turnLeft(50);
                   }
                   else if (rightFrontIR - leftFrontIR > 0){
                    //for (i=0; i<20; i++)
                        set_motors(-SPEED+20, -SPEED-20);
                    //turnRight(50);
                   }
               
               setAngles();
               

               

           //Make 90 deg left turn 
               //for (i=0; i<35; i++)
               //{ set_motors(SPEED, -SPEED); }
        } 
        */

        else{
            
            if (leftFrontIR > rightFrontIR)
            {       
                set_motors(SPEED-25, SPEED+25);

                //addNode(pointer, LEFT);
            }

            else if (rightFrontIR > leftFrontIR )
            {
                set_motors(SPEED+25, SPEED-25);

                //addNode(pointer, RIGHT);
            }

            else 
            { 
                set_motors(SPEED, SPEED);

                //addNode(pointer, STRAIGHT);
            }
        }
        calculateTheta();
    }
}

// Basic Right wall following code
void wallFollowingR()
{
    int rightSideIR, leftSideIR, rightFrontIR, leftFrontIR, distAhead, i;
    while (1)
    //for(i = 0; i < 170; i++)
    {
        if (thetaAcc > 40)
            { break; }

        get_front_ir_dists(&leftFrontIR, &rightFrontIR);
        get_side_ir_dists(&leftSideIR, &rightSideIR);
        distAhead = get_us_dist();

        int lbump, rbump;
        check_bumpers(&lbump, &rbump);
        if (lbump){
            set_motors(SPEED+20, SPEED-20);
        }
        if (rbump){
            set_motors(SPEED-20, SPEED+20);
        }
        
        if (distAhead < 35){    //in the L shape
            if (leftFrontIR - rightFrontIR > 0){
                set_motors(-SPEED, SPEED);
            }

            else {
                set_motors(+20,-20);
            }
        }

        else if (rightFrontIR > 30)
            {   set_motors(SPEED+10, SPEED-20); } //turn right

        else if (rightFrontIR < 20)
            {   set_motors (SPEED-25, SPEED+15); } //turn left

        else
            {set_motors(SPEED+7, SPEED+7); } // Straight

        calculateTheta();
        
    }
}
