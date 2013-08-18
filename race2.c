#include "race2.h"
#include "picomms.h"

#define SPEED 55
#define DIAMETER 22.5
#define WHEEL_RADIUS 10.0
#define defaultSpeed 10.0

double thetaAcc = -M_PI;
double click = M_PI*DIAMETER/ 360.0;
double rad = M_PI*(WHEEL_RADIUS*2/DIAMETER)/360.0;
int prevL, prevR, currL, currR;
int xpos, ypos, prevx, prevy;

int main()
{
         connect_to_robot();
        initialize_robot();
    setAngles();
    wallFollowing();

    one_sensor_read("MEL", &prevL);
    one_sensor_read("MER", &prevR);
}

void setAngles(){
    set_ir_angle(RIGHT, 20);
    set_ir_angle(LEFT, -20);
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
        set_motors(-SPEED, SPEED);
        }
           else{
        set_motors(-SPEED, -SPEED);         
                 set_motors(SPEED, -SPEED);
       }
    get_front_ir_dists(&leftFrontIR, &rightFrontIR);
    //}
                   
                   setAngles();
        }


        //Deadlock
        if (ultrasound < 30 && leftFrontIR < 40 && rightFrontIR < 40)
        {   
            int i;

            //while ((ultrasound < 80 && leftFrontIR < 45) || (ultrasound < 80 && rightFrontIR < 45))
            /*
            while (ultrasound < 87)
                { set_motors(-SPEED, -SPEED); 
                   //get_side_ir_dists(&leftFrontIR, &rightSideIR); 
                   ultrasound = get_us_dist();
               }
            
               
               set_ir_angle(RIGHT, 45);
               set_ir_angle(LEFT, -45);
               */
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

        else{
            
            if (leftFrontIR > rightFrontIR)
            {       
                set_motors(SPEED-25, SPEED+25);
                calculateXY();

                //addNode(pointer, LEFT);
            }

            else if (rightFrontIR > leftFrontIR )
            {
                set_motors(SPEED+25, SPEED-25);
                            calculateXY();

                //addNode(pointer, RIGHT);
            }

            else 
            { 
                set_motors(SPEED, SPEED);
                            calculateXY();

                //addNode(pointer, STRAIGHT);
            }
        }

    }
}

void calculateXY(){
    inputMotors();
    double diffL = currL - prevL;
    double diffR = currR - prevR;
    double ddist = 0.5 * (diffL + diffR)*click;
    double theta = thetaAcc;
    double dx =  ddist * cos(thetaAcc);
    double dy = ddist * sin(thetaAcc);
    double dtheta = (diffR - diffL)*rad;
    thetaAcc += dtheta;

    if (thetaAcc > 2*M_PI)
        { thetaAcc -= 2*M_PI; }
    else if (thetaAcc <= 0)
        {   thetaAcc += 2*M_PI;}

    xpos = (int) prevx + dx;
    ypos = (int) prevy + dy;

    prevL = currL;
    prevR = currR;

    //printf("%f %d %d\n", theta, xpos, ypos);
}

//Input the motor encoder values
void inputMotors(){
    one_sensor_read("MEL", &currL);
    one_sensor_read("MER", &currR);
}