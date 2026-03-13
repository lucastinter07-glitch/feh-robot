
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <stdbool.h>
#include <FEHRCS.h>

DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin10);
FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor2, 9.0);
AnalogInputPin CdS_cell(FEHIO::Pin1);

// main for AruCo code
/*


#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <FEHSD.h>

// RCS Delay time
#define RCS_WAIT_TIME_IN_SEC 0.35

// Shaft encoding counts for CrayolaBots
#define COUNTS_PER_INCH 40.5
#define COUNTS_PER_DEGREE 2.48

// Defines for pulsing the robot
#define PULSE_TIME 0.25
#define PULSE_POWER 20

// Define for the motor power
#define POWER 20

// Orientation of AruCo Code
#define PLUS 0
#define MINUS 1

//Declarations for encoders & motors
DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin9);
FEHMotor right_motor(FEHMotor::Motor1, 9.0);
FEHMotor left_motor(FEHMotor::Motor0, 9.0);


// * Pulse forward a short distance using time

void pulse_forward(int percent, float seconds)
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}


//Pulse counterclockwise a short distance using time

void pulse_counterclockwise(int percent, float seconds)
{
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    // Wait for the correct number of seconds
    Sleep(seconds);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//Move forward using shaft encoders where percent is the motor percent and counts is the distance to travel

void move_forward(int percent, int counts) //using encoders
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    // While the average of the left and right encoder are less than counts,
    // keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//Turn counterclockwise using shaft encoders where percent is the motor percent and counts is the distance to travel

void turn_counterclockwise(int percent, int counts)
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);

    // While the average of the left and right encoder are less than counts,
    // keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

// Use RCS to move to the desired x_coordinate based on the orientation of the AruCo code
void check_x(float x_coordinate, int orientation)
{
    // Determine the direction of the motors based on the orientation of the AruCo code
    int power = PULSE_POWER;
    if(orientation == MINUS){
        power = -PULSE_POWER;
    }

    RCSPose* pose = RCS.RequestPosition();

    // Check if receiving proper RCS coordinates and whether the robot is within an acceptable range
    for (int i = 0; i < 10; i++) {
        if(pose != nullptr && (pose->x < x_coordinate - 1 || pose->x > x_coordinate + 1))
        {
            if(pose->x < x_coordinate)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(-power, PULSE_TIME);
            }
            else if(pose->x > x_coordinate)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(power, PULSE_TIME);
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);

            pose = RCS.RequestPosition();
        }
}
}


//Use RCS to move to the desired y_coordinate based on the orientation of the QR code

void check_y(float y_coordinate, int orientation)
{
    // Determine the direction of the motors based on the orientation of the QR code
    int power = PULSE_POWER;
    if(orientation == MINUS){
        power = -PULSE_POWER;
    }

    RCSPose* pose = RCS.RequestPosition();

    // Check if receiving proper RCS coordinates and whether the robot is within an acceptable range
    for (int i = 0; i < 10; i++) {
        if(pose != nullptr && (pose->y < y_coordinate - 1 || pose->y > y_coordinate + 1))
        {
            if(pose->y < y_coordinate)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(-power, PULSE_TIME);
            }
            else if(pose->y > y_coordinate)
            {
                // Pulse the motors for a short duration in the correct direction
                pulse_forward(power, PULSE_TIME);
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);

            pose = RCS.RequestPosition();
        }
    }
}

// Use RCS to move to the desired heading

void check_heading(float heading)
{
    RCSPose* pose = RCS.RequestPosition();

    // Check if receiving proper RCS coordinates and whether the robot is within an acceptable range
    for (int i = 0; i < 10; i++) {
        // Calculate heading difference with wrap-around handling for 0/360 degrees
        float diff = heading - pose->heading;

        // Normalize difference to -180 to 180 range for proper angle comparison
        if(diff > 180) {
            diff -= 360;
        }
        else if(diff < -180) {
            diff += 360;
        }

        if(pose != nullptr && (diff < -1 || diff > 1))
        {
            if(diff > 0)
            {
                // Turn counterclockwise
                pulse_counterclockwise(PULSE_POWER, PULSE_TIME);
            }
            else if(diff < 0)
            {
                // Turn clockwise (opposite rotation)
                pulse_counterclockwise(-PULSE_POWER, PULSE_TIME);
            }
            Sleep(RCS_WAIT_TIME_IN_SEC);

            pose = RCS.RequestPosition();
        }
    }
}

void ERCMain()
{
    int touch_x,touch_y;
    float A_x, A_y, B_x, B_y, C_x, C_y, D_x, D_y;
    float A_heading, B_heading, C_heading, D_heading;
    int A_B_Counts, B_C_counts, C_D_counts, turn_90_counts;

    RCS.InitializeTouchMenu("Z1TESTING");

    LCD.WriteLine("RCS & Data Logging Test");
    LCD.WriteLine("Press Screen To Start");
    while(!LCD.Touch(&touch_x,&touch_y));
    while(LCD.Touch(&touch_x,&touch_y));

    // COMPLETE CODE HERE TO READ SD CARD FOR LOGGED X AND Y DATA POINTS
    FEHFile* fptr = SD.FOpen("RCS_TEST.txt", "r");
    SD.FScanf(fptr, "%f%f", &A_x, &A_y);
    SD.FScanf(fptr, "%f%f", &B_x, &B_y);
    SD.FScanf(fptr, "%f%f", &C_x, &C_y);
    SD.FScanf(fptr, "%f%f", &D_x, &D_y);
    SD.FClose(fptr);

    // WRITE CODE HERE TO SET THE HEADING DEGREES AND COUNTS VALUES
    A_heading = 90;
    B_heading = 180;
    C_heading = 270;
    D_heading = 0;

    A_B_Counts = 40.489*10;
    B_C_counts = 40.489*10;
    C_D_counts = 40.489*10;

    turn_90_counts = 2*3.1416*40.489;

    // A --> B
    move_forward(POWER, A_B_Counts);
    check_heading(A_heading);
    check_y(B_y, PLUS);
    check_heading(B_heading);

    // B --> C
    move_forward(POWER, B_C_counts);
    check_x(C_x, MINUS);
    turn_counterclockwise(POWER, turn_90_counts);
    check_heading(C_heading);

    // C --> D
    move_forward(POWER, C_D_counts);
    check_y(D_y, MINUS);
    turn_counterclockwise(POWER, turn_90_counts);
    check_heading(D_heading);
}
*/

void move_forward(int percent, int counts) // using encoders
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    // Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);
    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((right_encoder.Counts() + left_encoder.Counts() / 2) < counts)
        ;

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void move_backward(int percent, int counts) // using encoders
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    // Set both motors to desired percent
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(-percent);
    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((right_encoder.Counts() + left_encoder.Counts() / 2) < counts)
        ;

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void turn_left(int percent, int counts) // using encoders
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    // Set motors to turn left (counterclockwise)
    right_motor.SetPercent(percent);
    left_motor.SetPercent(-percent);
    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((right_encoder.Counts() + left_encoder.Counts() / 2) < counts);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void turn_right(int percent, int counts) // using encoders
{
    // Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    // Set motors to turn right (clockwise)
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(percent);
    // While the average of the left and right encoder is less than counts,
    // keep running motors
    while ((right_encoder.Counts() + left_encoder.Counts() / 2) < counts);

    // Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

bool isStartOn()
{
    float cellValue = CdS_cell.Value();
    return cellValue < 3.0;
}

// variant with higher threshold used for second/third detection loops
bool isStartOn25()
{
    float cellValue = CdS_cell.Value();
    return cellValue < 2.5;
}

void ERCMain()
{
    ///////////////////
    /////EDIT HERE/////
    ///////////////////

    // simple ring buffer for last three actions
    // 0=forward,1=backward,2=left,3=right
    int actType[3] = {0,0,0};
    int actPercent[3] = {0,0,0};
    int actCounts[3] = {0,0,0};
    int actIndex = 0;

    int motor_percent = 20; // Input power level here (must be between +/- 100)
    int x, y;               // for touch screen

    // Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Touch the screen");
    while (!LCD.Touch(&x, &y)); // Wait for screen to be pressed
    while (LCD.Touch(&x, &y)); // Wait for screen to be unpressed
    LCD.Clear(BLACK);

    float cellValue = CdS_cell.Value();

    while (isStartOn() != true)
    {
        // Wait for cell value to drop below 3 (original threshold)
        cellValue = CdS_cell.Value();
        LCD.Write("CdS Cell Value: ");
        LCD.WriteLine(cellValue);
        Sleep(0.4);

    }


    // Now move backward 1 seconds and set left and right motors to -20% to get off the button
    motor_percent = 20;
    right_motor.SetPercent(-motor_percent);
    left_motor.SetPercent(-motor_percent);
    Sleep(1.0);
    //turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //set left motor to 20% until it has moved 6 inches
    float inches = 6.5;
    float expected_counts = 40.489 * inches;
    left_motor.SetPercent(motor_percent);
    while (left_encoder.Counts() < expected_counts);

    //turn off motors
    right_motor.Stop();
    left_motor.Stop();
    Sleep(1.0);
    //move up the ramp
    inches = 48;
    motor_percent = 40;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts); // drive forward up the ramp
    // record forward action
    actType[actIndex] = 0; actPercent[actIndex] = motor_percent; actCounts[actIndex] = expected_counts;
    actIndex = (actIndex+1)%3;
    Sleep(1.0);

    // turn 90 degrees left at the top of the ramp
    inches = 8.6; //EDIT THIS 
    motor_percent = 20;
    expected_counts = 40.489 * inches;
    turn_left(motor_percent, expected_counts); // turn left a little bit
    // record turn left
    actType[actIndex] = 2; actPercent[actIndex] = motor_percent; actCounts[actIndex] = expected_counts;
    actIndex = (actIndex+1)%3;
    Sleep(1.0);

    //move forward until the cds cell value is below 2.5 (use new threshold)
    motor_percent = 20;
    while (isStartOn25() != true)
    {
        //set both motors to motor percent to move forward
        right_motor.SetPercent(motor_percent);
        left_motor.SetPercent(motor_percent);
    }
    //stop motors once the cell value is below 2
    right_motor.Stop();
    left_motor.Stop();
    //once the robot it on top of the light, read in the cds cell value and display it on the screen
    cellValue = CdS_cell.Value();
    LCD.Clear(BLACK);
    LCD.Write("CdS Cell Value: ");
    LCD.WriteLine(cellValue);

    //if the cell value is below 1.5, turn a little bit to the left and move towards the button. 
    if (cellValue < 1.0) //EDIT THIS
    {
        inches = 3.5;
        motor_percent = 20;
        expected_counts = 40.489 * inches;
        turn_left(motor_percent, expected_counts); // turn left a little bit
        // record turn left
        actType[actIndex] = 2; actPercent[actIndex] = motor_percent; actCounts[actIndex] = expected_counts;
        actIndex = (actIndex+1)%3;
        Sleep(1.0);
        move_forward(motor_percent, 40.489 * 5); // move forward 3 inches
        // record forward
        actType[actIndex] = 0; actPercent[actIndex] = motor_percent; actCounts[actIndex] = 40.489 * 5;
        actIndex = (actIndex+1)%3;
        Sleep(1.0);
        turn_right(motor_percent, expected_counts); // turn right a little bit
        // record turn right
        actType[actIndex] = 3; actPercent[actIndex] = motor_percent; actCounts[actIndex] = expected_counts;
        actIndex = (actIndex+1)%3;
        Sleep(1.0);
    }
    
    //if the cell value is above 3, turn a little bit to the right, move forward a little bit, then turn back to the left.
    else if (cellValue > 1.0)//EDIT THIS
    {
        inches = 3.5;
        motor_percent = 20;
        expected_counts = 40.489 * inches;
        turn_right(motor_percent, expected_counts); // turn right a little bit
        // record turn right
        actType[actIndex] = 3; actPercent[actIndex] = motor_percent; actCounts[actIndex] = expected_counts;
        actIndex = (actIndex+1)%3;
        move_forward(motor_percent, 40.489 * 5); // move forward 3 inches
        // record forward
        actType[actIndex] = 0; actPercent[actIndex] = motor_percent; actCounts[actIndex] = 40.489 * 5;
        actIndex = (actIndex+1)%3;
        turn_left(motor_percent, expected_counts); // turn left a little bit
        // record turn left
        actType[actIndex] = 2; actPercent[actIndex] = motor_percent; actCounts[actIndex] = expected_counts;
        actIndex = (actIndex+1)%3;
    }

    //move forward for 0.75 seconds to press the button 
    motor_percent = 20;
    right_motor.SetPercent(motor_percent);
    left_motor.SetPercent(motor_percent);
    Sleep(1000);
    //turn off motors
    right_motor.Stop();
    left_motor.Stop();

    // undo the last three recorded actions in reverse order
    for(int i=0; i<3; i++) {
        int idx = actIndex - 1 - i;
        if(idx < 0) idx += 3;
        switch(actType[idx]) {
            case 0: // forward -> back
                motor_percent = actPercent[idx];
                move_backward(motor_percent, actCounts[idx]);
                break;
            case 1: // backward -> forward
                motor_percent = actPercent[idx];
                move_forward(motor_percent, actCounts[idx]);
                break;
            case 2: // left -> right
                motor_percent = actPercent[idx];
                turn_right(motor_percent, actCounts[idx]);
                break;
            case 3: // right -> left
                motor_percent = actPercent[idx];
                turn_left(motor_percent, actCounts[idx]);
                break;
        }
        Sleep(1.0);
    }

    //move backwards 26 inches
    inches = 23;
    motor_percent = 25;
    expected_counts = 40.489 * inches;
    move_backward(motor_percent, expected_counts); // move backwards 26 inches to get off button and back down the ramp

    //turn 90 degrees to the left
    inches = 8.6; //EDIT THIS
    motor_percent = 20;
    expected_counts = 40.489 * inches;
    turn_left(motor_percent, expected_counts); // turn left 90 degrees to face the ramp

    //move forward and down the ramp
    inches = 32;
    motor_percent = 25;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts); // move forward and down the ramp





}