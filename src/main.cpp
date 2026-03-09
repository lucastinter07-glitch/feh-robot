

#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <stdbool.h>
#include <FEHRCS.h> 

DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin10);
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor2,9.0);
AnalogInputPin CdS_cell(FEHIO::Pin0);

//main for AruCo code
void ERCMain() 
{ 

    float XPosition, YPosition, Heading;            //Declare floats to store RCS information 

 

    RCS.DisableRateLimit();   //Optionally disable the rate limit during testing 

    RCS.InitializeTouchMenu(input for 9 character code);  //Run Menu to select course section 

    RCSPose* pose = RCS.RequestPosition();      //Request position data from RCS 

    XPosition = pose->x;        //Get X data from RCS & store in float 

    YPosition = pose->y;        //Get Y data from RCS & store in float 

    Heading   = pose->heading;  //Get Heading data from RCS & store in float 

} 
/*
void move_forward(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);
    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((right_encoder.Counts() + left_encoder.Counts() /2) < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void move_backward(int percent, int counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //Set both motors to desired percent
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(-percent);
    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((right_encoder.Counts() + left_encoder.Counts() /2) < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}



bool isStartOn()
{
    float cellValue = CdS_cell.Value();
    return cellValue < 1.0;
}


void ERCMain()
{
    ///////////////////
    /////EDIT HERE/////
    ///////////////////

 //Input distance here
    int motor_percent = 40; //Input power level here (must be between 0 and 100
    int x, y; //for touch screen

    //Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Motor Test");
    LCD.WriteLine("Touch the screen");
    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed

    
    while(true){
    float cellValue = CdS_cell.Value();
    LCD.Clear(BLACK);
    LCD.WriteLine(cellValue);
    Sleep(0.5);
    }

    //detect if start button turns on
    
    if (isStartOn()){
        int inches = 5;
        motor_percent = 20;
        int expected_counts = 40.489*inches;
        move_backward(motor_percent, expected_counts); //drive backwards 5 inches to press button
        Sleep(1.0);
        move_forward(motor_percent, expected_counts);
        Sleep(2.0);
    }
    
    
    //drive 37 inches
    int inches = 37;
    motor_percent = 20;
    int expected_counts = 40.489*inches;
    move_forward(motor_percent, expected_counts);
    Sleep(8.0); //Wait for counts to stabilize

    //drive 2 feet
    inches = 36;
    motor_percent = 40;
    expected_counts = 40.489*inches;
    move_forward(motor_percent, expected_counts);
    Sleep(2.0); //Wait for counts to stabilize
    
    //move back 2 feet
    inches = 36;
    motor_percent = 40;
    expected_counts = 40.489*inches;
    move_backward(motor_percent, expected_counts);
    Sleep(2.0); //Wait for counts to stabilize
    */
