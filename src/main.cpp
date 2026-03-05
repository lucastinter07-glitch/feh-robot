
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin10);
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor2,9.0);

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

}