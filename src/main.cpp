
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

DigitalEncoder right_encoder(FEHIO::Pin8);
DigitalEncoder left_encoder(FEHIO::Pin9);
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor1,9.0);


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
    while((right_encoder.Counts()) < counts);

    //Turn off motors
    right_motor.Stop();
}

void ERCMain()
{
    ///////////////////
    /////EDIT HERE/////
    ///////////////////

    int inches = 6; //Input distance here
    int motor_percent = 20; //Input power level here
    int expected_counts = 40.489; //Input theoretical counts per inch of linear motion here

    int x, y; //for touch screen

    //Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Motor Test");
    LCD.WriteLine("Touch the screen");
    while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
    while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed

    //drive 6 inches
    inches = 6;
    expected_counts = 40.489*inches;
    move_forward(motor_percent, expected_counts);

    Sleep(2.0); //Wait for counts to stabilize
    //Print out data
    LCD.Write("Theoretical Counts: ");
    LCD.WriteLine(expected_counts);
    LCD.Write("Motor Percent: ");
    LCD.WriteLine(motor_percent);
    LCD.Write("Actual Right Counts: ");
    LCD.WriteLine(right_encoder.Counts());
    LCD.WriteLine("Actual Left Counts: ");
    LCD.WriteLine(left_encoder.Counts());

}