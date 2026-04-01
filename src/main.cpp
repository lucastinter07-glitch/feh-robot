#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHRCS.h>
#include <FEHMotor.h>

FEHMotor right_motor(FEHMotor::Motor0, 9.0);
FEHMotor left_motor(FEHMotor::Motor2, 9.0);
AnalogInputPin CdS_cell(FEHIO::P0_0);

void ERCMain()
{
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


    //Now move backward 1 seconds and set left and right motors to -20% to get off the button
    motor_percent = 20;
    right_motor.SetPercent(-motor_percent);
    left_motor.SetPercent(-motor_percent);
    Sleep(1.0);
    //turn off motors
    right_motor.Stop();
    left_motor.Stop();
    Sleep(0.5);

    //move forward 3 inches 
    motor_percent = 20;
    float inches = 8.0;
    float expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);

    //turn right 90 degrees 
    motor_percent = 20;
    inches = 8.6; //90 degree turn in place (three quarters the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_right(motor_percent, expected_counts);
    Sleep(0.5);

    //drive backwards 5 inches
    inches = 3.0;
    expected_counts = 40.489 * inches;
    move_backward(motor_percent, expected_counts);

    //turn left 45 degrees
    inches = 4.3; // 45 degree turn in place (quarter the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_left(motor_percent, expected_counts);
    Sleep(0.5);

    //drive back for 5 seconds then turn the motors off 
    motor_percent = 35;
    right_motor.SetPercent(-motor_percent);
    left_motor.SetPercent(-motor_percent);
    Sleep(2.0);
    //turn off motors
    right_motor.Stop();
    left_motor.Stop();
    Sleep(0.5);

    //move forward 5 inches
    motor_percent = 20;
    inches = 5.0;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);
    Sleep(0.5);

    //turn 45 degrees to the right 
    inches = 4.3; // 45 degree turn in place (quarter the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_right(motor_percent, expected_counts);
        Sleep(0.5);
    
    //move forward 6 inches
    inches = 9.0;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);
    Sleep(0.5);

    //turn left 45 degrees
    inches = 4.4; // 45 degree turn in place (quarter the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_left(motor_percent, expected_counts);
    Sleep(0.5);

    //move forward 35 inches
    motor_percent = 40;
    inches = 40.0;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);
    Sleep(0.5);

    //turn lefft 90 degrees
    motor_percent = 20;
     inches = 8.6; // 90 degree turn in place (half the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_left(motor_percent, expected_counts);
    Sleep(0.5);

    //drive backward for 3 seconds then turn the motors off
    motor_percent = 35;
    right_motor.SetPercent(-motor_percent);
    left_motor.SetPercent(-motor_percent);
    Sleep(2.0);
    //turn off motors
    right_motor.Stop();
    left_motor.Stop();
    Sleep(0.5);

    //drive forward 20 inches but have the right motor go a tiny bit faster. 
    motor_percent = 50;
    inches = 25.0;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);
    Sleep(0.5);

    //move backwards 2 inches
    motor_percent = 20;
    inches = 2.0;
    expected_counts = 40.489 * inches;
    move_backward(motor_percent, expected_counts);
    Sleep(0.5);

    //turn right a little bit 
    inches = 4.0; // 45 degree turn in place (quarter the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_right(motor_percent, expected_counts);
    Sleep(0.5);

    //move forward 4 inches
    inches = 4.0;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);
    Sleep(0.5);

    //straighten out from the right turn by turning left a little bit
    inches = 4.0; // 45 degree turn in place (quarter the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_left(motor_percent, expected_counts);
    Sleep(0.5);

    //move forward 7 inches
    inches = 5.0;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);
    Sleep(0.5);

    //turn to the right a little bit.
    inches = 4.0; // 45 degree turn in place (quarter the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_right(motor_percent, expected_counts);
    Sleep(0.5);

    //move backwards 4 inches
    inches = 4.0;
    expected_counts = 40.489 * inches;
    move_backward(motor_percent, expected_counts);
    Sleep(0.5);

    //turn left a little bit
    inches = 4.0; // 45 degree turn in place (quarter the circumference
    expected_counts = 40.489 * inches;
    turn_left(motor_percent, expected_counts);
    Sleep(0.5);

    //move backwards 7 inches
    motor_percent = 40;
    inches = 12.0;
    expected_counts = 40.489 * inches;
    move_backward(motor_percent, expected_counts);
    Sleep(0.5);


    //move forward 2 inches
    inches = 2.0;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);

    //turn right a little bit
    inches = 4.0; // 45 degree turn in place (quarter the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_right(motor_percent, expected_counts);

    //move forward 4 inches
    inches = 4.0;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);

    //turn left to straighten out from the right turn
    inches = 4.0; // 45 degree turn in place (quarter the circumference
    expected_counts = 40.489 * inches;
    turn_left(motor_percent, expected_counts);

    //move backward 10 inches 
    inches = 10.0;
    expected_counts = 40.489 * inches;
    move_backward(motor_percent, expected_counts);

    //turn left 90 degrees
    inches = 8.6; // 90 degree turn in place (half the circumference of a circle with radius equal to half the distance between wheels)
    expected_counts = 40.489 * inches;
    turn_left(motor_percent, expected_counts);

    //forward 30 inches
    inches = 30.0;
    expected_counts = 40.489 * inches;
    move_forward(motor_percent, expected_counts);


    //stop both motors
    right_motor.Stop();
    left_motor.Stop();
*/
