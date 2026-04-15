/* ------------------------------*/
/* FEH ROBOT PROJECT TEAM H3     */
/* Lucas Tinter, Will Castlen,   */
/* Hayden Yang, Faiza Choudhry   */
/* 3/30/26  v2.0.4               */
/* ------------------------------*/

// Library Declarations
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHBuzzer.h>
#include <FEHRCS.h>
#include <FEHSD.h>
#include <stdio.h>
#include <string.h>

// Part/Hardware Declarations
FEHMotor right_motor(FEHMotor::Motor0, 9.0); // Right Motor
FEHMotor left_motor(FEHMotor::Motor2, 9.0); // Left Motor
FEHMotor rear_servo_motor(FEHMotor::Motor3, 5.0); // Rear Servo Motor
DigitalEncoder right_encoder(FEHIO::Pin8); // Right Motor Shaft Encoder
DigitalEncoder left_encoder(FEHIO::Pin10); // Left Motor Shaft Encoder

AnalogInputPin CdS_cell(FEHIO::Pin1); // CdS Cell Sensor

AnalogInputPin line_left(FEHIO::Pin2); // Left Optosensor
AnalogInputPin line_center(FEHIO::Pin3); // Center Optosensor
AnalogInputPin line_right(FEHIO::Pin4); // Right Optosensor

FEHServo front_arm_servo(FEHServo::Servo0); // Front Servo Motor

// General Variable Declarations
#define g 9.8

// Driving Variable Declarations
#define DRIVE_PERCENT 20
#define RAMP_PERCENT 40f
#define TURN_PERCENT 20
#define COUNTS_PER_INCH 34.82
#define COUNTS_PER_DEGREE 2.58

// Line Following Variable Declarations
#define LINE_THRESHOLD 3.5f
#define LINE_BASE_SPEED 15             
#define LINE_KP 30             
#define LINE_TIMEOUT 10.0f
#define LINE_FOLLOW_POWER 20.0f      

// PID Constant Variable Declarations
#define P_CONST 0.85f
#define I_CONST 0.05f             
#define D_CONST 0.25f             
#define PID_SLEEP 0.025f             
#define TARGET_SPEED 10.0f
#define TURN_TARGET_SPEED 3.0f
#define TURN_SLOWDOWN_SPEED 0.7f
#define RAMP_TARGET_SPEED 12.0f

// Cds Cell Sensor Value Declarations
#define CDS_RED_MAX 0.75f
#define CDS_BLUE_MAX 2.25f

// Servo Motor Variable Declarations
#define ARM_SERVO_MIN 500            // TODO
#define ARM_SERVO_MAX 2500            // TODO
#define ARM_UP 160.0f         // degrees: arm stowed
#define ARM_DOWN 30.0f         // degrees: arm pressing a button

// Light Enum for CdS
enum LightColor { NO_LIGHT, RED_LIGHT, BLUE_LIGHT };

// PID Struct
struct PIDState
{
    float past_error; // stores error from the previous update (D)
    float error_sum; // running total of all errors (I)
    int   past_counts; // encoder counts at last update  
    float past_time; // TimeNow() at last update  
    float motor_power; // current motor power %
};

// PID State Variables for left and right motors
PIDState pid_right;
PIDState pid_left;

// Function Prototypes
void reset_pid();
float pid_adjustment(PIDState &state, int current_counts, float target_speed);
void drive_forward(float inches);
void drive_backward(float inches);
void turn_left(float degrees);
void turn_right(float degrees);
void drive_forward_time(float percent_r,float percent_l, float seconds);
void drive_backward_time(float percent, float seconds);
void stop_motors();
LightColor read_light_color();
void wait_for_start();
void press_start_button();
void follow_line(float timeout);
void sweep_servo(float start_deg, float end_deg, float step, float step_delay);
void drive_forward_ramp(float inches);
void compost();
void apple_pickup_ramp();
void lever_flip();
void humidifier();
void window();

// Execution of Tasks
void ERCMain()
{
    //calibration
    //RCS.InitializeTouchMenu("H3");
    front_arm_servo.TouchCalibrate();

    wait_for_start();
    press_start_button();

    //complete
    compost();

    //drive to apple pickup. 
        drive_forward_time(35.0,35.0,0.50);
        turn_left(90);
        drive_forward(10.0);
        turn_left(20);
        drive_forward(1.5);
        turn_right(20);
        drive_forward_time(28.0,28.0,4.0);
        drive_backward(7.5);
        turn_left(90);
    //pick up apple.
    apple_pickup_ramp();

    //move to lever
    drive_backward(9.0);
    turn_left(90);
    drive_forward(7.0);
    turn_right(45);

    //flip lever
    lever_flip();

    //drive to humidifier
    drive_backward(6.2);
    turn_left(45);

    //read and press the correct button
    humidifier();

}

void humidifier()
{
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);
    LCD.WriteLine("Humidifier: navigating...");

    // Drive forward to light color
    drive_forward(0.5); 

    // Read light color
    LightColor color = read_light_color();

    LCD.Clear(BLACK);
    if (color == RED_LIGHT)
    {
        LCD.SetFontColor(RED);
        LCD.WriteLine("Humidifier: RED");
    }
    else if (color == BLUE_LIGHT)
    {
        LCD.SetFontColor(BLUE);
        LCD.WriteLine("Humidifier: BLUE");
    }
    else
    {
        LCD.SetFontColor(WHITE);
        LCD.WriteLine("WARNING: no light detected!");
        LCD.WriteLine("Check CDS thresholds.");
        LCD.WriteLine("Skipping humidifier.");
        drive_backward(6.0);  // TODO: tune — back to start position
        return;
    }
    LCD.SetFontColor(WHITE);

    // ── Step 3 & 4: Offset to correct button and drive into it ───────────
    if (color == BLUE_LIGHT)
    {
        // Blue is on the LEFT — turn left, drive into button, back out
        LCD.WriteLine("Pressing BLUE (left)...");
        turn_left(90.0);                  // face the blue button
        drive_forward(3.0);               // TODO: tune — distance to button
        Sleep(0.2);                       // hold against button
        drive_backward(3.0);             // TODO: tune — back off button
        turn_right(90.0);                 // return to original heading
    }
    else if (color == RED_LIGHT)
    {
        // Red is on the RIGHT — turn right, drive into button, back out
        LCD.WriteLine("Pressing RED (right)...");
        turn_right(90.0);                 // face the red button
        drive_forward(3.0);               // TODO: tune — distance to button
        Sleep(0.2);                       // hold against button
        drive_backward(3.0);             // TODO: tune — back off button
        turn_left(90.0);                  // return to original heading
    }

    // ── Step 5: Back away from humidifier ────────────────────────────────
    drive_backward(6.0);   // TODO: tune — match Step 1 distance

    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);
    LCD.WriteLine("Humidifier: done.");
}
void apple_pickup_ramp()
{
    front_arm_servo.SetDegree(75);//arm down
    Sleep(1.0);
    drive_forward(3.2);//approach apples
    sweep_servo(78,(78+45),10,0.1); //pick up apples

    //MOVE TO DROP OFF APPLES
    drive_backward(3.0);
    turn_right(90);
    drive_backward(6);
    turn_right(90);
    drive_forward_time(40,40,4.0);
    drive_backward(0.9);
    turn_left(90);
    drive_forward(28.0);
    turn_right(90);
    drive_forward_time(40,40,2.0);
    drive_backward(8.0);
    turn_left(90);
    drive_forward_time(40,39,4.0);
    drive_backward(1.5);
    //DROP OFF APPLES
    front_arm_servo.SetDegree(64);
    Sleep(1.0);


}
void lever_flip()
{
    
    front_arm_servo.SetDegree(90);
    drive_forward(3.5);
    front_arm_servo.SetDegree(50);
    Sleep(1.0);
    //back up
    drive_backward(3.0);
    front_arm_servo.SetDegree(25);
    Sleep(5.0);
    drive_forward(3.0);
    front_arm_servo.SetDegree(80);
    Sleep(1.0);

}
void compost()
{
    drive_forward(5);
    turn_left(45);
    drive_forward(4);
    turn_left(90);
    Sleep(0.5);
    drive_forward_time(30.0f, 30.0f, 1.5f);
    Sleep(0.5);
    drive_backward(0.94);
    Sleep(0.5);
    turn_left(100);
    Sleep(0.5);
    drive_backward(2.9);
    turn_right(13);
    Sleep(0.5);
    drive_backward(3.1);
    rear_servo_motor.SetPercent(35);
    Sleep(4.0);
    rear_servo_motor.Stop();
    Sleep(0.5);
    rear_servo_motor.SetPercent(-35);
    Sleep(4.0);
    rear_servo_motor.SetPercent(35);
    Sleep(1.0);
    rear_servo_motor.Stop();
}
void window()
{



}
// Rest PID Function used before every drive function
void reset_pid()
{
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    float now = TimeNow();

    pid_right.past_error = 0.0f;
    pid_right.error_sum = 0.0f;
    pid_right.past_counts = 0;
    pid_right.past_time = now;
    pid_right.motor_power = TARGET_SPEED; 

    pid_left.past_error = 0.0f;
    pid_left.error_sum = 0.0f;
    pid_left.past_counts = 0;
    pid_left.past_time = now;
    pid_left.motor_power = TARGET_SPEED;

    Sleep(PID_SLEEP);
}

// Primary PID Function
float pid_adjustment(PIDState &state, int current_counts, float target_speed)
{
    // Step 1 & 2: Deltas
    int delta_counts = current_counts - state.past_counts;
    float delta_time = TimeNow() - state.past_time;
    if (delta_time <= 0.0f) return state.motor_power;

    // Step 3: Actual velocity in inches/second
    float actual_velocity = (delta_counts / COUNTS_PER_INCH) / delta_time;

    // Step 4: Error
    float error = target_speed - actual_velocity;

    // Step 5: Accumulate for I term (clamped to prevent windup)
    state.error_sum += error;
    if (state.error_sum >  50.0f) state.error_sum =  50.0f;
    if (state.error_sum < -50.0f) state.error_sum = -50.0f;

    // Step 6: Three terms
    float p_term = error * P_CONST;
    float i_term = state.error_sum * I_CONST;
    float d_term = (error - state.past_error) * D_CONST;

    // Step 7: Save state for next call
    state.past_error = error;
    state.past_counts = current_counts;
    state.past_time = TimeNow();

    // Step 8: Clamp and return updated power
    float new_power = state.motor_power + p_term + i_term + d_term;
    if (new_power < 5.0f) new_power = 5.0f;
    if (new_power > 100.0f) new_power = 100.0f;
    state.motor_power = new_power;
    return new_power;
}

// Function to stop motors at the same time
void stop_motors()
{
    right_motor.Stop();
    left_motor.Stop();
}

// Primary Drive Function
void drive_forward(float inches)
{
    int target_counts = (int)(inches * COUNTS_PER_INCH);
    reset_pid();

    while ((right_encoder.Counts() + left_encoder.Counts()) / 2 < target_counts)
    {
        pid_right.motor_power = pid_adjustment(pid_right,right_encoder.Counts(),TARGET_SPEED);
        pid_left.motor_power  = pid_adjustment(pid_left,left_encoder.Counts(),TARGET_SPEED);
        right_motor.SetPercent(pid_right.motor_power);
        left_motor.SetPercent (pid_left.motor_power);
        Sleep(PID_SLEEP);
    }

    stop_motors();
}

// Ramp Driving Function
void drive_forward_ramp(float inches)
{
    int target_counts = (int)(inches * COUNTS_PER_INCH);
    reset_pid();

    while ((right_encoder.Counts() + left_encoder.Counts()) / 2 < target_counts)
    {
        pid_right.motor_power = pid_adjustment(pid_right,right_encoder.Counts(),RAMP_TARGET_SPEED);
        pid_left.motor_power  = pid_adjustment(pid_left,left_encoder.Counts(),RAMP_TARGET_SPEED);
        right_motor.SetPercent(pid_right.motor_power);
        left_motor.SetPercent (pid_left.motor_power);
        Sleep(PID_SLEEP);
    }

    stop_motors();
}

// Primary Reverse Function
void drive_backward(float inches)
{
    int target_counts = (int)(inches * COUNTS_PER_INCH);
    reset_pid();

    while ((right_encoder.Counts() + left_encoder.Counts()) / 2 < target_counts)
    {
        pid_right.motor_power = pid_adjustment(pid_right,right_encoder.Counts(),TARGET_SPEED);
        pid_left.motor_power  = pid_adjustment(pid_left,left_encoder.Counts(),TARGET_SPEED);
        right_motor.SetPercent(-pid_right.motor_power);
        left_motor.SetPercent (-pid_left.motor_power);
        Sleep(PID_SLEEP);
    }

    stop_motors();
}

// Turn Left
void turn_left(float degrees)
{
    int target_counts = (int)(degrees * COUNTS_PER_DEGREE);
    reset_pid();

    while ((right_encoder.Counts() + left_encoder.Counts()) / 2 < target_counts)
    {
        int avg_counts = (right_encoder.Counts() + left_encoder.Counts()) / 2;
        float speed = (target_counts - avg_counts <= 20) ? TURN_SLOWDOWN_SPEED : TURN_TARGET_SPEED;
        pid_right.motor_power = pid_adjustment(pid_right,right_encoder.Counts(),speed);
        pid_left.motor_power  = pid_adjustment(pid_left,left_encoder.Counts(),speed);
        right_motor.SetPercent(pid_right.motor_power);
        left_motor.SetPercent (-pid_left.motor_power);
        Sleep(PID_SLEEP);
    }

    stop_motors();
}

// Turn Right
void turn_right(float degrees)
{
    int target_counts = (int)(degrees * COUNTS_PER_DEGREE);
    reset_pid();

    while ((right_encoder.Counts() + left_encoder.Counts()) / 2 < target_counts)
    {
        int avg_counts = (right_encoder.Counts() + left_encoder.Counts()) / 2;
        float speed = (target_counts - avg_counts <= 20) ? TURN_SLOWDOWN_SPEED : TURN_TARGET_SPEED;
        pid_right.motor_power = pid_adjustment(pid_right,right_encoder.Counts(),speed);
        pid_left.motor_power  = pid_adjustment(pid_left,left_encoder.Counts(),speed);
        right_motor.SetPercent(-pid_right.motor_power);
        left_motor.SetPercent (pid_left.motor_power);
        Sleep(PID_SLEEP);
    }

    stop_motors();
}

// Function that drives forward for a set time
void drive_forward_time(float percent_r,float percent_l, float seconds)
{
    right_motor.SetPercent(percent_r);
    left_motor.SetPercent (percent_l);
    Sleep(seconds);
    stop_motors();
}

// Function that drives backward for a set time
void drive_backward_time(float percent, float seconds)
{
    right_motor.SetPercent(-percent);
    left_motor.SetPercent (-percent);
    Sleep(seconds);
    stop_motors();
}

void follow_line(float timeout)
{
    float start = TimeNow();
    float last_error = 0.0f;

    while (TimeNow() - start < timeout)
    {
        // Read all three sensors
        float l = line_left.Value();
        float m = line_center.Value();
        float r = line_right.Value();

        // Determine which sensors are on the line
        bool left_on   = (l > LINE_THRESHOLD);
        bool center_on = (m > LINE_THRESHOLD);
        bool right_on  = (r > LINE_THRESHOLD);

        // Compute weighted error
        // Line to the left  → error negative → steer left
        // Line to the right → error positive → steer right
        // Centered (center only or all on) → error zero → straight
        float error = 0.0f;

        if (left_on   && !right_on) error = -1.0f; // line left of center
        if (right_on  && !left_on)  error =  1.0f; // line right of center
        if (left_on   &&  right_on) error =  0.0f; // straddling — straight
        if (center_on && !left_on && !right_on) error = 0.0f; // centered

        // Lost line — hold last correction to help reacquire
        if (!left_on && !center_on && !right_on) error = last_error;
        last_error = error;

        // Proportional steering correction
        float correction = LINE_KP * error;
        float left_power  = LINE_FOLLOW_POWER + correction;
        float right_power = LINE_FOLLOW_POWER - correction;

        // Clamp motor values to valid range
        if (left_power  >  100.0f) left_power  =  100.0f;
        if (left_power  < -100.0f) left_power  = -100.0f;
        if (right_power >  100.0f) right_power =  100.0f;
        if (right_power < -100.0f) right_power = -100.0f;

        left_motor.SetPercent (left_power);
        right_motor.SetPercent(right_power);

        Sleep(0.02f); 
    }

    stop_motors();
}

// Function that reads the light color
LightColor read_light_color()
{
    float min_val = CdS_cell.Value();
    for (int i = 0; i < 4; i++)
    {
        Sleep(0.02f);
        float v = CdS_cell.Value();
        if (v < min_val) min_val = v;
    }

    LCD.Write("CdS min: ");
    LCD.WriteLine(min_val);

    if (min_val < CDS_RED_MAX)  return RED_LIGHT;
    if (min_val < CDS_BLUE_MAX) return BLUE_LIGHT;
    else return NO_LIGHT;
}

// Waits for the start of the run.
void wait_for_start()
{
    while (read_light_color() != RED_LIGHT)
    {
        Sleep(0.02f);
    }
    LCD.WriteLine("GO!");
}

// Executes pressing the start button
void press_start_button()
{
    LCD.WriteLine("Pressing start button");
    drive_forward_time(25.0f, 25.0f, 0.4f);  // TODO: tune duration
    Sleep(0.1f);
    drive_backward_time(25.0f, 0.8f);
}

void sweep_servo(float start_deg, float end_deg, float step, float step_delay)
{
    LCD.Clear(BLACK);
    LCD.WriteLine("Sweeping servo...");
    LCD.Write("From: "); LCD.WriteLine(start_deg);
    LCD.Write("To:   "); LCD.WriteLine(end_deg);
    LCD.Write("Step: "); LCD.WriteLine(step);

    // Determine direction
    float direction = (end_deg > start_deg) ? 1.0 : -1.0;
    float current   = start_deg;

    front_arm_servo.SetDegree(current);
    Sleep(step_delay);

    while ((direction > 0 && current < end_deg) ||
           (direction < 0 && current > end_deg))
    {
        current += direction * step;

        // Don't overshoot end_deg
        if (direction > 0 && current > end_deg) current = end_deg;
        if (direction < 0 && current < end_deg) current = end_deg;

        front_arm_servo.SetDegree(current);

        LCD.Clear(BLACK);
        LCD.WriteLine("Sweeping servo...");
        LCD.Write("Current angle: ");
        LCD.WriteLine(current);

        Sleep(step_delay);
    }

    LCD.Clear(BLACK);
    LCD.WriteLine("Sweep complete.");
    LCD.Write("Final angle: ");
    LCD.WriteLine(current);
}
