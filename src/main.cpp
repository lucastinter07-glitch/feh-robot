/* ------------------------------*/
/* FEH ROBOT PROJECT TEAM H3     */
/* Lucas Tinter, Will Castlen,   */
/* Hayden Yang, Faiza Choudhry   */
/* 3/30/26  v2.0.0               */
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
DigitalEncoder right_encoder(FEHIO::Pin8); // Right Motor Shaft Encoder
DigitalEncoder left_encoder(FEHIO::Pin10); // Left Motor Shaft Encoder

AnalogInputPin CdS_cell(FEHIO::Pin1); // CdS Cell Sensor

AnalogInputPin line_left(FEHIO::Pin2); // Left Optosensor
AnalogInputPin line_center(FEHIO::Pin3); // Center Optosensor
AnalogInputPin line_right(FEHIO::Pin4); // Right Optosensor

// General Variable Declarations
#define PI 3.1415
#define g 9.8

// Driving Variable Declarations
#define DRIVE_PERCENT 20
#define RAMP_PERCENT 40
#define TURN_PERCENT 20
#define COUNTS_PER_INCH 40.489
#define COUNTS_PER_DEGREE 2.88

// Line Following Variable Declarations
#define LINE_THRESHOLD 3.5f
#define LINE_BASE_SPEED 15             
#define LINE_KP 30             
#define LINE_TIMEOUT 10.0f
#define LINE_FOLLOW_POWER    20.0f          // TODO: tune base power (%)

// PID Constant Variable Declarations
#define P_CONST 0.75f
#define I_CONST 0.05f             
#define D_CONST 0.25f             
#define PID_SLEEP 0.15f             
#define TARGET_SPEED 10.0f

// Cds Cell Sensor Value Declarations
#define CDS_RED_MAX 1.0f
#define CDS_BLUE_MAX 2.0f

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
void drive_forward_time(float percent, float seconds);
void drive_backward_time(float percent, float seconds);
void stop_motors();
LightColor read_light_color();
void wait_for_start();
void press_start_button();

// Execution of Tasks
void ERCmain()
{
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);
    LCD.WriteLine("Waiting To Start");

    wait_for_start();
    press_start_button();

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
    if (new_power <   5.0f) new_power =   5.0f;
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
        pid_right.motor_power = pid_adjustment(pid_right,right_encoder.Counts(),TARGET_SPEED);
        pid_left.motor_power  = pid_adjustment(pid_left,left_encoder.Counts(),TARGET_SPEED);
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
        pid_right.motor_power = pid_adjustment(pid_right,right_encoder.Counts(),TARGET_SPEED);
        pid_left.motor_power  = pid_adjustment(pid_left,left_encoder.Counts(),TARGET_SPEED);
        right_motor.SetPercent(-pid_right.motor_power);
        left_motor.SetPercent (pid_left.motor_power);
        Sleep(PID_SLEEP);
    }

    stop_motors();
}

// Function that drives forward for a set time
void drive_forward_time(float percent, float seconds)
{
    right_motor.SetPercent( percent);
    left_motor.SetPercent ( percent);
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
    float start       = TimeNow();
    float last_error  = 0.0f;

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

        // Optional: display sensor values during testing
        // LCD.Clear(BLACK);
        // LCD.Write("L:"); LCD.Write(l);
        // LCD.Write(" M:"); LCD.Write(m);
        // LCD.Write(" R:"); LCD.WriteLine(r);

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

        Sleep(0.02f); // 20 ms loop tick
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

void wait_for_start()
{
    while (read_light_color() != RED_LIGHT)
    {
        Sleep(0.02f);
    }
    LCD.WriteLine("GO!");
}

void press_start_button()
{
    LCD.WriteLine("Pressing start button");
    drive_forward_time(25.0f, 0.4f);  // TODO: tune duration
    Sleep(0.1f);
    drive_backward_time(25.0f, 0.4f);
}