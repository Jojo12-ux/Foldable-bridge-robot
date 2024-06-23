#include "mbed.h"

// pes board pin map
#include "pm2_drivers/PESBoardPinMap.h"

// drivers
#include "pm2_drivers/DebounceIn.h"

// servo motors
#include "pm2_drivers/Servo.h"

// PWM drivers
#include "pm2_drivers/FastPWM/FastPWM.h"

//Motors operate in a closed loop to control the velocity
#include "pm2_drivers/DCMotor.h"
#include <cstdint>

// DEFINITION OF VARIABLES

// Number of revolutions to 90 degree turn
const float FIRST_FORWARD_REVOLUTIONS = 3;
// Number of revolutions required to make the 90 degrees
const float REV_TO_MAKE_TURN = 1;
// Number of revolutions to table edge
const float SECOND_FORWARD_REVOLUTIONS = 6;
// Number of revolutions required to extend bridge
const float REVOLUTIONS_TO_EXTEND_BRIDGE = 10;
// Number of revolutions to retract bridge
const float REVOLUTIONS_TO_RETRACT_BRIDGE = 5;
// Number of reverse revolutions
const float REVERSE_REVOLUTIONS = 3;
// Number of revolutions to cross
const float REVOLUTIONS_TO_CROSS = 10;
// Number of revolutions to complete task
const float REVOLUTIONS_TO_COMPLETE_TASK = 15;

// this variable will be toggled via the user button (blue button) and 
// decides whether to execute the main task or not
bool do_execute_main_task = false; 

// this variable is used to reset certain variables and objects and 
// shows how you can run a code segment only once                                 
bool do_reset_all_once = false; 

// objects for user button (blue button) handling on nucleo board
// create DebounceIn object to evaluate the user button
// falling and rising edge
DebounceIn user_button(USER_BUTTON); 
// custom function which is getting executed when user
// button gets pressed, definition below
void toggle_do_execute_main_fcn();   
  
// create object to enable power electronics for the DC motors
DigitalOut enable_motors(PB_ENABLE_DCMOTORS); 

// maximum voltage of battery packs
const float voltage_max =12.0f;

// MOTOR 1
const float gear_ratio_M1 = 100.0f; // gear ratio
const float kn_M1 = 140.0f / 12.0f;  //  [rpm/V]
DCMotor motor_M1(PB_PWM_M1, PB_ENC_A_M1, PB_ENC_B_M1, gear_ratio_M1, kn_M1, voltage_max);

// MOTOR  2
const float gear_ratio_M2 = 100.0f; // gear ratio
const float kn_M2 = 140.0f / 12.0f;  //  [rpm/V]
DCMotor motor_M2(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_M2, kn_M2, voltage_max);

// MOTOR 3

const float gear_ratio_M3 = 250.0f; // gear ratio
const float kn_M3 = 57.0f / 12.0f;  //  [rpm/V]
DCMotor motor_M3(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_M3, kn_M3, voltage_max);

// ir distance sensor
// define a variable to store measurement (in mV & cm)
float ir_distance_mV = 0.0f; 
float ir_distance_cm = 0.0f; 

// SERVO MOTORS

Servo servo_D0(PB_D0);
float servo_input = 0.0f;
// define servo counter, this is an additional variable
// used to command the servo
int servo_counter = 0; 
// minimal pulse width and maximal pulse width obtained from the servo calibration process
// reely S0090
float servo_D0_ang_min = 0.0325f;
float servo_D0_ang_max = 0.1175f;

// Translation - forward and reverse
uint8_t translation_motion(float revolutions, float direction, float* position_M1, float* position_M2, float M1_compensation, float M2_compensation);
float FORWARD = 1;
float REVERSE = 0;

// Motor positions to initiate turn
float position_M1;
float position_M2;

// Rotation - making the 90 degrees turn
uint8_t rotational_motion(float postion_M1, float position_M2, float num_rev);

// Extending bridge at the edge of the table
uint8_t extend_bridge (float rev_extension);

// Retracting bridge after crossing the gap
uint8_t retract_bridge (float rev_retraction);

// Definition and declaration of variables
uint8_t turn_reached = 0;
uint8_t turn_completed = 0;
uint8_t reached_50mm_from_edge = 0;
uint8_t bridge_extended = 0;
uint8_t bridge_crossed = 0;
uint8_t bridge_retracted = 0;
uint8_t task_completed = 0;
uint8_t reverse_completed = 0;

// main() runs in its own thread in the OS
int main()
{
        // STATE MACHINE

       // set up states for state machine
        enum RobotState {
            INITIAL,
            FIRST_FORWARD_Xmm,
            MAKING_90_DEGREES_TURN,
            SECOND_FORWARD_Xmm,
            EXTEND_BRIDGE,
            REVERSE_Xmm,
            CROSS_BRIDGE,
            RETRACT_BRIDGE,
            THIRD_FORWARD_Xmm,
            BUZZER_SLEEP
        } robot_state = RobotState::INITIAL;

    // TIMER

    // while loop gets executed every main_task_period_ms milliseconds, this is a
    // simple approach to repeatedly execute main
    // define main task period time in ms e.g. 20 ms, therefore
    // the main task will run 50 times per second
    const int main_task_period_ms = 200; 
    // create Timer object which we use to run the main task 
    // every main_task_period_ms                             
    Timer main_task_timer;              
                                       
    const int loops_per_seconds = static_cast<int>(ceilf(1.0f / (0.001f * static_cast<float>(main_task_period_ms))));

    // USER BUTTON

    // attach button fall function address to user button object, button has a pull-up resistor
    user_button.fall(&toggle_do_execute_main_fcn);

    // ADDITIONAL LED

    // create DigitalOut object to command extra led, you need to add an aditional resistor, e.g. 220...500 Ohm
    // a led has an anode (+) and a cathode (-), the cathode needs to be connected to ground via a resistor
    DigitalOut led1(PB_9);
    // led1 turned off
    led1 = 0;

    // MOTOR M1

    // enable the motion planner for smooth movement
    motor_M1.enableMotionPlanner(true);
    // limit max. velocity to half physical possible velocity
    motor_M1.setMaxVelocity(motor_M1.getMaxPhysicalVelocity() * 0.5f);
    // limit max. acceleration to half of the default acceleration
    motor_M1.setMaxAcceleration(motor_M1.getMaxAcceleration() * 0.5f);
    //motor_M1.setVelocity(motor_M1.getMaxVelocity() * 0.5jf); // set speed setpoint to half physical possible velocity

    // MOTOR M2

    // enable the motion planner for smooth movement
    motor_M2.enableMotionPlanner(true);
     // limit max. velocity to half physical possible velocity
    motor_M2.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // limit max. acceleration to half of the default acceleration
    motor_M2.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);
    // set speed setpoint to half physical possible velocity
    //motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.5jf); 

    // MOTOR 3

    // enable the motion planner for smooth movement
    motor_M3.enableMotionPlanner(true);
     // limit max. velocity to half physical possible velocity
    motor_M3.setMaxVelocity(motor_M2.getMaxPhysicalVelocity() * 0.5f);
    // limit max. acceleration to half of the default acceleration
    motor_M3.setMaxAcceleration(motor_M2.getMaxAcceleration() * 0.5f);
    // set speed setpoint to half physical possible velocity
    //motor_M2.setVelocity(motor_M2.getMaxVelocity() * 0.5jf); 
          
    // ANALOG IR DISTANCE SENSOR

    // create AnalogIn object to read in the infrared distance sensor
    // 0...3.3V are mapped to 0...1
    AnalogIn ir_analog_in(PC_2);                        
    // function declaration, definition at the end
    float ir_sensor_compensation(float ir_distance_mV);
     // min and max analog IR sensor reading, (ir_distance_min, ir_distance_max) -> (servo_min, servo_max)
    float ir_distance_min = 4.0f;
    float ir_distance_max = 30.0f;



     // start timer
    main_task_timer.start();

    while (true) {
        main_task_timer.reset();

        if (do_execute_main_task) {

            // LED

            // toggling the user led
            led1 = !led1;

            // SERVO MOTORS
             // enable the servos and move it to zero
            if (!servo_D0.isEnabled())
            servo_D0.enable();
           // enable the servo and move it to the center
             if (!servo_D0.isEnabled()) {
            servo_D0.enable(1.0f);
               } 
            // command the servos
            servo_D0.setNormalisedPulseWidth(servo_input);
            //servo_D0.calibratePulseMinMax(0, 180);
            servo_D0.calibratePulseMinMax(servo_D0_ang_min, servo_D0_ang_max);
            // default acceleration of the servo motion profile is 1.0e6f
             servo_D0.setMaxAcceleration(0.3f);

             // ANALOG IR POSITION SENSOR

            // read analog input
                ir_distance_mV = 1.0e3f * ir_analog_in.read() * 3.3f;
                float ir_distance_cm_candidate = ir_sensor_compensation(ir_distance_mV);
                // read us sensor distance, only valid measurements will update us_distance_cm
                if (ir_distance_cm_candidate > 0.0f) {
                ir_distance_cm = ir_distance_cm_candidate;

             // STATE MACHINE

             switch (robot_state) {
                case RobotState::INITIAL:
                {
                    printf("INITIAL\n");
                    // enable hardwaredriver dc motors: 0 -> disabled, 1 -> enabled
                    // setting this once would actually be enough
                    enable_motors = 1;
                    robot_state =  RobotState::FIRST_FORWARD_Xmm; 

                    break;
                }

                case RobotState::FIRST_FORWARD_Xmm:
                {
                    printf("FIRST_FORWARD_Xmm\n");
                    turn_reached = translation_motion(FIRST_FORWARD_REVOLUTIONS, FORWARD, &position_M1, &position_M2, REV_TO_MAKE_TURN, REV_TO_MAKE_TURN);
                    
                    if (turn_reached)
                    {
                       robot_state =  RobotState::MAKING_90_DEGREES_TURN;
                    }
                      
                    break;
                }

                case RobotState::MAKING_90_DEGREES_TURN:
                {
                    printf("MAKING_90_DEGREES_TURN\n");
                  turn_completed =  rotational_motion(position_M1, position_M2, REV_TO_MAKE_TURN);
                   if (turn_completed)
                    {
                       robot_state =  RobotState::SECOND_FORWARD_Xmm;
                    }
                        
                    break;
                }

                case RobotState::SECOND_FORWARD_Xmm:
                {
                 
                    printf("SECOND_FORWARD_Xmm\n");

                    /* ADD AN ANALOG IR POSITION SENSOR*/

                    reached_50mm_from_edge = translation_motion(SECOND_FORWARD_REVOLUTIONS, FORWARD, &position_M1, &position_M2,REV_TO_MAKE_TURN, -REV_TO_MAKE_TURN );
                    if (reached_50mm_from_edge)
                    {
                        reached_50mm_from_edge = 0;
                        robot_state =  RobotState::EXTEND_BRIDGE;
                       
                    }
                    
                    break;
                }
                case RobotState::EXTEND_BRIDGE:
                    printf("EXTEND_BRIDGE\n");

                    bridge_extended = extend_bridge(REVOLUTIONS_TO_EXTEND_BRIDGE);
                    if (bridge_extended)
                    {
                        bridge_extended = 0;
                        robot_state =  RobotState::REVERSE_Xmm;
                    }
                    
                    break;

                case RobotState::REVERSE_Xmm:
                    printf("REVERSE_Xmm\n");
                    reverse_completed = translation_motion(REVERSE_REVOLUTIONS, REVERSE, &position_M1, &position_M2, REV_TO_MAKE_TURN, -REV_TO_MAKE_TURN);
                    if (reverse_completed)
                    {
                        reverse_completed = 0;
                        robot_state =  RobotState::CROSS_BRIDGE;
                    }
                   
                    break;

                case RobotState::CROSS_BRIDGE:
                    printf("CROSS_BRIDGE\n");
                     bridge_crossed = translation_motion(REVOLUTIONS_TO_CROSS, FORWARD, &position_M1, &position_M2, REV_TO_MAKE_TURN, -REV_TO_MAKE_TURN);
                    if (bridge_crossed)
                    {
                        bridge_crossed = 0;
                        robot_state =  RobotState::RETRACT_BRIDGE;
                    }

                    break;

                case RobotState::RETRACT_BRIDGE:
                    printf("RETRACT_BRIDGE\n");
                     bridge_retracted = retract_bridge(REVOLUTIONS_TO_RETRACT_BRIDGE);
                    if (bridge_retracted)
                    {
                        bridge_retracted = 0;
                        robot_state =  RobotState::THIRD_FORWARD_Xmm;
                    }
                    break;

                case RobotState::THIRD_FORWARD_Xmm:
                    printf("THIRD_FORWARD_Xmm\n");
                     task_completed = translation_motion(REVOLUTIONS_TO_COMPLETE_TASK, FORWARD, &position_M1, &position_M2, REV_TO_MAKE_TURN, -REV_TO_MAKE_TURN);
                    if (task_completed)
                    {
                        task_completed = 0;
                        robot_state =  RobotState::BUZZER_SLEEP;
                    }
                    break;

                case RobotState::BUZZER_SLEEP:
                    printf("BUZZER_SLEEP\n");

                    break;
                default:
                    break;
            }

        }

        else{
            // the following code block gets executed only once
            if (do_reset_all_once) {
                do_reset_all_once = false;

                // reset variables and objects
                led1 = 0;
                enable_motors = 0;
                ir_distance_cm = 0.0f;
                motor_M1.enableMotionPlanner(true);
                robot_state = RobotState::INITIAL;
                 }
    
        }

        

        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);
      }

   }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;

    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
}

float ir_sensor_compensation(float ir_distance_mV)
{
    // insert values that you got from the MATLAB file
    static const float a = 2.574e+04f;
    static const float b = -29.37f;

    // avoid division by zero by adding a small value to the denominator
    if (ir_distance_mV + b == 0.0f)
        ir_distance_mV -= 0.001f;

    return a / (ir_distance_mV + b);
}

uint8_t translation_motion(float revolutions, float direction, float* position_M1, float* position_M2, float M1_compensation, float M2_compensation)
{
    motor_M1.setRotation(revolutions + M1_compensation);
    motor_M2.setRotation(revolutions + M2_compensation);
    //printf(" DC Motor1 Rotations: %f\n", motor_M1.getRotation());
    //printf(" DC Motor2 Rotations: %f\n", motor_M2.getRotation());

    if (direction == 1)
    {
        if ((motor_M1.getRotation() >= revolutions + M1_compensation) && (motor_M2.getRotation() >= revolutions + M2_compensation))
        {

            *position_M1 = motor_M1.getRotation();
            *position_M2 = motor_M2.getRotation();

            return 1;
        
        }
        
    }
    if (direction == 0)
    {
      if ((motor_M1.getRotation() <= revolutions + M1_compensation) && (motor_M2.getRotation() <= revolutions + M2_compensation))
        {   
            *position_M1 = motor_M1.getRotation();
            *position_M2 = motor_M2.getRotation();

            return 1;
        }
      
    }
    return 0;
}

uint8_t rotational_motion(float position_M1, float position_M2, float num_rev)
{
    motor_M1.setRotation(position_M1 + num_rev);
    motor_M2.setRotation(position_M2 - num_rev);
   // printf(" DC Motor1 Rotations: %f\n", motor_M1.getRotation());
   // printf(" DC Motor2 Rotations: %f\n", motor_M2.getRotation());

    if ((motor_M1.getRotation() >= position_M1 + num_rev) && (motor_M2.getRotation() <= position_M2 - num_rev))
    {
        return 1;
    }
    return 0;
}

uint8_t extend_bridge (float rev_extension)
{

    motor_M3.setRotation(rev_extension/2); // CHANGE BACK TO MOTOR 2 TODO@
    // printf(" DC Motor Rotations: %f\n", motor_M2.getRotation());
    //  printf(" rev_extension :%f", rev_extension);
    if ((motor_M3.getRotation()>= 4.9f))  //@TODO.. find the issue
    {
        return 1;
    }

    return 0;
}

uint8_t retract_bridge (float rev_retraction)
{
    motor_M3.setRotation(rev_retraction/2);
     printf(" DC Motor Rotations: %f\n", motor_M3.getRotation());
    if ((motor_M3.getRotation() <= 2.51f))
    {
        return 1;
    }
    return 0;
}