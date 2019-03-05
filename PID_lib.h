/**********************************************************************************************
 * Arduino PID Library - Version 1.0
 * by Vincent Politzer
 * 
 * inspired by Brett Beauregard's PID library
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#ifndef PID_LIB_H
#define PID_LIB_H

#define DEFAULT_PERIOD 100
#define DEFAULT_OUTPUT_MIN 0
#define DEFAULT_OUTPUT_MAX 255

enum Direction {Direct, Reverse};

class PID
{
    public:
        /* Constructor
        * ...Parameters:
        * ......double* input -- pointer to user's input
        * ......double* output -- pointer to user's output
        * ......double* setpoint -- pointer to user's setpoint
        * ......double initKp -- initial P gain value
        * ......double initKi -- initial I gain value
        * ......double initKd -- initial D gain value˘
        * ......double outMin -- minimum value for controller output
        * ......double outMax -- maximum value for controller output
        * ......Direction direction -- Direct for +input -> +output
        * ......                       Reverse for  +input -> -output
        * ......unsigned long period -- update period in ms
        * ...Returns:
        * ......Nothing
        */
        PID(double*, double*, double*,
            double, double, double,
            double, double, Direction,
            unsigned long period = DEFAULT_PERIOD);
        
        /* compute()
        * ...Computes the output value based on input, with PID.
        * ...Call every time loop() in your main file executes.
        * ...Only computes if the controller has been enabled with
        * ...start() AND period has elapsed.
        * ...Returns:
        * ......true if new output computed, false otherwise
        */
        bool compute();

        /* start()
        * ...Enables the PID controller.
        * ...Returns:
        * ......Nothing
        */
        void start();

        /* stop()
        * ...Disables the PID controller and the output.
        * ...Returns:
        * ......Nothing
        */
        void stop(); // Disable the PID controller


        /* setGains(...)
        * Sets the controller's P, I, and D gain values, optimizing
        * the I and D values to reduce number of computations during
        * control loop˘
        * ...Parameters:
        * ......double kp -- P gain value
        * ......double ki -- I gain value
        * ......double kd -- D gain value
        * ...Returns:
        * ......Nothing
        */
        void setGains(double, double, double);

        /* setPeriod(...)
        * Sets the period with which the calculation is performed 
        * and adjusts the I and D gains accordingly
        * ...Parameters:
        * ......unsigned long newSampleTime
        * ...Returns:
        * ......Nothing
        */
        void setPeriod(unsigned long);

        /* getKp()
        * ...Returns:
        * ......Current P gain value
        */
        double getKp();

        /* getKi()
        * ...Returns:
        * ......Current I gain value
        */
        double getKi();

        /* getKd()
        * ...Returns:
        * ......Current D gain value
        */
        double getKd();

        /* getDirection()
        * ...Returns:
        * ......Current controller direction
        * ......(Direct or Reverse)
        */
        Direction getDirection();

        /* isEnabled()
        * ...Returns:
        * ......true if controller is enabled, false otherwise
        */
        bool isEnabled();

    private:
        double clampToEndpoints(double); // clamps value to specified range

        /* The PID gains for the controller */
        double kp;
        double ki;
        double kd;

        /* The PID gains for returning to the user, original and un-optimized */
        double kpDisp;
        double kiDisp;
        double kdDisp;

        Direction direction; // Direct for +input -> +output
                            // Reverse for  +input -> -output

        /* Pointers to the user's input, output, and setpoint variables */
        double *controllerInput;
        double *controllerOutput;
        double *controllerSetpoint;
        
        unsigned long prevTime; // for checking if period has elapsed
        double errorSum; // this is the I-term, accumulated error
        double prevError; // for tracking the 

        unsigned long period; // update period in milliseconds
        double outMin, outMax; // range for clamping the output

        bool enabled;
};
#endif
