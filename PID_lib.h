/**********************************************************************************************
 * Arduino PID Library - Version 1.0
 * by Vincent Politzer
 * 
 * Special thanks to Dan Landau
 *
 **********************************************************************************************/

#ifndef PID_LIB_H
#define PID_LIB_H

#define DEFAULT_PERIOD 100
#define DEFAULT_OUTPUT_MIN 0
#define DEFAULT_OUTPUT_MAX 255

#define INT_RESOLUTION 100

enum Direction {Direct, Reverse};

class PID
{
    public:
        /* Constructor
        * ...Parameters:
        * ......int16_t kp -- P gain value
        * ......int16_t ki -- I gain value
        * ......int16_t kd -- D gain value
        * ......int16_t outMin -- minimum value for controller output
        * ......int16_t outMax -- maximum value for controller output
        * ......unsigned long period -- update period in ms
        * ...Returns:
        * ......Nothing
        */
        PID(int16_t, int16_t, int16_t,
            int16_t, int16_t,
            unsigned long period = DEFAULT_PERIOD);
        
        /* compute()
        * ...Computes the output value based on input, with PID.
        * ...Call every time loop() in your main file executes.
        * ...Only computes if the controller has been enabled with
        * ...start() AND period has elapsed.
        * ...Returns:
        * ......computed output value
        */
        int16_t compute(double);

        void set(double);

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

        /* isEnabled()
        * ...Returns:
        * ......true if controller is enabled, false otherwise
        */
        bool isEnabled();

    private:
        /* The PID gains for the controller */
        int16_t k1;
        int16_t k2;
        int16_t k3;

        int16_t pidOutput;
        int16_t pidSetpoint;
        
        unsigned long prevTime; // for checking if period has elapsed
        int16_t e1, e2, e3; 

        unsigned long period; // update period in milliseconds
        int16_t outMin, outMax; // range for clamping the output

        bool enabled;
};
#endif
