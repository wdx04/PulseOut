#include "mbed.h"
#include "stm32lib.h"
#include "PulseOut.h"

// Micro Step: 1/8 (OFF/ON/OFF)
// Current Limit: Max(OFF/OFF/OFF)

#define USE_OPM_PULSEOUT 1

using namespace stm32; // for DebouncedButton class

EventQueue event_queue;

DigitalOut motor_en(D0, 0); // Effective on low voltage
DigitalOut motor_dir(D1, 0);
DebouncedButton button(BUTTON1);
#if USE_OPM_PULSEOUT
OPMPulseOut pulse_out(D6);
#else
TickerPulseOut *pulse_out = TickerPulseOut::get_instance();
#endif

// if the button was pressed, move the skid platform rightwards by 1000 steps
// if the button was not pressed, move the skid platform leftwards by 1000 steps
void check_buttons()
{
    if(button.query_and_reset())
    {
        motor_en = 0; // Enable drive
        printf("Sending pulses to driver...\n");
        motor_dir = 1; // Direction: Rightward
        wait_us(10); // Wait after direction change
#if USE_OPM_PULSEOUT
        pulse_out.write_us(4/*period*/, 2/*width*/, 1000/*count*/);
#else
        pulse_out->write_us(D2/*output pin*/, 4/*period*/, 2/*width*/, 1000/*count*/);
        ThisThread::sleep_for(5ms);
#endif
        motor_en = 1; // Disable drive
    }
    else
    {
        motor_en = 0; // Enable Drive
        printf("Sending reverse pulses to driver...\n");
        motor_dir = 0; // Direction: Leftward
        wait_us(10); // Wait after direction change
#if USE_OPM_PULSEOUT
        pulse_out.write_us(4/*period*/, 2/*width*/, 1000/*count*/);
#else
        pulse_out->write_us(D2/*output pin*/, 4/*period*/, 2/*width*/, 1000/*count*/);
        ThisThread::sleep_for(5ms);
#endif
        motor_en = 1; // Disable Drive
    }

}

int main()
{
    sleep_manager_lock_deep_sleep();

    printf("Application started.\n");

    event_queue.call_every(1s, &check_buttons);
    event_queue.dispatch_forever();
}
