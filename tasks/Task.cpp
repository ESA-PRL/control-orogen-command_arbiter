#include "Task.hpp"

using namespace command_arbiter;
using namespace locomotion_switcher;

Task::Task(std::string const& name)
    : TaskBase(name)
{

}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{

}

bool Task::configureHook()
{
    if(!TaskBase::configureHook())
    {
        return false;
    }

    locomotion_mode = LocomotionMode::DRIVING;

    input_method = FOLLOWING;
    last_state = FOLLOWING;

    return true;
}
bool Task::startHook()
{
    if(!TaskBase::startHook())
    {
        return false;
    }
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    // stop in case a fault is detected
    bool fault_detected = false;
    if(_fault_detected.read(fault_detected) == RTT::NewData)
    {
        if (fault_detected && (state() != EMERGENCY) && state() != JOYSTICK)
        {
            last_state = state();
            sendStopCommand();
            state(EMERGENCY);
        }
        else if (!fault_detected && (state() == EMERGENCY))
        {
            state(last_state);
        }
    }

    // Arbiter state transition based on the user input (button pressed)
    if(_raw_command.read(joystick_command) == RTT::NewData)
    {
        //joystick_buttons = joystick_command.buttonValue;

        // Button (rising edge) detection:
        if(  joystick_command.buttons["BTN_C"]  &&    // Button is pressed down now // FIXME: Because of wrong button mapping in the system driver BTN_B is actually BTN_C
            !joystick_command_prev.buttons["BTN_C"])  // and was not pressed previously
        {
            // Toggle between joystick and motion command input types
            input_method = input_method == JOYSTICK ? FOLLOWING : JOYSTICK;
        }
        //if (input_method == JOYSTICK)  // uncomment if mode should not be selected by joystick in Following mode
        {
            // Button A (rising edge) detection:
            if(  joystick_command.buttons[3]  &&    // Button is pressed down now // FIXME: Because of wrong button mapping in the system driver BTN_B is actually BTN_C
                !joystick_command_prev.buttons[3])  // and was not pressed previously
            {
                // Toggle between joystick and motion command input types
                if (locomotion_mode == LocomotionMode::DRIVING)
                    locomotion_mode = LocomotionMode::WHEEL_WALKING;
                else if (locomotion_mode == LocomotionMode::WHEEL_WALKING)
                    locomotion_mode = LocomotionMode::DRIVING;
                _locomotion_mode.write(locomotion_mode);
            }
        }
        joystick_command_prev = joystick_command;
    }

    // trap in emergency state
    if (state() == EMERGENCY)
    {
        if (input_method == FOLLOWING)
            return;
    }

    // Read input motion commands
    if(_joystick_motion_command.read(joystick_motion_command) == RTT::NewData && input_method == JOYSTICK)
    {
        _motion_command.write(joystick_motion_command);
    }

    if(_follower_motion_command.read(follower_motion_command) == RTT::NewData && input_method == FOLLOWING)
    {
        _motion_command.write(follower_motion_command);
    }

    // Propagate the preference to the component state, must do it here as the state cannot be initialised in configureHook
    if(input_method == JOYSTICK && state() != JOYSTICK)
    {
        state(JOYSTICK);
        // When the input type is changed, but no commands are provided from the
        // new input the old command will continue to be applied, so a stop signal
        // must be safe in order to guarantee no unexpected behaviour
        sendStopCommand();
    }
    else if(input_method == FOLLOWING && state() != FOLLOWING)
    {
        state(FOLLOWING);
        _motion_command.write(follower_motion_command);
        _locomotion_mode.write(LocomotionMode::DONT_CARE);
    }
}

void Task::sendStopCommand()
{
    base::commands::Motion2D stop_command;
    stop_command.translation = 0.0;
    stop_command.rotation = 0.0;
    _motion_command.write(stop_command);
    locomotion_mode = LocomotionMode::DRIVING;
    _locomotion_mode.write(locomotion_mode);
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
