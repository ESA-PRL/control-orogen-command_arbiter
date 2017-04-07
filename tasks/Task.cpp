#include "Task.hpp"

using namespace command_arbiter;

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

    input_method = JOYSTICK;

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

    // Arbiter state transition based on the user input (button pressed)
    if(_raw_command.read(joystick_command) == RTT::NewData)
    {
        // Button (rising edge) detection
        if(joystick_command.buttonValue[B]  && !joystick_command_prev.buttonValue[B])
        {
            // Toggle between joystick and motion command input types
            input_method = input_method == JOYSTICK ? FOLLOWING : JOYSTICK;

            // When the input type is changed, but no commands are provided from the
            // new input the old command will continue to be applied, so a stop signal
            // must be safe in order to guarantee no unexpected behaviour
            base::commands::Motion2D stop_command;
            stop_command.translation = 0.0;
            stop_command.rotation = 0.0;
            _motion_command.write(stop_command);
        }
        joystick_command_prev = joystick_command;
    }

    // Read input motion commands
    if(_joystick_motion_command.readNewest(joystick_motion_command) == RTT::NewData && input_method == JOYSTICK)
    {
        _motion_command.write(joystick_motion_command);
    }

    if(_follower_motion_command.readNewest(follower_motion_command) == RTT::NewData && input_method == FOLLOWING)
    {
        _motion_command.write(follower_motion_command);
    }

    // Propagate the preference to the component state, must do it here as the state cannot be initialised in configureHook
    if(input_method == JOYSTICK && state() != JOYSTICK)
    {
        state(JOYSTICK);
    }
    else if(input_method == FOLLOWING && state() != FOLLOWING)
    {
        state(FOLLOWING);
    }
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
