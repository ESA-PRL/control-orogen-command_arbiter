/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace command_arbiter;

Task::Task(std::string const& name)
    : TaskBase(name),
      joystickPreferred(true),
      newMotionCommand(false)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
      joystickPreferred(true),
      newMotionCommand(false)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
    state(JOYSTICK);
}
void Task::updateHook()
{
    TaskBase::updateHook();
    controldev::RawCommand joystick_command_prev;
    joystick_command_prev = joystick_command;

    // Arbiter state transition based on the user input (button pressed)
    if(_raw_command.read(joystick_command) == RTT::NewData)
    {
        // Button (rising edge) detection:
        if(  joystick_command.buttonValue[B]  &&    // Button is pressed down now
            !joystick_command_prev.buttonValue[B])  // and was not pressed previously 
        {
            // Invert the value of the joystickPreferred
            joystickPreferred = !joystickPreferred;

            // Propagate the preference to the component state
            if(joystickPreferred){
                state(JOYSTICK);
            } else {
                state(FOLLOWING);
            }
        }
    }

    // Read input motion commands
    if(_joystick_motion_command.readNewest(joystick_motion_command) == RTT::NewData ){
        newMotionCommand = true;
    }
    if(_follower_motion_command.readNewest(follower_motion_command) == RTT::NewData ){
        newMotionCommand = true;   
    }

    // Selected motion command, to be forwarded to locomotion control
    if(newMotionCommand){
        if(joystickPreferred){
            _motion_command.write(joystick_motion_command);
        }
        else {
            _motion_command.write(follower_motion_command);
        }
        newMotionCommand = false;   
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
