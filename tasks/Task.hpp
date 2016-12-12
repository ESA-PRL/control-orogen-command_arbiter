/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef COMMAND_ARBITER_TASK_TASK_HPP
#define COMMAND_ARBITER_TASK_TASK_HPP

#include "command_arbiter/TaskBase.hpp"

namespace command_arbiter {

    // Button names are in order in which they appear in the vector
    enum ButtonName
    {
        X,
        A,
        B,
        Y,
        LB,
        RB,
        LT,
        RT,
        BACK,
        START,
        LJOY,
        RJOY
    };

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        controldev::RawCommand joystick_command;
        // Enable the joystick commands, disabling the path following
        bool joystickPreferred, newMotionCommand;
        // Storage variables for input motion command from joystick resp. path following component
        base::commands::Motion2D joystick_motion_command, follower_motion_command;
        

    public:
        Task(std::string const& name = "command_arbiter::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
    	~Task();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif

