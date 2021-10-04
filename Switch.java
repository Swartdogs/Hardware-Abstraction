package frc.robot.abstraction;

import java.util.HashMap;
import java.util.HashSet;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.abstraction.Enumerations.State;

public abstract class Switch implements Abstraction
{
    private State _previousState;
    private State _currentState;

    protected abstract State getRaw();

    public abstract void whenActivated(SwartdogCommand command, boolean interruptible);
    public abstract void whileActive(SwartdogCommand command, boolean interruptible);
    public abstract void cancelWhenActivated(SwartdogCommand command);

    public void whenActivated(SwartdogCommand command)
    {
        whenActivated(command, true);
    }

    public void whileActive(SwartdogCommand command)
    {
        whileActive(command, true);
    }

    public State get()
    {
        return _currentState;
    }

    public boolean transitionedTo(State state)
    {
        return _currentState == state && _previousState != state;
    }

    public void cache()
    {
        _previousState = _currentState;
        _currentState = getRaw();

        if (_previousState == null)
        {
            _previousState = _currentState;
        }
    }

    protected void schedule(SwartdogCommand command)
    {
        if (SwartdogSubsystem.REGISTER_SUBSYSTEMS)
        {
            CommandScheduler.getInstance().schedule(command);
        }
    }

    public static Switch genericSwitch(Supplier<State> stateSupplier)
    {
        return new HardwareSwitch()
        {
            @Override
            protected State getRaw()
            {
                return stateSupplier.get();
            }
        };
    }

    public static Switch lightSensor(int port)
    {
        return new HardwareSwitch()
        {
            private DigitalInput _lightSensor = new DigitalInput(port);

            @Override
            protected State getRaw()
            {
                return _lightSensor.get() ? State.Off : State.On;
            }
        };
    }

    public static abstract class HardwareSwitch extends Switch
    {
        private HashSet<SwartdogCommand> _whenActivatedCommands = new HashSet<SwartdogCommand>();
        private HashSet<SwartdogCommand> _whileActiveCommands   = new HashSet<SwartdogCommand>();
        private HashSet<SwartdogCommand> _cancelCommands        = new HashSet<SwartdogCommand>();

        private HashMap<SwartdogCommand, Boolean> _interruptibleMap = new HashMap<SwartdogCommand, Boolean>();

        @Override
        public void whenActivated(SwartdogCommand command, boolean interruptible)
        {
            _whenActivatedCommands.add(command);
            _interruptibleMap.put(command, interruptible);
        }

        @Override
        public void whileActive(SwartdogCommand command, boolean interruptible)
        {
            _whileActiveCommands.add(command);
            _interruptibleMap.put(command, interruptible);
        }

        @Override
        public void cancelWhenActivated(SwartdogCommand command)
        {
            _cancelCommands.add(command);
        }

        @Override
        public void cache()
        {
            super.cache();

            CommandScheduler scheduler = CommandScheduler.getInstance();

            if (transitionedTo(State.On))
            {
                for (SwartdogCommand command : _whenActivatedCommands)
                {
                    scheduler.schedule(_interruptibleMap.get(command), command);
                }

                for (SwartdogCommand command : _whileActiveCommands)
                {
                    scheduler.schedule(_interruptibleMap.get(command), command);
                }
            }

            else if (transitionedTo(State.Off))
            {
                for (SwartdogCommand command : _whileActiveCommands)
                {
                    if (scheduler.isScheduled(command))
                    {
                        scheduler.cancel(command);
                    }
                }

                for (SwartdogCommand command : _cancelCommands)
                {
                    if (scheduler.isScheduled(command))
                    {
                        scheduler.cancel(command);
                    }
                }
            }
        }
    }
}
