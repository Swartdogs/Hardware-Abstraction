package frc.robot.abstraction;

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
}
