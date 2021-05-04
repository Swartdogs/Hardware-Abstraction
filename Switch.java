package frc.robot.abstraction;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.abstraction.Enumerations.State;

public abstract class Switch implements Abstraction
{
    private State _previousState;
    private State _currentState;

    protected abstract State getRaw();

    public abstract void whenActivated(Command command);
    public abstract void whileActive(Command command);
    public abstract void cancelWhenActivated(Command command);

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
