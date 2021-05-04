package frc.robot.abstraction;

import frc.robot.abstraction.Enumerations.ExtendState;

public abstract class Solenoid implements Abstraction
{
    private ExtendState _state;

    protected abstract ExtendState getRaw();

    public abstract void extend();
    public abstract void retract();

    public ExtendState get()
    {
        return _state;
    }
    
    public void set(ExtendState state)
    {
        switch (state)
        {
            case Extended:
                extend();
                break;

            case Retracted:
                retract();
                break;

            default:
                break;
        }
    }

    public void cache()
    {
        _state = getRaw();
    }

    public static Solenoid compose(Solenoid... solenoids)
    {
        return new Solenoid()
        {
            private ExtendState _state;

            @Override
            public void extend()
            {
                _state = ExtendState.Extended;

                for (Solenoid s : solenoids)
                {
                    s.extend();
                }
            }

            @Override
            public void retract()
            {
                _state = ExtendState.Retracted;

                for (Solenoid s : solenoids)
                {
                    s.retract();
                }
            }

            @Override
            protected ExtendState getRaw()
            {
                return _state;
            }
        };
    }

    public static Solenoid invert(Solenoid solenoid)
    {
        return new Solenoid()
        {
            @Override
            public void extend()
            {
                solenoid.retract();
            }

            @Override
            public void retract()
            {
                solenoid.extend();
            }

            @Override
            protected ExtendState getRaw()
            {
                return solenoid.get() == ExtendState.Extended ? ExtendState.Retracted : ExtendState.Extended;
            }
        };
    }
}
