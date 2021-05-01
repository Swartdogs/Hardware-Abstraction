package frc.robot.abstraction;

import frc.robot.abstraction.Enumerations.ExtendState;

public abstract class Solenoid
{
    public abstract void         extend();
    public abstract void         retract();
    public abstract ExtendState  get();
    
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

    public static Solenoid compose(Solenoid... solenoids)
    {
        return new Solenoid()
        {
            private ExtendState _state;

            public void extend()
            {
                _state = ExtendState.Extended;

                for (Solenoid s : solenoids)
                {
                    s.extend();
                }
            }

            public void retract()
            {
                _state = ExtendState.Retracted;

                for (Solenoid s : solenoids)
                {
                    s.retract();
                }
            }

            public ExtendState get()
            {
                return _state;
            }
        };
    }

    public static Solenoid invert(Solenoid solenoid)
    {
        return new Solenoid()
        {
            public void extend()
            {
                solenoid.retract();
            }

            public void retract()
            {
                solenoid.extend();
            }

            public ExtendState get()
            {
                return solenoid.get() == ExtendState.Extended ? ExtendState.Retracted : ExtendState.Extended;
            }
        };
    }
}
