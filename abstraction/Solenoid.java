package frc.robot.abstraction;

import frc.robot.abstraction.Enumerations.ExtendState;

public abstract class Solenoid
{
    public abstract void         extend();
    public abstract void         retract();
    public abstract ExtendState  getExtendState();

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

            public ExtendState getExtendState()
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

            public ExtendState getExtendState()
            {
                return solenoid.getExtendState() == ExtendState.Extended ? ExtendState.Retracted : ExtendState.Extended;
            }
        };
    }
}
