package frc.robot.abstraction;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.abstraction.Enumerations.ExtendState;

public abstract class Solenoid
{
    public abstract ExtendState get();
    public abstract void extend();
    public abstract void retract();
    
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
            @Override
            public void extend()
            {
                for (Solenoid s : solenoids)
                {
                    s.extend();
                }
            }

            @Override
            public void retract()
            {
                for (Solenoid s : solenoids)
                {
                    s.retract();
                }
            }

            @Override
            public ExtendState get()
            {
                return solenoids[0].get();
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
            public ExtendState get()
            {
                return solenoid.get() == ExtendState.Extended ? ExtendState.Retracted : ExtendState.Extended;
            }
        };
    }

    public static Solenoid solenoid(int canId, PneumaticsModuleType type, int port)
    {
        return new Solenoid()
        {
            private edu.wpi.first.wpilibj.Solenoid _solenoid = new edu.wpi.first.wpilibj.Solenoid(canId, type, port);

            @Override
            public ExtendState get()
            {
                return _solenoid.get() ? ExtendState.Extended : ExtendState.Retracted;
            }

            @Override
            public void extend()
            {
                _solenoid.set(true);
            }

            @Override
            public void retract()
            {
                _solenoid.set(false);
            }
        };
    }

    public static class MockSolenoid extends Solenoid 
    {
        private ExtendState _state;
    
        public MockSolenoid()
        {
            this(ExtendState.Retracted);
        }
    
        public MockSolenoid(ExtendState initialState)
        {
            set(initialState);
        }
    
        @Override
        public void set(ExtendState state)
        {
            _state = state;
        }
    
        @Override
        public void extend() 
        {
            _state = ExtendState.Extended;
        }
    
        @Override
        public void retract() 
        {
            _state = ExtendState.Retracted;
        }
    
        @Override
        public ExtendState get() 
        {
            return _state;
        }
    }    
}
