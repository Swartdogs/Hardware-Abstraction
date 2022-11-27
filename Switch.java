package frc.robot.abstraction;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.abstraction.Enumerations.State;

public abstract class Switch
{
    public abstract State get();

    public void whenDeactivated(CommandBase command)
    {
        whenDeactivated(command, true);
    }

    public void whenDeactivated(CommandBase command, boolean interruptible)
    {

    }

    public void whileActive(CommandBase command)
    {
        whileActive(command, true);
    }

    public void whileActive(CommandBase command, boolean interruptible)
    {

    }

    public void whenActivated(CommandBase command)
    {
        whenActivated(command, true);
    }

    public void whenActivated(CommandBase command, boolean interruptible)
    {

    }

    public void cancelWhenActivated(CommandBase command)
    {
        cancelWhenActivated(command, true);
    }

    public void cancelWhenActivated(CommandBase command, boolean interruptible)
    {

    }

    public static Switch genericSwitch(Supplier<State> stateSupplier)
    {
        return new Switch()
        {
            @Override
            public State get()
            {
                return stateSupplier.get();
            }
        };
    }

    public static Switch lightSensor(int port)
    {
        return new Switch()
        {
            private DigitalInput _lightSensor = new DigitalInput(port);

            @Override
            public State get()
            {
                return _lightSensor.get() ? State.Off : State.On;
            }
        };
    }

    public static Switch limitSwitch(int port)
    {
        return new Switch() 
        {
            private DigitalInput _limitSwitch = new DigitalInput(port);

            @Override
            public State get()
            {
                return _limitSwitch.get() ? State.On : State.Off;
            }
        };
    }
    
    public static abstract class SettableSwitch extends Switch
    {
        public abstract void set(State state);
        
        public static SettableSwitch compressor(int moduleId, PneumaticsModuleType moduleType)
        {
            return new SettableSwitch() 
            {
                private Compressor _compressor = new Compressor(moduleId, moduleType);
                
                @Override
                public State get()
                {
                    return _compressor.enabled() ? State.On : State.Off;
                }
                
                @Override
                public void set(State state)
                {
                    switch (state)
                    {
                        case On:
                        _compressor.enableDigital();
                        break;
                        
                        case Off:
                        _compressor.disable();
                        break;
                        
                        default:
                        break;
                    }
                }
            };
        }

        public static SettableSwitch relay(int port)
        {
            return new SettableSwitch() 
            {
                private Relay _relay = new Relay(port, Relay.Direction.kBoth);
                
                @Override
                public State get()
                {
                    State state;

                    switch (_relay.get())
                    {
                        case kOn:
                        case kForward:
                            state = State.On;
                            break;

                        case kReverse:
                            state = State.Reverse;
                            break;

                        default:
                            state = State.Off;
                            break;
                    }

                    return state;
                }
                
                @Override
                public void set(State state)
                {
                    switch (state)
                    {
                        case On:
                            _relay.set(Value.kForward);
                            break;

                        case Reverse:
                            _relay.set(Value.kReverse);
                            break;
                        
                        case Off:
                            _relay.set(Value.kOff);
                            break;
                        
                        default:
                            break;
                    }
                }
            };
        }
    }
    
    public static class MockSwitch extends SettableSwitch 
    {
        private State _state;
    
        public MockSwitch() 
        {
            this(State.Off);
        }
    
        public MockSwitch(State initialState) 
        {
            set(initialState);
        }
    
        @Override
        public State get() 
        {
            return _state;
        }
    
        @Override
        public void set(State state) 
        {
            _state = state;
        }
    }
}
