package frc.robot.abstraction;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.abstraction.Enumerations.State;

public abstract class Joystick
{
    protected JoystickAxis _x        = new JoystickAxis();
    protected JoystickAxis _y        = new JoystickAxis();
    protected JoystickAxis _z        = new JoystickAxis();
    protected JoystickAxis _throttle = new JoystickAxis();

    public abstract double getX();
    public abstract double getY();
    public abstract double getZ();
    public abstract double getThrottle();

    public abstract Switch getButton(int buttonNum);
    public abstract int    getButtonCount();

    public JoystickAxis getXAxis()
    {
        return _x;
    }

    public JoystickAxis getYAxis()
    {
        return _y;
    }

    public JoystickAxis getZAxis()
    {
        return _z;
    }

    public JoystickAxis getThrottleAxis()
    {
        return _throttle;
    }
    public static class JoystickAxis
    {
        private double _deadband;
        private double _motionThreshold;
        private double _exponent = 1;

        public void setDeadband(double deadband)
        {
            _deadband = deadband;
        }

        public void setMotionThreshold(double motionThreshold)
        {
            _motionThreshold = motionThreshold;
        }

        public void setExponent(double exponent)
        {
            _exponent = exponent;
        }

        public double applyDeadband(double raw)
        {
            double inputAbs = Math.abs(raw);

            double withDeadband = (inputAbs < _deadband) ? 0 : (inputAbs - _deadband) / (1 - _deadband);

            double withExponent = Math.pow(_exponent, withDeadband - 1) * withDeadband;

            double withMotionThreshold = (withExponent * (1 - _motionThreshold)) + (Math.signum(withExponent) * _motionThreshold);

            return withMotionThreshold * Math.signum(raw);
        }
    }

    public static Joystick joystick(int joystickId)
    {
        return new Joystick()
        {
            private edu.wpi.first.wpilibj.Joystick _joystick = new edu.wpi.first.wpilibj.Joystick(joystickId);

            private Switch[] _switches = new Switch[12];
            private JoystickButton[] _buttons = new JoystickButton[12];

            @Override
            public double getX()
            {
                return _x.applyDeadband(_joystick.getX());
            }

            @Override
            public double getY()
            {
                return _y.applyDeadband(-_joystick.getY());
            }

            @Override
            public double getZ()
            {
                return _z.applyDeadband(_joystick.getZ());
            }

            @Override
            public double getThrottle()
            {
                return _throttle.applyDeadband(_joystick.getRawAxis(3));
            }

            @Override
            public Switch getButton(int buttonNum)
            {
                if (buttonNum < 1 || buttonNum > _switches.length)
                {
                    throw new IndexOutOfBoundsException(String.format("Button %d isn't available on this joystick!", buttonNum));
                }

                if (_switches[buttonNum - 1] == null)
                {
                    _buttons[buttonNum - 1]  = new JoystickButton(_joystick, buttonNum);
                    _switches[buttonNum - 1] = new Switch()
                    {
                        @Override
                        public State get()
                        {
                            return _joystick.getRawButton(buttonNum) ? State.On : State.Off;
                        }

                        @Override
                        public void whenDeactivated(SwartdogCommand command, boolean interruptible)
                        {
                            _buttons[buttonNum - 1].whenReleased(command, interruptible);
                        }

                        @Override
                        public void whileActive(SwartdogCommand command, boolean interruptible)
                        {
                            _buttons[buttonNum - 1].whenHeld(command, interruptible);       
                        }

                        @Override
                        public void whenActivated(SwartdogCommand command, boolean interruptible)
                        {
                            _buttons[buttonNum - 1].whenPressed(command, interruptible);          
                        }

                        @Override
                        public void cancelWhenActivated(SwartdogCommand command, boolean interruptible)
                        {
                            _buttons[buttonNum - 1].cancelWhenActive(command);
                        }
                    };
                }

                return _switches[buttonNum - 1];
            }

            @Override
            public int getButtonCount()
            {
                return _buttons.length;
            }
        };
    }
}