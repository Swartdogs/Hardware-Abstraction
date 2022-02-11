package frc.robot.abstraction;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.abstraction.Enumerations.State;

public abstract class Joystick
{
    protected double  _xDeadband;
    protected double  _yDeadband;
    protected double  _zDeadband;
    protected double  _throttleDeadband;

    protected boolean _squareX;
    protected boolean _squareY;
    protected boolean _squareZ;
    protected boolean _squareThrottle;

    public abstract double getX();
    public abstract double getY();
    public abstract double getZ();
    public abstract double getThrottle();

    public abstract Switch getButton(int buttonNum);
    public abstract int    getButtonCount();

    public void setXDeadband(double xDeadband)
    {
        _xDeadband = xDeadband;
    }

    public void setYDeadband(double yDeadband)
    {
        _yDeadband = yDeadband;
    }

    public void setZDeadband(double zDeadband)
    {
        _zDeadband = zDeadband;
    }

    public void setThrottleDeadband(double throttleDeadband)
    {
        _throttleDeadband = throttleDeadband;
    }

    public void setSquareX(boolean squareX)
    {
        _squareX = squareX;
    }

    public void setSquareY(boolean squareY)
    {
        _squareY = squareY;
    }

    public void setSquareZ(boolean squareZ)
    {
        _squareZ = squareZ;
    }

    public void setSquareThottle(boolean squareThrottle)
    {
        _squareThrottle = squareThrottle;
    }

    protected double applyDeadband(double raw, double deadband, boolean squareInputs)
    {
        double modified = 0.0;

        if (raw < -deadband)
        {
            modified = ((raw + 1) / (1 - deadband)) - 1;
        }

        else if (raw > deadband)
        {
            modified = ((raw - 1) / (1 - deadband)) + 1;
        }

        else
        {
            modified = 0.0;
        }

        if (squareInputs)
        {
            modified *= Math.abs(modified);
        }

        return modified;
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
                return applyDeadband(_joystick.getX(), _xDeadband, _squareX);
            }

            @Override
            public double getY()
            {
                return applyDeadband(-_joystick.getY(), _yDeadband, _squareY);
            }

            @Override
            public double getZ()
            {
                return applyDeadband(_joystick.getZ(), _zDeadband, _squareZ);
            }

            @Override
            public double getThrottle()
            {
                return applyDeadband(_joystick.getRawAxis(3), _throttleDeadband, _squareThrottle);
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