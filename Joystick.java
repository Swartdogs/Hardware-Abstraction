package frc.robot.abstraction;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.abstraction.Enumerations.State;

public abstract class Joystick implements Abstraction
{
    private double  _x;
    private double  _y;
    private double  _z;
    private double  _throttle;

    private double  _xDeadband;
    private double  _yDeadband;
    private double  _zDeadband;
    private double  _throttleDeadband;

    private boolean _squareX;
    private boolean _squareY;
    private boolean _squareZ;
    private boolean _squareThrottle;

    protected abstract double getRawX();
    protected abstract double getRawY();
    protected abstract double getRawZ();
    protected abstract double getRawThrottle();

    public abstract Switch getButton(int buttonNum);
    public abstract int    getButtonCount();

    public double getX()
    {
        return _x;
    }

    public double getY()
    {
        return _y;
    }

    public double getZ()
    {
        return _z;
    }

    public double getThrottle()
    {
        return _throttle;
    }

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

    public void cache()
    {
        _x        = applyDeadband(getRawX(),        _xDeadband,        _squareX);
        _y        = applyDeadband(getRawY(),        _yDeadband,        _squareY);
        _z        = applyDeadband(getRawZ(),        _zDeadband,        _squareZ);
        _throttle = applyDeadband(getRawThrottle(), _throttleDeadband, _squareThrottle);
    }

    private double applyDeadband(double raw, double deadband, boolean squareInputs)
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
            protected double getRawX()
            {
                return _joystick.getX();
            }

            @Override
            protected double getRawY()
            {
                return -_joystick.getY();
            }

            @Override
            protected double getRawZ()
            {
                return _joystick.getZ();
            }

            @Override
            protected double getRawThrottle()
            {
                return _joystick.getRawAxis(3);
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
                        protected State getRaw()
                        {
                            return _joystick.getRawButton(buttonNum) ? State.On : State.Off;
                        }

                        @Override
                        public void whenActivated(SwartdogCommand command, boolean interruptible)
                        {
                            _buttons[buttonNum - 1].whenPressed(command, interruptible);
                        }

                        @Override
                        public void whileActive(SwartdogCommand command, boolean interruptible)
                        {
                            _buttons[buttonNum - 1].whileHeld(command, interruptible);
                        }

                        @Override
                        public void cancelWhenActivated(SwartdogCommand command)
                        {
                            _buttons[buttonNum - 1].cancelWhenPressed(command);
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

            @Override
            public void cache()
            {
                super.cache();

                for (Switch button : _switches)
                {
                    if (button != null)
                    {
                        button.cache();
                    }
                }
            }
        };
    }
}