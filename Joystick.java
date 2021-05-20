package frc.robot.abstraction;

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
}