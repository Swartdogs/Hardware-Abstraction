package frc.robot.abstraction;

public abstract class Joystick implements Abstraction
{
    private double _x;
    private double _y;
    private double _z;
    private double _throttle;

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

    public void cache()
    {
        _x        = getRawX();
        _y        = getRawY();
        _z        = getRawZ();
        _throttle = getRawThrottle();
    }
}
