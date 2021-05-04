package frc.robot.abstraction;

public abstract class NetworkTableDouble implements Abstraction
{
    private double _value;

    protected abstract double getRaw();

    public abstract void set(double value);
    
    public double get()
    {
        return _value;
    }

    public void cache()
    {
        _value = getRaw();
    }
}
