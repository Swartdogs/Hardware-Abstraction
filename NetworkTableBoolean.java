package frc.robot.abstraction;

public abstract class NetworkTableBoolean implements Abstraction
{
    private boolean _value;

    protected abstract boolean getRaw();

    public abstract void set(boolean value);

    public boolean get()
    {
        return _value;
    }

    public void cache()
    {
        _value = getRaw();
    }
}
