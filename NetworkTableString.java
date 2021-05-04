package frc.robot.abstraction;

public abstract class NetworkTableString implements Abstraction
{
    private String _value;

    protected abstract String getRaw();

    public abstract void set(String value);

    public String get()
    {
        return _value;
    }

    public void cache()
    {
        _value = getRaw();
    }
}
