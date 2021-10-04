package frc.robot.abstraction;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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

    public static NetworkTableDouble networkTableDouble(String tableName, String varName)
    {
        return new NetworkTableDouble()
        {
            private NetworkTableEntry _entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

            @Override
            protected double getRaw()
            {
                return _entry.getDouble(0);
            }

            @Override
            public void set(double value)
            {
                _entry.setDouble(value);
            }
        };
    }
}
