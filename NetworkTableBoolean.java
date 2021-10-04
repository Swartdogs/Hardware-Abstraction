package frc.robot.abstraction;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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

    public static NetworkTableBoolean networkTableBoolean(String tableName, String varName)
    {
        return new NetworkTableBoolean()
        {
            private NetworkTableEntry _entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

            @Override
            protected boolean getRaw()
            {
                return _entry.getBoolean(false);
            }

            @Override
            public void set(boolean value)
            {
                _entry.setBoolean(value);
            }
        };
    }
}
