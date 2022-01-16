package frc.robot.abstraction;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class NetworkTableBoolean
{
    public abstract boolean get();
    public abstract void    set(boolean value);

    public static NetworkTableBoolean networkTableBoolean(String tableName, String varName)
    {
        return new NetworkTableBoolean()
        {
            private NetworkTableEntry _entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

            @Override
            public boolean get()
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
