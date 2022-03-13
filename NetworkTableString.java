package frc.robot.abstraction;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class NetworkTableString
{
    public abstract String get();
    public abstract void   set(String value);

    public static NetworkTableString networkTableString(String tableName, String varName)
    {
        return new NetworkTableString()
        {
            private NetworkTableEntry _entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

            @Override
            public String get()
            {
                return _entry.getString("");
            }

            @Override
            public void set(String value)
            {
                _entry.setString(value);
            }
        };
    }
}
