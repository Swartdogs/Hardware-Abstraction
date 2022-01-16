package frc.robot.abstraction;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class NetworkTableDouble
{
    public abstract double get();
    public abstract void   set(double value);
    
    public static NetworkTableDouble networkTableDouble(String tableName, String varName)
    {
        return new NetworkTableDouble()
        {
            private NetworkTableEntry _entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

            @Override
            public double get()
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
