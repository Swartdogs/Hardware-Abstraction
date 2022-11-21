package frc.robot.abstraction;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class NetworkTableDouble
{
    public abstract double get();
    public abstract void   set(double value);

    public abstract void   onChanged(Consumer<Double> action);
    
    public static NetworkTableDouble fromEntry(NetworkTableEntry entry)
    {
        return new NetworkTableDouble()
        {
            @Override
            public double get()
            {
                return entry.getDouble(0);
            }

            @Override
            public void set(double value)
            {
                entry.setDouble(value);
            }

            @Override
            public void onChanged(Consumer<Double> action)
            {
                entry.addListener(event -> action.accept(event.getEntry().getDouble(0)), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
            }
        };
    }

    public static NetworkTableDouble networkTableDouble(String tableName, String varName)
    {
        NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

        return fromEntry(entry);
    }
}
