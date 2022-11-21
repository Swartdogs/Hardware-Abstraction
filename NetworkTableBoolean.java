package frc.robot.abstraction;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class NetworkTableBoolean
{
    public abstract boolean get();
    public abstract void    set(boolean value);

    public abstract void    onChanged(Consumer<Boolean> action);

    public static NetworkTableBoolean fromEntry(NetworkTableEntry entry)
    {
        return new NetworkTableBoolean()
        {
            @Override
            public boolean get()
            {
                return entry.getBoolean(false);
            }

            @Override
            public void set(boolean value)
            {
                entry.setBoolean(value);
            }

            @Override
            public void onChanged(Consumer<Boolean> action)
            {
                entry.addListener(event -> action.accept(event.getEntry().getBoolean(false)), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
            }
        };
    }

    public static NetworkTableBoolean networkTableBoolean(String tableName, String varName)
    {
        NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

        return fromEntry(entry);
    }
}
