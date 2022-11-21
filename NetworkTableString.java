package frc.robot.abstraction;

import java.util.function.Consumer;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public abstract class NetworkTableString
{
    public abstract String get();
    public abstract void   set(String value);

    public abstract void   onChanged(Consumer<String> action);

    public static NetworkTableString fromEntry(NetworkTableEntry entry)
    {
        return new NetworkTableString()
        {
            @Override
            public String get()
            {
                return entry.getString("");
            }

            @Override
            public void set(String value)
            {
                entry.setString(value);
            }

            @Override
            public void onChanged(Consumer<String> action)
            {
                entry.addListener(event -> action.accept(event.getEntry().getString("")), EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
            }
        };
    }

    public static NetworkTableString networkTableString(String tableName, String varName)
    {
        NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

        return fromEntry(entry);
    }
}
