package frc.robot.abstraction;

import java.util.HashSet;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public abstract class ShuffleboardTab implements Abstraction
{
    private HashSet<NetworkTableBoolean> _ntbs;
    private HashSet<NetworkTableDouble>  _ntds;
    private HashSet<NetworkTableString>  _ntss;
    private HashSet<ShuffleboardLayout>  _sbls;

    public ShuffleboardTab()
    {
        _ntbs = new HashSet<NetworkTableBoolean>();
        _ntds = new HashSet<NetworkTableDouble>();
        _ntss = new HashSet<NetworkTableString>();
        _sbls = new HashSet<ShuffleboardLayout>();
    }

    protected abstract NetworkTableBoolean addBoolean(String networkTableId, boolean defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties);
    protected abstract NetworkTableDouble  addDouble(String networkTableId, double defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties);
    protected abstract NetworkTableString  addString(String networkTableId, String defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties);
    protected abstract ShuffleboardLayout  addLayout(String layoutName, BuiltInLayouts layout, int x, int y, int w, int h, Map<String, Object> properties);

    public abstract void            addAutonomousChooser(int x, int y, int w, int h, BuiltInWidgets widget);
    public abstract void            addAutonomous(String name, SwartdogCommand autonomous);
    public abstract void            addDefaultAutonomous(String name, SwartdogCommand autonomous);
    public abstract SwartdogCommand getSelectedAutonomous();

    public NetworkTableBoolean addBooleanWidget(String networkTableId, boolean defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
    {
        NetworkTableBoolean ntb = addBoolean(networkTableId, defaultValue, x, y, w, h, widget, properties);

        _ntbs.add(ntb);

        return ntb;
    }

    public NetworkTableDouble addDoubleWidget(String networkTableId, double defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
    {
        NetworkTableDouble ntd = addDouble(networkTableId, defaultValue, x, y, w, h, widget, properties);

        _ntds.add(ntd);

        return ntd;
    }

    public NetworkTableString addStringWidget(String networkTableId, String defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
    {
        NetworkTableString nts = addString(networkTableId, defaultValue, x, y, w, h, widget, properties);

        _ntss.add(nts);

        return nts;
    }

    public ShuffleboardLayout addShuffleboardLayout(String layoutName, BuiltInLayouts layout, int x, int y, int w, int h, Map<String, Object> properties)
    {
        ShuffleboardLayout sbl = addLayout(layoutName, layout, x, y, w, h, properties);

        _sbls.add(sbl);

        return sbl;
    }

    public void cache()
    {
        for (NetworkTableBoolean ntb : _ntbs)
        {
            ntb.cache();
        }

        for (NetworkTableDouble ntd : _ntds)
        {
            ntd.cache();
        }

        for (NetworkTableString nts : _ntss)
        {
            nts.cache();
        }

        for (ShuffleboardLayout sbl : _sbls)
        {
            sbl.cache();
        }
    }
}
