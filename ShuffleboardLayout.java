package frc.robot.abstraction;

import java.util.HashSet;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public abstract class ShuffleboardLayout implements Abstraction
{
    private HashSet<NetworkTableBoolean> _ntbs;
    private HashSet<NetworkTableDouble>  _ntds;
    private HashSet<NetworkTableString>  _ntss;
    private HashSet<ShuffleboardLayout>  _sbls;

    public ShuffleboardLayout()
    {
        _ntbs = new HashSet<NetworkTableBoolean>();
        _ntds = new HashSet<NetworkTableDouble>();
        _ntss = new HashSet<NetworkTableString>();
        _sbls = new HashSet<ShuffleboardLayout>();
    }

    protected abstract NetworkTableBoolean addBoolean(String networkTableId, boolean defaultValue, BuiltInWidgets widget, Map<String, Object> properties);
    protected abstract NetworkTableDouble  addDouble(String networkTableId, double defaultValue, BuiltInWidgets widget, Map<String, Object> properties);
    protected abstract NetworkTableString  addString(String networkTableId, String defaultValue, BuiltInWidgets widget, Map<String, Object> properties);
    protected abstract ShuffleboardLayout  addLayout(String layoutName, BuiltInLayouts layout, int x, int y, Map<String, Object> properties);

    public NetworkTableBoolean addBooleanWidget(String networkTableId, boolean defaultValue, BuiltInWidgets widget, Map<String, Object> properties)
    {
        NetworkTableBoolean ntb = addBoolean(networkTableId, defaultValue, widget, properties);

        _ntbs.add(ntb);

        return ntb;
    }

    public NetworkTableDouble addDoubleWidget(String networkTableId, double defaultValue, BuiltInWidgets widget, Map<String, Object> properties)
    {
        NetworkTableDouble ntd = addDouble(networkTableId, defaultValue, widget, properties);

        _ntds.add(ntd);

        return ntd;
    }

    public NetworkTableString addStringWidget(String networkTableId, String defaultValue, BuiltInWidgets widget, Map<String, Object> properties)
    {
        NetworkTableString nts = addString(networkTableId, defaultValue, widget, properties);

        _ntss.add(nts);

        return nts;
    }

    public ShuffleboardLayout addShuffleboardLayout(String layoutName, BuiltInLayouts layout, int x, int y, Map<String, Object> properties)
    {
        ShuffleboardLayout sbl = addLayout(layoutName, layout, x, y, properties);

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
