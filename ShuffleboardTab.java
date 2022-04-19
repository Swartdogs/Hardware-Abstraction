package frc.robot.abstraction;

import java.util.HashSet;
import java.util.Map;

import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public abstract class ShuffleboardTab
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
    public abstract void            addCamera(int x, int y, int w, int h, VideoSource videoSource, Map<String, Object> properties);
    public abstract void            setVideoSource(VideoSource videoSource);

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

    public static ShuffleboardTab shuffleboardTab(String tabName)
    {
        return new ShuffleboardTab()
        {
            private edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab _tab         = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab(tabName);
            private SendableChooser<SwartdogCommand>                   _autoChooser = null;
            private ComplexWidget                                      _camera      = null;

            @Override
            protected NetworkTableBoolean addBoolean(String networkTableId, boolean defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
            {
                NetworkTableEntry entry = createWidget(_tab, networkTableId, defaultValue, x, y, w, h, widget, properties);

                return NetworkTableBoolean.fromEntry(entry);
            }

            @Override
            protected NetworkTableDouble addDouble(String networkTableId, double defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
            {
                NetworkTableEntry entry = createWidget(_tab, networkTableId, defaultValue, x, y, w, h, widget, properties);

                return NetworkTableDouble.fromEntry(entry);
            }

            @Override
            protected NetworkTableString addString(String networkTableId, String defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
            {
                NetworkTableEntry entry = createWidget(_tab, networkTableId, defaultValue, x, y, w, h, widget, properties);

                return NetworkTableString.fromEntry(entry);
            }
        
            @Override
            protected ShuffleboardLayout addLayout(String layoutName, BuiltInLayouts layout, int x, int y, int w, int h, Map<String, Object> properties)
            {
                edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout sbl = createLayout(_tab, layoutName, layout, x, y, w, h, properties);

                return createLayout(sbl);
            }

            @Override
            public void addAutonomousChooser(int x, int y, int w, int h, BuiltInWidgets widget)
            {
                if (_autoChooser == null)
                {
                    _autoChooser = new SendableChooser<SwartdogCommand>();

                    _tab.add("Autonomous Selector", _autoChooser).withPosition(x, y).withSize(w, h).withWidget(widget);
                }
            }

            @Override
            public void addAutonomous(String name, SwartdogCommand autonomous)
            {
                if (_autoChooser != null)
                {
                    _autoChooser.addOption(name, autonomous);
                }
            }

            @Override
            public void addDefaultAutonomous(String name, SwartdogCommand autonomous)
            {
                if (_autoChooser != null)
                {
                    _autoChooser.setDefaultOption(name, autonomous);
                }
            }

            @Override
            public SwartdogCommand getSelectedAutonomous()
            {
                SwartdogCommand auto = null;

                if (_autoChooser != null)
                {
                    auto = _autoChooser.getSelected();
                }

                return auto;
            }

            private ShuffleboardLayout createLayout(edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout sbl)
            {
                return new ShuffleboardLayout()
                {
                    @Override
                    protected NetworkTableBoolean addBoolean(String networkTableId, boolean defaultValue, BuiltInWidgets widget, Map<String, Object> properties)
                    {
                        NetworkTableEntry entry = createWidget(sbl, networkTableId, defaultValue, widget, properties);

                        return NetworkTableBoolean.fromEntry(entry);
                    }

                    @Override
                    protected NetworkTableDouble addDouble(String networkTableId, double defaultValue, BuiltInWidgets widget, Map<String, Object> properties)
                    {
                        NetworkTableEntry entry = createWidget(sbl, networkTableId, defaultValue, widget, properties);

                        return NetworkTableDouble.fromEntry(entry);
                    }

                    @Override
                    protected NetworkTableString addString(String networkTableId, String defaultValue, BuiltInWidgets widget, Map<String, Object> properties)
                    {
                        NetworkTableEntry entry = createWidget(sbl, networkTableId, defaultValue, widget, properties);

                        return NetworkTableString.fromEntry(entry);
                    }

                    @Override
                    protected ShuffleboardLayout addLayout(String layoutName, BuiltInLayouts layout, int x, int y, Map<String, Object> properties)
                    {
                        return createLayout(sbl);
                    }
                };
            }

            private edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout createLayout(edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab tab, String layoutName, BuiltInLayouts layout, int x, int y, int w, int h, Map<String, Object> properties)
            {
                edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout sbl = tab.getLayout(layoutName, layout).withPosition(x, y).withSize(w, h);

                if (properties != null && !properties.isEmpty())
                {
                    sbl = sbl.withProperties(properties);
                }

                return sbl;
            }

            private NetworkTableEntry createWidget(edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout layout, String networkTableId, Object defaultValue, BuiltInWidgets widget, Map<String, Object> properties)
            {
                SimpleWidget sbw = layout.add(networkTableId, defaultValue).withWidget(widget);

                if (properties != null && !properties.isEmpty())
                {
                    sbw = sbw.withProperties(properties);
                }

                return sbw.getEntry();
            }

            private NetworkTableEntry createWidget(edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab tab, String networkTableId, Object defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
            {
                SimpleWidget sbw = tab.add(networkTableId, defaultValue).withPosition(x, y).withSize(w, h).withWidget(widget);

                if (properties != null && !properties.isEmpty())
                {
                    sbw = sbw.withProperties(properties);
                }

                return sbw.getEntry();
            }

            public void addCamera(int x, int y, int w, int h, VideoSource videoSource, Map<String, Object> properties)
            {
                _camera = _tab.add(videoSource).withPosition(x, y).withSize(w, h);

                if (properties != null && !properties.isEmpty())
                {
                    _camera = _camera.withProperties(properties);
                }
            }

            public void setVideoSource(VideoSource videoSource)
            {
                
            }
        };
    }
}
