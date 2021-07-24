package frc.robot.abstraction;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.function.Supplier;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.abstraction.Enumerations.ExtendState;
import frc.robot.abstraction.Enumerations.State;

@SuppressWarnings("resource")
public final class Hardware extends SwartdogSubsystem
{
    private ArrayList<Abstraction> _hardware;

    public Hardware(Abstraction... hardware)
    {
        _hardware = new ArrayList<Abstraction>();

        addHardware(hardware);
    }

    @Override
    public void periodic()
    {
        cacheAll();
    }

    public void addHardware(Abstraction... hardware)
    {
        for (Abstraction abstraction : hardware)
        {
            abstraction.cache();
            _hardware.add(abstraction);
        }
    }

    private void cacheAll()
    {
        for (Abstraction abstraction : _hardware)
        {
            abstraction.cache();
        }
    }

    public static class Controls
    {
        public static Joystick joystick(int joystickId)
        {
            return new Joystick()
            {
                private edu.wpi.first.wpilibj.Joystick _joystick = new edu.wpi.first.wpilibj.Joystick(joystickId);

                private Switch[] _switches = new Switch[12];
                private JoystickButton[] _buttons = new JoystickButton[12];

                @Override
                protected double getRawX()
                {
                    return _joystick.getX();
                }

                @Override
                protected double getRawY()
                {
                    return -_joystick.getY();
                }

                @Override
                protected double getRawZ()
                {
                    return _joystick.getZ();
                }

                @Override
                protected double getRawThrottle()
                {
                    return _joystick.getRawAxis(3);
                }

                @Override
                public Switch getButton(int buttonNum)
                {
                    if (buttonNum < 1 || buttonNum > _switches.length)
                    {
                        throw new IndexOutOfBoundsException(String.format("Button %d isn't available on this joystick!", buttonNum));
                    }

                    if (_switches[buttonNum - 1] == null)
                    {
                        _buttons[buttonNum - 1]  = new JoystickButton(_joystick, buttonNum);
                        _switches[buttonNum - 1] = new Switch()
                        {
                            @Override
                            protected State getRaw()
                            {
                                return _joystick.getRawButton(buttonNum) ? State.On : State.Off;
                            }

                            @Override
                            public void whenActivated(SwartdogCommand command, boolean interruptible)
                            {
                                _buttons[buttonNum - 1].whenPressed(command, interruptible);
                            }

                            @Override
                            public void whileActive(SwartdogCommand command, boolean interruptible)
                            {
                                _buttons[buttonNum - 1].whileHeld(command, interruptible);
                            }

                            @Override
                            public void cancelWhenActivated(SwartdogCommand command)
                            {
                                _buttons[buttonNum - 1].cancelWhenPressed(command);
                            }
                        };
                    }

                    return _switches[buttonNum - 1];
                }

                @Override
                public int getButtonCount()
                {
                    return _buttons.length;
                }

                @Override
                public void cache()
                {
                    super.cache();

                    for (Switch button : _switches)
                    {
                        if (button != null)
                        {
                            button.cache();
                        }
                    }                    
                }
            };
        }
    }

    public static class Actuators
    {
        public static Motor neo(int canId)
        {
            CANSparkMax motor   = new CANSparkMax(canId, MotorType.kBrushless);
            CANEncoder  encoder = motor.getEncoder();

            return new Motor()
            {
                private PositionSensor _positionSensor = new PositionSensor()
                {

                    @Override
                    protected double getRaw() {
                        return encoder.getPosition();
                    }

                    @Override
                    public void set(double newPosition) {
                        encoder.setPosition(newPosition);
                    }
                    
                };
                private VelocitySensor _velocitySensor = new VelocitySensor()
                {
                    @Override
                    protected double getRaw() {
                        return encoder.getVelocity();
                    }
                    
                };
                @Override
                protected double getRaw()
                {
                    return motor.get();
                }

                @Override
                public PositionSensor getPositionSensor()
                {
                    return _positionSensor;
                }

                @Override
                public VelocitySensor getVelocitySensor()
                {
                    return _velocitySensor;
                }

                @Override
                public void set(double speed)
                {
                    motor.set(speed);
                }
            };
        }

        public static Motor neoFlywheel(int canId, double maxMotorRPM)
        {
            CANSparkMax motor   = new CANSparkMax(canId, MotorType.kBrushless);
            CANEncoder  encoder = motor.getEncoder();

            return new Motor()
            {
                private double _setpoint = 0;

                private VelocitySensor _velocitySensor = new VelocitySensor()
                {
                    @Override 
                    protected double getRaw()
                    {
                        return encoder.getVelocity();
                    }
                };

                @Override
                protected double getRaw()
                {
                    return _setpoint;
                }

                @Override
                public PositionSensor getPositionSensor()
                {
                    return null;
                }

                @Override
                public VelocitySensor getVelocitySensor()
                {
                    return _velocitySensor;
                }

                @Override
                public void set(double speed)
                {
                    _setpoint = speed;
                    motor.set(_setpoint / maxMotorRPM);
                }
            };
        }

        public static Motor falcon(int canId)
        {
            TalonFX motor = new TalonFX(canId);

            return new Motor()
            {
                private double _speed;

                private PositionSensor _positionSensor = new PositionSensor()
                {
                    @Override
                    protected double getRaw()
                    {
                        return motor.getSensorCollection().getIntegratedSensorPosition();
                    }

                    @Override
                    public void set(double newPosition)
                    {
                        motor.getSensorCollection().setIntegratedSensorPosition(newPosition, 0);
                    }
                };

                private VelocitySensor _velocitySensor = new VelocitySensor()
                {
                    @Override
                    protected double getRaw()
                    {
                        return motor.getSensorCollection().getIntegratedSensorVelocity();
                    }
                };

                @Override
                protected double getRaw()
                {
                    return _speed;
                }

                @Override
                public PositionSensor getPositionSensor()
                {
                    return _positionSensor;
                }

                @Override
                public VelocitySensor getVelocitySensor()
                {
                    return _velocitySensor;
                }

                @Override
                public void set(double speed)
                {
                    _speed = speed;
                    motor.set(ControlMode.PercentOutput, speed);
                }

                @Override
                public void cache()
                {
                    super.cache();

                    _positionSensor.cache();
                    _velocitySensor.cache();
                }
            };
        }

        public static Motor falconFlywheel(int canId, double maxMotorRPM)
        {
            TalonFX                 motor   = new TalonFX(canId);

            return new Motor()
            {
                private double  _setpoint;

                private VelocitySensor _velocitySensor = new VelocitySensor()
                {
                    @Override
                    protected double getRaw()
                    {
                        return motor.getSensorCollection().getIntegratedSensorVelocity();
                    }
                };

				@Override
				protected double getRaw() {
					return _setpoint;
				}

				@Override
				public PositionSensor getPositionSensor() {
					return null;
				}

				@Override
				public VelocitySensor getVelocitySensor() {
					return _velocitySensor;
				}

				@Override
				public void set(double speed) {
                    _setpoint = speed;
                    motor.set(ControlMode.PercentOutput, _setpoint / maxMotorRPM);
                }
                
                @Override
                public void cache()
                {
                    super.cache();

                    _velocitySensor.cache();
                }
            };
        }

        public static Motor victorSP(int port)
        {
            VictorSP motor = new VictorSP(port);


            return new Motor(){

                @Override
                protected double getRaw() {
                    return motor.get();
                }

                @Override
                public PositionSensor getPositionSensor() {
                    return null;
                }

                @Override
                public VelocitySensor getVelocitySensor() {
                    return null;
                }

                @Override
                public void set(double speed) {
                    motor.set(speed);
                }
                
            };
        }

        public static Motor victorSPX(int canId)
        {
            WPI_VictorSPX motor = new WPI_VictorSPX(canId);

            return new Motor()
            {

				@Override
				protected double getRaw() {
					return motor.get();
				}

				@Override
				public PositionSensor getPositionSensor() {
					return null;
				}

				@Override
				public VelocitySensor getVelocitySensor() {
					return null;
				}

				@Override
				public void set(double speed) {
					motor.set(speed);
				}
                
            };
        }

        public static Solenoid solenoid(int port)
        {
            edu.wpi.first.wpilibj.Solenoid solenoid = new edu.wpi.first.wpilibj.Solenoid(port);

            return new Solenoid()
            {
                private ExtendState _state = ExtendState.Retracted;

                @Override
                protected ExtendState getRaw()
                {
                    return _state;
                }

                @Override
                public void extend()
                {
                    solenoid.set(true);
                    _state = ExtendState.Extended;
                    cache();
                }

                @Override
                public void retract()
                {
                    solenoid.set(false);
                    _state = ExtendState.Retracted;
                    cache();
                }
            };
        }
    }

    public static class Sensors
    {
        public static PositionSensor gyro()
        {
            ADXRS450_Gyro gyro = new ADXRS450_Gyro();

            return new PositionSensor()
            {
                private double _offset = 0;
                
                @Override
                protected double getRaw()
                {
                    return -gyro.getRotation2d().getDegrees() + _offset;
                }

                @Override
                public void set(double newPosition)
                {
                    gyro.reset();
                    _offset = newPosition;
                }
            };
        }

        public static PositionSensor imu()
        {
            ADIS16448_IMU imu = new ADIS16448_IMU();

            return new PositionSensor()
            {
                private double _offset = 0;

                @Override
                protected double getRaw()
                {
                    return -imu.getGyroAngleX() + _offset;
                }

                @Override
                public void set(double newPosition)
                {
                    imu.reset();
                    _offset = newPosition;
                }
            };
        }

        public static PositionSensor potentiometer(int port, double scale, double offset)
        {
            AnalogPotentiometer potentiometer = new AnalogPotentiometer(port, scale, offset);

            return new PositionSensor()
            {
                private double _offset = 0;

                @Override
                protected double getRaw()
                {
                    return -(potentiometer.get() - 360) + _offset;
                }

                @Override
                public void set(double newPosition)
                {
                    return;
                }
            };
        }

        public static PositionSensor analogInput(int port)
        {
            AnalogInput analogInput = new AnalogInput(port);

            analogInput.setAverageBits(2);
            analogInput.setOversampleBits(0);

            return new PositionSensor()
            {
                private double _offset = 0;

                @Override
                protected double getRaw()
                {
                    return analogInput.getAverageValue() + _offset;
                }

                @Override
                public void set(double newPosition)
                {
                    _offset = newPosition - analogInput.getAverageValue();
                }
            };
        }

        public static Switch genericSwitch(Supplier<State> stateSupplier)
        {
            return new HardwareSwitch()
            {
                @Override
                protected State getRaw()
                {
                    return stateSupplier.get();
                }
            };
        }

        public static Switch lightSensor(int port)
        {
            DigitalInput lightSensor = new DigitalInput(port);

            return new HardwareSwitch()
            {
                @Override
                protected State getRaw()
                {
                    return lightSensor.get() ? State.Off : State.On;
                }
            };
        }

        public static PositionSensor dutyCycleEncoder(int port)
        {
            DutyCycleEncoder encoder = new DutyCycleEncoder(port);

            return new PositionSensor()
            {
                private double _offset = 0;

                @Override
                protected double getRaw() 
                {
                    return encoder.get() + _offset;
                }

                @Override
                public void set(double newPosition) 
                {
                    encoder.reset();
                    _offset = newPosition;
                }
            };
        }
    }

    public static final class NetworkTable
    {
        public static ShuffleboardTab shuffleboardTab(String tabName)
        {
            edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab tab = Shuffleboard.getTab(tabName);

            return new ShuffleboardTab()
            {
                private SendableChooser<SwartdogCommand> _autoChooser = null;

                @Override
                protected NetworkTableBoolean addBoolean(String networkTableId, boolean defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
                {
                    NetworkTableEntry entry = createWidget(tab, networkTableId, defaultValue, x, y, w, h, widget, properties);

                    return new NetworkTableBoolean()
                    {
                        @Override
                        public boolean getRaw()
                        {
                            return entry.getBoolean(false);
                        }

                        @Override
                        public void set(boolean value)
                        {
                            entry.setBoolean(value);
                        }
                    };
                }

                @Override
                protected NetworkTableDouble addDouble(String networkTableId, double defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
                {
                    NetworkTableEntry entry = createWidget(tab, networkTableId, defaultValue, x, y, w, h, widget, properties);

                    return new NetworkTableDouble()
                    {
                        @Override
                        public double getRaw()
                        {
                            return entry.getDouble(0);
                        }

                        @Override
                        public void set(double value)
                        {
                            entry.setDouble(value);
                        }
                    };
                }

                @Override
                protected NetworkTableString addString(String networkTableId, String defaultValue, int x, int y, int w, int h, BuiltInWidgets widget, Map<String, Object> properties)
                {
                    NetworkTableEntry entry = createWidget(tab, networkTableId, defaultValue, x, y, w, h, widget, properties);

                    return new NetworkTableString()
                    {
                        @Override
                        public String getRaw()
                        {
                            return entry.getString("");
                        }

                        @Override
                        public void set(String value)
                        {
                            entry.setString(value);
                        }
                    };
                }
            
                @Override
                protected ShuffleboardLayout addLayout(String layoutName, BuiltInLayouts layout, int x, int y, int w, int h, Map<String, Object> properties)
                {
                    edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout sbl = createLayout(tab, layoutName, layout, x, y, w, h, properties);

                    return createLayout(sbl);
                }

                @Override
                public void addAutonomousChooser(int x, int y, int w, int h, BuiltInWidgets widget)
                {
                    if (_autoChooser == null)
                    {
                        _autoChooser = new SendableChooser<SwartdogCommand>();

                        tab.add("Autonomous Selector", _autoChooser).withPosition(x, y).withSize(w, h).withWidget(widget);
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

                            return new NetworkTableBoolean()
                            {
                                @Override
                                protected boolean getRaw()
                                {
                                    return entry.getBoolean(false);
                                }

                                @Override
                                public void set(boolean value)
                                {
                                    entry.setBoolean(value);
                                }
                            };
                        }

                        @Override
                        protected NetworkTableDouble addDouble(String networkTableId, double defaultValue, BuiltInWidgets widget, Map<String, Object> properties)
                        {
                            NetworkTableEntry entry = createWidget(sbl, networkTableId, defaultValue, widget, properties);

                            return new NetworkTableDouble()
                            {
                                @Override
                                protected double getRaw()
                                {
                                    return entry.getDouble(0);
                                }

                                @Override
                                public void set(double value)
                                {
                                    entry.setDouble(value);
                                }
                            };
                        }

                        @Override
                        protected NetworkTableString addString(String networkTableId, String defaultValue, BuiltInWidgets widget, Map<String, Object> properties)
                        {
                            NetworkTableEntry entry = createWidget(sbl, networkTableId, defaultValue, widget, properties);

                            return new NetworkTableString()
                            {
                                @Override
                                protected String getRaw()
                                {
                                    return entry.getString("");
                                }

                                @Override
                                public void set(String value)
                                {
                                    entry.setString(value);
                                }
                            };
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
            };
        }

        public static NetworkTableDouble networkTableDouble(String tableName, String varName)
        {
            NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

            return new NetworkTableDouble()
            {
				@Override
                protected double getRaw()
                {
					return entry.getDouble(0);
				}

				@Override
                public void set(double value)
                {
					entry.setDouble(value);
				}
            };
        }

        public static NetworkTableBoolean networkTableBoolean(String tableName, String varName)
        {
            NetworkTableEntry entry = NetworkTableInstance.getDefault().getTable(tableName).getEntry(varName);

            return new NetworkTableBoolean()
            {
                @Override
                protected boolean getRaw()
                {
                    return entry.getBoolean(false);
                }

                @Override
                public void set(boolean value)
                {
                    entry.setBoolean(value);
                }
            };
        }
    }

    public static abstract class HardwareSwitch extends Switch
    {
        private HashSet<SwartdogCommand> _whenActivatedCommands = new HashSet<SwartdogCommand>();
        private HashSet<SwartdogCommand> _whileActiveCommands   = new HashSet<SwartdogCommand>();
        private HashSet<SwartdogCommand> _cancelCommands        = new HashSet<SwartdogCommand>();

        private HashMap<SwartdogCommand, Boolean> _interruptibleMap = new HashMap<SwartdogCommand, Boolean>();

        @Override
        public void whenActivated(SwartdogCommand command, boolean interruptible)
        {
            _whenActivatedCommands.add(command);
            _interruptibleMap.put(command, interruptible);
        }

        @Override
        public void whileActive(SwartdogCommand command, boolean interruptible)
        {
            _whileActiveCommands.add(command);
            _interruptibleMap.put(command, interruptible);
        }

        @Override
        public void cancelWhenActivated(SwartdogCommand command)
        {
            _cancelCommands.add(command);
        }

        @Override
        public void cache()
        {
            super.cache();

            CommandScheduler scheduler = CommandScheduler.getInstance();

            if (transitionedTo(State.On))
            {
                for (SwartdogCommand command : _whenActivatedCommands)
                {
                    scheduler.schedule(_interruptibleMap.get(command), command);
                }

                for (SwartdogCommand command : _whileActiveCommands)
                {
                    scheduler.schedule(_interruptibleMap.get(command), command);
                }
            }

            else if (transitionedTo(State.Off))
            {
                for (SwartdogCommand command : _whileActiveCommands)
                {
                    if (scheduler.isScheduled(command))
                    {
                        scheduler.cancel(command);
                    }
                }

                for (SwartdogCommand command : _cancelCommands)
                {
                    if (scheduler.isScheduled(command))
                    {
                        scheduler.cancel(command);
                    }
                }
            }
        }
    }
}
