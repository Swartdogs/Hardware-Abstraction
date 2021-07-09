package frc.robot.abstraction;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
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
                    public void reset() {
                        encoder.setPosition(0);
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
                    public void reset()
                    {
                        motor.getSensorCollection().setIntegratedSensorPosition(0, 0);
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
            };
        }

        public static Motor falconFlywheel(int canId, double p, double i, double d, double f)
        {
            final double            scale   = 2048.0 / 600.0;

            TalonFX                 motor   = new TalonFX(canId);

            motor.configNominalOutputForward(0);
            motor.configNominalOutputReverse(0);
            motor.configPeakOutputForward(1);
            motor.configPeakOutputReverse(-1);

            motor.config_kP(0, p);
            motor.config_kI(0, i);
            motor.config_kD(0, d);
            motor.config_kF(0, f);

            return new Motor()
            {
                private double  _speed;

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
					return _speed;
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
                    _speed = speed * scale;

                    motor.set(ControlMode.Velocity, speed);
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
					// TODO Auto-generated method stub
					return motor.get();
				}

				@Override
				public PositionSensor getPositionSensor() {
					// TODO Auto-generated method stub
					return null;
				}

				@Override
				public VelocitySensor getVelocitySensor() {
					// TODO Auto-generated method stub
					return null;
				}

				@Override
				public void set(double speed) {
					// TODO Auto-generated method stub
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
                @Override
                protected double getRaw()
                {
                    return -gyro.getRotation2d().getDegrees();
                }

                @Override
                public void reset()
                {
                    gyro.reset();
                }
            };
        }

        public static PositionSensor imu()
        {
            ADIS16448_IMU imu = new ADIS16448_IMU();

            return new PositionSensor()
            {
                @Override
                protected double getRaw()
                {
                    return -imu.getGyroAngleX();
                }

                @Override
                public void reset()
                {
                    imu.reset();
                }
            };
        }

        public static PositionSensor potentiometer(int port, double scale, double offset)
        {
            AnalogPotentiometer potentiometer = new AnalogPotentiometer(port, scale, offset);

            return new PositionSensor()
            {
                @Override
                protected double getRaw()
                {
                    return -(potentiometer.get() - 360);
                }

                @Override
                public void reset()
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
                private double _zero = 0;

                @Override
                protected double getRaw()
                {
                    return analogInput.getAverageValue() - _zero;
                }

                @Override
                public void reset()
                {
                    _zero = analogInput.getAverageValue();
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
                @Override
                protected double getRaw() 
                {
                    return encoder.get();
                }

                @Override
                public void reset() 
                {
                    encoder.reset();
                }
            };
        }
    }

    public static final class NetworkTable
    {
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
