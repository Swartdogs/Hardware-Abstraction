package frc.robot.abstraction;

import java.util.ArrayList;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
                            public void whenActivated(Command command)
                            {
                                _buttons[buttonNum - 1].whenPressed(command);
                            }

                            @Override
                            public void whileActive(Command command)
                            {
                                _buttons[buttonNum - 1].whileHeld(command);
                            }

                            @Override
                            public void cancelWhenActivated(Command command)
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
            };
        }
    }

    public static class Motors
    {
        public static Motor neo(int canId)
        {
            CANSparkMax motor = new CANSparkMax(canId, MotorType.kBrushless);

            return new Motor()
            {
                @Override
                protected double getRaw()
                {
                    return motor.get();
                }

                @Override
                public PositionSensor getPositionSensor()
                {
                    return null;
                }

                @Override
                public VelocitySensor getVelocitySensor()
                {
                    return null;
                }

                @Override
                public void set(double speed)
                {
                    motor.set(speed);
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
                    return imu.getAngle();
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
    }
}