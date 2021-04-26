package frc.robot.abstraction;

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
public final class Hardware
{
    private Hardware()
    {
        // do nothing
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

                public double getX()
                {
                    return _joystick.getX();
                }

                public double getY()
                {
                    return -_joystick.getY();
                }

                public double getZ()
                {
                    return _joystick.getZ();
                }

                public double getThrottle()
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
                            public State getState()
                            {
                                return _joystick.getRawButton(buttonNum) ? State.On : State.Off;
                            }

                            public void whenActivated(Command command)
                            {
                                _buttons[buttonNum - 1].whenPressed(command);
                            }

                            public void whileActive(Command command)
                            {
                                _buttons[buttonNum - 1].whileHeld(command);
                            }

                            public void cancelWhenActivated(Command command)
                            {
                                _buttons[buttonNum - 1].cancelWhenPressed(command);
                            }
                        };
                    }

                    return _switches[buttonNum - 1];
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
                public double get()
                {
                    return motor.get();
                }

                public PositionSensor getPositionSensor()
                {
                    return null;
                }

                public VelocitySensor getVelocitySensor()
                {
                    return null;
                }

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
                    public double getRawPosition()
                    {
                        return motor.getSensorCollection().getIntegratedSensorPosition();
                    }

                    public void reset()
                    {
                        motor.getSensorCollection().setIntegratedSensorPosition(0, 0);
                    }
                };

                private VelocitySensor _velocitySensor = new VelocitySensor()
                {
                    public double getRawVelocity()
                    {
                        return motor.getSensorCollection().getIntegratedSensorVelocity();
                    }
                };

                public double get()
                {
                    return _speed;
                }

                public PositionSensor getPositionSensor()
                {
                    return _positionSensor;
                }

                public VelocitySensor getVelocitySensor()
                {
                    return _velocitySensor;
                }

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
                public double getRawPosition()
                {
                    return -gyro.getRotation2d().getDegrees();
                }

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
                public double getRawPosition()
                {
                    return imu.getAngle();
                }

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
                public double getRawPosition()
                {
                    return -(potentiometer.get() - 360);
                }

                public void reset()
                {
                    return;
                }
            };
        }
    }
}
