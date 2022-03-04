package frc.robot.abstraction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public abstract class Motor
{
    public abstract double         get();
    public abstract PositionSensor getPositionSensor();
    public abstract VelocitySensor getVelocitySensor();
    public abstract void           set(double speed);

    public static Motor invert(Motor toInvert)
    {
        return new Motor()
        {
            @Override
            public double get()
            {
                return -toInvert.get();
            }

            @Override
            public PositionSensor getPositionSensor()
            {
                return toInvert.getPositionSensor();
            }

            @Override
            public VelocitySensor getVelocitySensor()
            {
                return toInvert.getVelocitySensor();
            }

            @Override
            public void set(double speed)
            {
                toInvert.set(-speed);
            }
        };
    }

    public static Motor compose(Motor... motors)
    {
        return new Motor()
        {
            @Override
            public double get()
            {
                double speed = 0;

                if (motors.length > 0)
                {
                    speed = motors[0].get();
                }

                return speed;
            }

            @Override
            public PositionSensor getPositionSensor()
            {
                PositionSensor sensor = null;

                if (motors.length > 0)
                {
                    sensor = motors[0].getPositionSensor();
                }

                return sensor;
            }

            @Override
            public VelocitySensor getVelocitySensor()
            {
                VelocitySensor sensor = null;

                if (motors.length > 0)
                {
                    sensor = motors[0].getVelocitySensor();
                }

                return sensor;
            }

            @Override
            public void set(double speed)
            {
                for (Motor m : motors)
                {
                    m.set(speed);
                }
            }
        };
    }

    public static Motor neo(int canId)
    {
        return new Motor()
        {
            private CANSparkMax     _motor   = new CANSparkMax(canId, MotorType.kBrushless);
            private RelativeEncoder _encoder = _motor.getEncoder();
    
            private PositionSensor _positionSensor = new PositionSensor()
            {

                @Override
                public double get() 
                {
                    return _encoder.getPosition();
                }

                @Override
                public void set(double newPosition) 
                {
                    _encoder.setPosition(newPosition);
                }
            };

            private VelocitySensor _velocitySensor = new VelocitySensor()
            {
                @Override
                public double get() 
                {
                    return _encoder.getVelocity();
                }
            };

            @Override
            public double get()
            {
                return _motor.get();
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
                _motor.set(speed);
            }
        };
    }

    public static Motor neoFlywheel(int canId, double maxMotorRPM)
    {
        return new Motor()
        {
            private CANSparkMax     _motor    = new CANSparkMax(canId, MotorType.kBrushless);
            private RelativeEncoder _encoder  = _motor.getEncoder();
            private double          _setpoint = 0;

            private VelocitySensor _velocitySensor = new VelocitySensor()
            {
                @Override 
                public double get()
                {
                    return _encoder.getVelocity();
                }
            };

            @Override
            public double get()
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
                _motor.set(_setpoint / maxMotorRPM);
            }
        };
    }

    public static Motor falcon(int canId)
    {
        return new Motor()
        {
            private TalonFX _motor = new TalonFX(canId);
            private double  _speed;

            private PositionSensor _positionSensor = new PositionSensor()
            {
                @Override
                public double get()
                {
                    return _motor.getSensorCollection().getIntegratedSensorPosition();
                }

                @Override
                public void set(double newPosition)
                {
                    _motor.getSensorCollection().setIntegratedSensorPosition(newPosition, 0);
                }
            };

            private VelocitySensor _velocitySensor = new VelocitySensor()
            {
                @Override
                public double get()
                {
                    return _motor.getSensorCollection().getIntegratedSensorVelocity();
                }
            };

            @Override
            public double get()
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
                _motor.set(ControlMode.PercentOutput, speed);
            }
        };
    }

    public static Motor falconFlywheel(int canId, double maxMotorRPM)
    {
        return new Motor()
        {
            private TalonFX _motor = new TalonFX(canId);
            private double  _setpoint;

            private final double SENSOR_TO_RPM_CONVERSION = 600.0 / 2048.0;

            private VelocitySensor _velocitySensor = new VelocitySensor()
            {
                @Override
                public double get()
                {
                    return _motor.getSensorCollection().getIntegratedSensorVelocity() * SENSOR_TO_RPM_CONVERSION;
                }
            };

            @Override
            public double get() 
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
                _motor.set(ControlMode.PercentOutput, _setpoint / maxMotorRPM);
            }
        };
    }

    public static Motor victorSP(int port)
    {
        return new Motor()
        {
            private VictorSP _motor = new VictorSP(port);

            @Override
            public double get() 
            {
                return _motor.get();
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
                _motor.set(speed);
            }
        };
    }

    public static Motor victorSPX(int canId)
    {
        return new Motor()
        {
            private WPI_VictorSPX _motor = new WPI_VictorSPX(canId);

            @Override
            public double get() 
            {
                return _motor.get();
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
                _motor.set(speed);
            }
        };
    }

    public static Motor talonSRX(int canId)
    {
        return new Motor()
        {
            private TalonSRX _motor = new TalonSRX(canId);

            @Override
            public double get()
            {
                return _motor.getMotorOutputPercent();
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
                _motor.set(ControlMode.PercentOutput, speed);
            }
        };
    }
}
