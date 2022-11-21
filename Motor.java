package frc.robot.abstraction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.motorcontrol.Jaguar;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.abstraction.PositionSensor.MockPositionSensor;
import frc.robot.abstraction.VelocitySensor.MockVelocitySensor;

@SuppressWarnings("resource")
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

    public static Motor fromNeo(CANSparkMax motor)
    {
        return new Motor()
        {
            RelativeEncoder _encoder = motor.getEncoder();

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

    public static Motor neo(int canId)
    {
        CANSparkMax _motor = new CANSparkMax(canId, MotorType.kBrushless);

        _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
        _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
        _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);

        return Motor.fromNeo(_motor);
    }

    public static Motor fromNeoFlywheel(CANSparkMax motor, double maxMotorRPM)
    {
        return new Motor()
        {
            private RelativeEncoder _encoder = motor.getEncoder();

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
                motor.set(_setpoint / maxMotorRPM);
            }
        };
    }

    public static Motor neoFlywheel(int canId, double maxMotorRPM)
    {
        CANSparkMax _motor = new CANSparkMax(canId, MotorType.kBrushless);

        _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
        _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 100);
        _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 100);
        _motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);

        return Motor.fromNeoFlywheel(_motor, maxMotorRPM);
    }

    public static Motor fromFalcon(TalonFX motor)
    {
        return new Motor()
        {
            private double  _speed;

            private PositionSensor _positionSensor = new PositionSensor()
            {
                @Override
                public double get()
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
                public double get()
                {
                    return motor.getSensorCollection().getIntegratedSensorVelocity();
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
                motor.set(ControlMode.PercentOutput, speed);
            }
        };
    }

    public static Motor falcon(int canId)
    {
        TalonFX _motor = new TalonFX(canId);

        _motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        for (StatusFrameEnhanced frame : StatusFrameEnhanced.values())
        {
            _motor.setStatusFramePeriod(frame, 100);
        }

        for (StatusFrame frame : StatusFrame.values())
        {
            _motor.setStatusFramePeriod(frame, 100);
        }

        _motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        _motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
        _motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        _motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20);

        return Motor.fromFalcon(_motor);
    }

    public static Motor fromFalconFlywheel(TalonFX motor, double scalingFactor)
    {
        return new Motor()
        {
            private double _scalingFactor        = scalingFactor;
            private double _inverseScalingFactor = 1.0 / scalingFactor;

            private final double RPM_TO_SENSOR_CONVERSION = 256.0 / 75.0;
            private final double SENSOR_TO_RPM_CONVERSION =   1.0 / RPM_TO_SENSOR_CONVERSION;

            private VelocitySensor _velocitySensor = new VelocitySensor()
            {
                @Override
                public double get()
                {
                    return motor.getSensorCollection().getIntegratedSensorVelocity() * SENSOR_TO_RPM_CONVERSION * _inverseScalingFactor;
                }
            };

            @Override
            public double get() 
            {
                double value = 0;

                if (motor.getControlMode() == ControlMode.Velocity)
                {
                    value = motor.getClosedLoopTarget() * SENSOR_TO_RPM_CONVERSION * _inverseScalingFactor;
                }

                return value;
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
                if (speed == 0)
                {
                    motor.set(ControlMode.PercentOutput, 0);
                }
                else
                {
                    motor.set(ControlMode.Velocity, speed * RPM_TO_SENSOR_CONVERSION * _scalingFactor);
                }
            }
        };
    }

    public static Motor falconFlywheel(int canId, double scalingFactor)
    {
        TalonFX _motor = new TalonFX(canId);
            
        _motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        for (StatusFrameEnhanced frame : StatusFrameEnhanced.values())
        {
            _motor.setStatusFramePeriod(frame, 100);
        }

        for (StatusFrame frame : StatusFrame.values())
        {
            _motor.setStatusFramePeriod(frame, 100);
        }

        _motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        _motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
        _motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        _motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20);

        return Motor.fromFalconFlywheel(_motor, scalingFactor);
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

    public static Motor fromTalonSRX(TalonSRX motor)
    {
        return new Motor()
        {
            @Override
            public double get()
            {
                return motor.getMotorOutputPercent();
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
                motor.set(ControlMode.PercentOutput, speed);
            }
        };
    }

    public static Motor talonSRX(int canId)
    {
        TalonSRX _motor = new TalonSRX(canId);

        _motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.None, 0, 0);

        for (StatusFrameEnhanced frame : StatusFrameEnhanced.values())
        {
            _motor.setStatusFramePeriod(frame, 100);
        }

        for (StatusFrame frame : StatusFrame.values())
        {
            _motor.setStatusFramePeriod(frame, 100);
        }

        _motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        _motor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
        _motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        _motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 20);

        return Motor.fromTalonSRX(_motor);
    }

    public static Motor fromJaguar(Jaguar motor)
    {
        return new Motor()
        {
            @Override
            public double get()
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

    public static Motor jaguar(int pwmPort)
    {
        return Motor.fromJaguar(new Jaguar(pwmPort));
    }
    
    public static class MockMotor extends Motor
    {
        private double          _speed;
        private PositionSensor  _positionSensor;
        private VelocitySensor  _velocitySensor;

        public MockMotor()
        {
            _speed          = 0;
            _positionSensor = new MockPositionSensor();
            _velocitySensor = new MockVelocitySensor();
        }

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
        }
    }
}
