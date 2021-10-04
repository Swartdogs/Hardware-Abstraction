package frc.robot.abstraction;

import java.util.function.DoubleUnaryOperator;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public abstract class PositionSensor implements Abstraction
{
    public enum IMUAxis
    {
        X,
        Y,
        Z
    }

    private double              _position;
    private DoubleUnaryOperator _scalingFunction = DoubleUnaryOperator.identity();

    protected abstract double getRaw();
    public    abstract void   set(double newPosition);

    public double get()
    {
        return _scalingFunction.applyAsDouble(_position);
    }

    public void setScalingFunction(DoubleUnaryOperator scalingFunction)
    {
        if (scalingFunction == null)
        {
            scalingFunction = DoubleUnaryOperator.identity();
        }

        _scalingFunction = scalingFunction;
    }

    public void cache()
    {
        _position = getRaw();
    }

    public static PositionSensor gyro()
    {
        return new PositionSensor()
        {
            private ADXRS450_Gyro _gyro   = new ADXRS450_Gyro();
            private double        _offset = 0;
            
            @Override
            protected double getRaw()
            {
                return -_gyro.getRotation2d().getDegrees() + _offset;
            }

            @Override
            public void set(double newPosition)
            {
                _gyro.reset();
                _offset = newPosition;
            }
        };
    }

    public static PositionSensor imu(IMUAxis axis, boolean inverted)
    {
        return new PositionSensor()
        {
            private ADIS16448_IMU _imu    = new ADIS16448_IMU();
            private double        _offset = 0;

            @Override
            protected double getRaw()
            {
                double raw = 0;

                switch (axis)
                {
                    case X:
                        raw = _imu.getGyroAngleX();
                        break;
                    
                    case Y:
                        raw = _imu.getGyroAngleY();
                        break;

                    case Z:
                        raw = _imu.getGyroAngleZ();
                        break;
                }

                if (inverted)
                {
                    raw *= -1;
                }

                return raw + _offset;
            }

            @Override
            public void set(double newPosition)
            {
                _imu.reset();
                _offset = newPosition;
            }
        };
    }

    public static PositionSensor potentiometer(int port, double scale, double offset)
    {
        return new PositionSensor()
        {
            private AnalogPotentiometer _potentiometer = new AnalogPotentiometer(port, scale, offset);
            private double              _offset = 0;

            @Override
            protected double getRaw()
            {
                return -(_potentiometer.get() - 360) + _offset;
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
        @SuppressWarnings("resource")
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

    public static PositionSensor dutyCycleEncoder(int port)
    {
        return new PositionSensor()
        {
            private DutyCycleEncoder _encoder = new DutyCycleEncoder(port);
            private double           _offset = 0;

            @Override
            protected double getRaw() 
            {
                return _encoder.get() + _offset;
            }

            @Override
            public void set(double newPosition) 
            {
                _encoder.reset();
                _offset = newPosition;
            }
        };
    }
}
