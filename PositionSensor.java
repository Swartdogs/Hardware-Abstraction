package frc.robot.abstraction;

import java.util.function.DoubleUnaryOperator;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI;

public abstract class PositionSensor
{
    public enum IMUAxis
    {
        Yaw,
        Pitch,
        Roll
    }

    protected DoubleUnaryOperator _scalingFunction = DoubleUnaryOperator.identity();

    public abstract double get();
    public abstract void   set(double newPosition);

    public void setScalingFunction(DoubleUnaryOperator scalingFunction)
    {
        if (scalingFunction == null)
        {
            scalingFunction = DoubleUnaryOperator.identity();
        }

        _scalingFunction = scalingFunction;
    }

    public static PositionSensor navX(IMUAxis axis)
    {
        return new PositionSensor()
        {
            private AHRS   _navX   = new AHRS(SPI.Port.kMXP);
            private double _offset = 0;

            @Override
            public double get()
            {
                double angle = 0.0;

                switch(axis)
                {
                    case Yaw:
                        angle = _navX.getAngle();
                        break;

                    case Pitch:
                        angle = _navX.getPitch();
                        break;

                    case Roll:
                        angle = _navX.getRoll();
                        break;
                }
                
                return _scalingFunction.applyAsDouble(angle) + _offset;
            }

            @Override
            public void set(double newPosition)
            {
                _navX.reset();
                _offset = newPosition;
            }
        };
    }

    public static PositionSensor gyro()
    {
        return new PositionSensor()
        {
            private ADXRS450_Gyro _gyro   = new ADXRS450_Gyro();
            private double        _offset = 0;
            
            @Override
            public double get()
            {
                return _scalingFunction.applyAsDouble(-_gyro.getRotation2d().getDegrees() + _offset);
            }

            @Override
            public void set(double newPosition)
            {
                _gyro.reset();
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
            public double get()
            {
                return _scalingFunction.applyAsDouble(-(_potentiometer.get() - 360) + _offset);
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
            public double get()
            {
                return _scalingFunction.applyAsDouble(analogInput.getAverageValue() + _offset);
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
            public double get() 
            {
                return _scalingFunction.applyAsDouble(_encoder.get() + _offset);
            }

            @Override
            public void set(double newPosition) 
            {
                _encoder.reset();
                _offset = newPosition;
            }
        };
    }

    public static class MockPositionSensor extends PositionSensor
    {
        private double _position;

        public MockPositionSensor()
        {
            this(0);
        }

        public MockPositionSensor(double initialPosition)
        {
            set(initialPosition);
        }

        @Override
        public double get() 
        {
            return _scalingFunction.applyAsDouble(_position);
        }

        @Override
        public void set(double newPosition)
        {
            _position = newPosition;
        }
        
        public void increment(double amount)
        {
            set(_position + amount);
        }

        public void decrement(double amount)
        {
            set(_position - amount);
        }
    }
}