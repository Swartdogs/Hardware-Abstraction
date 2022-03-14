package frc.robot.abstraction;

import java.util.function.DoubleUnaryOperator;

public abstract class VelocitySensor
{
    protected double              _velocity;
    protected DoubleUnaryOperator _scalingFunction = DoubleUnaryOperator.identity();

    public abstract double get();

    public void setScalingFunction(DoubleUnaryOperator scalingFunction)
    {
        if (scalingFunction == null)
        {
            scalingFunction = DoubleUnaryOperator.identity();
        }

        _scalingFunction = scalingFunction;
    }

    public static class MockVelocitySensor extends VelocitySensor
    {
        private double _velocity;
    
        public MockVelocitySensor()
        {
            this(0);
        }
    
        public MockVelocitySensor(double initialVelocity)
        {
            set(initialVelocity);
        }
    
        @Override
        public double get() 
        {
            return _velocity;
        }
        
        public void increment(double amount)
        {
            set(_velocity + amount);
        }
    
        public void decrement(double amount)
        {
            set(_velocity - amount);
        }
    
        public void set(double velocity)
        {
            _velocity = velocity;
        }
    }    
}
