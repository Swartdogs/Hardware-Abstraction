package frc.robot.abstraction;

import java.util.function.DoubleUnaryOperator;

public abstract class VelocitySensor implements Abstraction
{
    private double              _velocity;
    private DoubleUnaryOperator _scalingFunction = DoubleUnaryOperator.identity();

    protected abstract double getRaw();

    public double get()
    {
        return _scalingFunction.applyAsDouble(_velocity);
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
        _velocity = getRaw();
    }
}
