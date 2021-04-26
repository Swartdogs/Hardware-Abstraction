package frc.robot.abstraction;

import java.util.function.DoubleUnaryOperator;

public abstract class VelocitySensor
{
    private DoubleUnaryOperator _scalingFunction = DoubleUnaryOperator.identity();

    public abstract double getRawVelocity();

    public double get()
    {
        return _scalingFunction.applyAsDouble(getRawVelocity());
    }

    public void setScalingFunction(DoubleUnaryOperator scalingFunction)
    {
        if (scalingFunction == null)
        {
            scalingFunction = DoubleUnaryOperator.identity();
        }

        _scalingFunction = scalingFunction;
    }
}
