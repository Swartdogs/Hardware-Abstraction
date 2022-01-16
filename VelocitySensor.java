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
}
