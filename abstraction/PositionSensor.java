package frc.robot.abstraction;

import java.util.function.DoubleUnaryOperator;

public abstract class PositionSensor
{
    private DoubleUnaryOperator _scalingFunction = DoubleUnaryOperator.identity();

    public abstract double getRawPosition();
    public abstract void   reset();

    public double get()
    {
        return _scalingFunction.applyAsDouble(getRawPosition());
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
