package frc.robot.abstraction;

import java.util.function.DoubleUnaryOperator;

public abstract class PositionSensor implements Abstraction
{
    private double              _position;
    private DoubleUnaryOperator _scalingFunction = DoubleUnaryOperator.identity();

    protected abstract double getRaw();

    public abstract void reset();

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
}
