package frc.robot.abstraction;

public abstract class Motor implements Abstraction
{
    private double _speed;

    protected abstract double getRaw();

    public abstract PositionSensor getPositionSensor();
    public abstract VelocitySensor getVelocitySensor();
    public abstract void           set(double speed);

    public double get()
    {
        return _speed;
    }

    public void cache()
    {
        _speed = getRaw();
    }

    public static Motor invert(Motor toInvert)
    {
        return new Motor()
        {
            @Override
            protected double getRaw()
            {
                return -toInvert.getRaw();
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
            protected double getRaw()
            {
                double speed = 0;

                if (motors.length > 0)
                {
                    speed = motors[0].getRaw();
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
}
