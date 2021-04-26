package frc.robot.abstraction;

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
            public double get()
            {
                return -toInvert.get();
            }

            public PositionSensor getPositionSensor()
            {
                return toInvert.getPositionSensor();
            }

            public VelocitySensor getVelocitySensor()
            {
                return toInvert.getVelocitySensor();
            }

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
            public double get()
            {
                double speed = 0;

                if (motors.length > 0)
                {
                    speed = motors[0].get();
                }

                return speed;
            }

            public PositionSensor getPositionSensor()
            {
                PositionSensor sensor = null;

                if (motors.length > 0)
                {
                    sensor = motors[0].getPositionSensor();
                }

                return sensor;
            }

            public VelocitySensor getVelocitySensor()
            {
                VelocitySensor sensor = null;

                if (motors.length > 0)
                {
                    sensor = motors[0].getVelocitySensor();
                }

                return sensor;
            }

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
