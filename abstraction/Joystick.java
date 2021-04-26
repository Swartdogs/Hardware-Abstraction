package frc.robot.abstraction;

public abstract class Joystick
{
    public abstract double getX();
    public abstract double getY();
    public abstract double getZ();
    public abstract double getThrottle();
    public abstract Switch getButton(int buttonNum);
}
