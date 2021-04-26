package frc.robot.abstraction;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class SwartdogSubsystem implements Subsystem
{
    public static boolean REGISTER_SUBSYSTEMS = true;

    public SwartdogSubsystem()
    {
        if (REGISTER_SUBSYSTEMS)
        {
            CommandScheduler.getInstance().registerSubsystem(this);
        }
    }
}
