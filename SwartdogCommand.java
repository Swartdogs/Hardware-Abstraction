package frc.robot.abstraction;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class SwartdogCommand implements Command
{
    private Set<Subsystem> _requirements = new HashSet<Subsystem>();

    protected void addRequirements(SwartdogSubsystem... subsystems)
    {
        for (SwartdogSubsystem subsystem : subsystems)
        {
            _requirements.add(subsystem);
        }
    }

    @Override
    public Set<Subsystem> getRequirements()
    {
        return _requirements;
    }

    public static SwartdogCommand runnable(Runnable runnable)
    {
        return new SwartdogCommand()
        {
            @Override
            public void initialize()
            {
                System.out.println("Command run");
                runnable.run();
            }

            @Override
            public boolean isFinished()
            {
                return true;
            }
        };
    }
}
