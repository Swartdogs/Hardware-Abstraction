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

    public static SwartdogCommand run(Runnable runnable)
    {
        return run(runnable, false);
    }

    public static SwartdogCommand run(Runnable runnable, boolean runsWhenDisabled)
    {
        return new SwartdogCommand()
        {
            @Override
            public void initialize()
            {
                runnable.run();
            }

            @Override
            public boolean isFinished()
            {
                return true;
            }

            @Override
            public boolean runsWhenDisabled()
            {
                return runsWhenDisabled;
            }
        };
    }
}
