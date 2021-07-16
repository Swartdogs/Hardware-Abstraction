package frc.robot.abstraction;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class SwartdogCommandGroup extends SwartdogCommand
{
    private ArrayList<SwartdogCommand> _commands;

    public SwartdogCommandGroup()
    {
        _commands = new ArrayList<SwartdogCommand>();
    }

    public void add(SwartdogCommand... commands)
    {
        for (SwartdogCommand command : commands)
        {
            _commands.add(command);

            for (Subsystem subsystem : command.getRequirements())
            {
                addRequirements((SwartdogSubsystem)subsystem);
            }
        }
    }

    protected List<SwartdogCommand> getCommands()
    {
        return _commands;
    }
}
