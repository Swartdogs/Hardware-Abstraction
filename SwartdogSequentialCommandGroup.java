package frc.robot.abstraction;

public class SwartdogSequentialCommandGroup extends SwartdogCommandGroup 
{
    private int _commandIndex;

    public SwartdogSequentialCommandGroup(SwartdogCommand... commands)
    {
        _commandIndex = -1;

        add(commands);
    }

    @Override
    public void initialize()
    {
        _commandIndex = 0;

        if (!getCommands().isEmpty())
        {
            getCommands().get(0).initialize();
        }
    }

    @Override
    public void execute()
    {
        if (!getCommands().isEmpty())
        {
            SwartdogCommand currentCommand = getCommands().get(_commandIndex);

            currentCommand.execute();

            if (currentCommand.isFinished())
            {
                currentCommand.end(false);
                _commandIndex++;

                if (_commandIndex < getCommands().size())
                {
                    getCommands().get(_commandIndex).initialize();
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        if (interrupted && _commandIndex > -1 && _commandIndex < getCommands().size())
        {
            getCommands().get(_commandIndex).end(true);
        }

        _commandIndex = -1;
    }

    @Override
    public boolean isFinished()
    {
        return _commandIndex >= getCommands().size();
    }
}
