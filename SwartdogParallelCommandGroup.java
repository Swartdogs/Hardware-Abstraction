package frc.robot.abstraction;

public class SwartdogParallelCommandGroup extends SwartdogCommandGroup
{
    private boolean[] _commandsRunning;

    public SwartdogParallelCommandGroup(SwartdogCommand... commands)
    {
        _commandsRunning = new boolean[commands.length];

        for (int i = 0; i < _commandsRunning.length; i++)
        {
            _commandsRunning[i] = false;
        }

        add(commands);
    }

    @Override
    public void initialize()
    {
        for (int i = 0; i <_commandsRunning.length; i++)
        {
            getCommands().get(i).initialize();
            _commandsRunning[i] = true;
        }
    }

    @Override
    public void execute()
    {
        for (int i = 0; i < _commandsRunning.length; i++)
        {
            if (_commandsRunning[i])
            {
                getCommands().get(i).execute();

                if (getCommands().get(i).isFinished())
                {
                    getCommands().get(i).end(false);
                    _commandsRunning[i] = false;
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        if (interrupted)
        {
            for (int i = 0; i < _commandsRunning.length; i++)
            {
                if (_commandsRunning[i])
                {
                    getCommands().get(i).end(true);
                }
            }
        }
    }

    @Override
    public boolean isFinished()
    {
        boolean finished = true;

        for (int i = 0; i < _commandsRunning.length; i++)
        {
            if (_commandsRunning[i])
            {
                finished = false;
                break;
            }
        }

        return finished;
    }
}
