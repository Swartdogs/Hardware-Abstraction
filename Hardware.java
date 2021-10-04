package frc.robot.abstraction;

import java.util.ArrayList;

public final class Hardware extends SwartdogSubsystem
{
    private ArrayList<Abstraction> _hardware;

    public Hardware(Abstraction... hardware)
    {
        _hardware = new ArrayList<Abstraction>();

        addHardware(hardware);
    }

    @Override
    public void periodic()
    {
        cacheAll();
    }

    public void addHardware(Abstraction... hardware)
    {
        for (Abstraction abstraction : hardware)
        {
            abstraction.cache();
            _hardware.add(abstraction);
        }
    }

    private void cacheAll()
    {
        for (Abstraction abstraction : _hardware)
        {
            abstraction.cache();
        }
    }
}
