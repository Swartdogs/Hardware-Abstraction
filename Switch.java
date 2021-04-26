package frc.robot.abstraction;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.abstraction.Enumerations.State;

public abstract class Switch
{
    public abstract State getState();
    public abstract void  whenActivated(Command command);
    public abstract void  whileActive(Command command);
    public abstract void  cancelWhenActivated(Command command);
}
