package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Konstants.TurretConstants.TurretPosition;
import frc.robot.subsystems.turret.SK26Turret;

public class TurretButtonCommand extends Command
{
    private final SK26Turret turret;
    private final TurretPosition angle;

    public TurretButtonCommand(TurretPosition angle, SK26Turret turret)
    {
        this.angle = angle;
        this.turret = turret;

        addRequirements(turret);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        turret.setAngleDegrees(angle);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}

