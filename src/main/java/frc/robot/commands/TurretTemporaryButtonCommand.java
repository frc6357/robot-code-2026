package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Turret;

import static frc.robot.Konstants.TurretConstants.kManualTurretSpeed;

public class TurretTemporaryButtonCommand extends Command
{
    private final SK26Turret turret;
    private final double angle;

    public TurretTemporaryButtonCommand(double angle, SK26Turret turret)
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
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }
}

