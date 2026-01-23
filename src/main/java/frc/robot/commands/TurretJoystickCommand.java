package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Turret;

import static frc.robot.Konstants.TurretConstants.kManualTurretSpeed;

public class TurretJoystickCommand extends Command
{
    private final SK26Turret turret;
    private final DoubleSupplier joystickAxis;

    private double lastTimestamp;

    public TurretJoystickCommand(SK26Turret turret, DoubleSupplier joystickAxis)
    {
        this.turret = turret;
        this.joystickAxis = joystickAxis;

        addRequirements(turret);
    }

    @Override
    public void initialize()
    {
        lastTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute()
    {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTimestamp;
        lastTimestamp = now;

        double input = joystickAxis.getAsDouble();

        double angularVelocity = input * kManualTurretSpeed;
        double newAngle = turret.getAngleDegrees() + angularVelocity * dt;
        turret.setAngleDegrees(newAngle);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
