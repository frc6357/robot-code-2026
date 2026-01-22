package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Turret;

import static frc.robot.Konstants.TurretConstants.kManualTurretSpeed;

public class TurretJoystickCommand2 extends Command {

    private SK26Turret turret;
    private Supplier<Double> joystickInput;

    public TurretJoystickCommand2(SK26Turret turret, Supplier<Double> joystickInput) 
    {
        this.turret = turret;
        this.joystickInput = joystickInput;
        
        addRequirements(turret);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() 
    {
        turret.runTurret(kManualTurretSpeed);
    }

    @Override
    public void end(boolean interrupted) 
    {
        turret.stop();
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
