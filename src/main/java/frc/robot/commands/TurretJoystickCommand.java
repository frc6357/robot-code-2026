package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SK26Turret;

import static frc.robot.Konstants.TurretConstants.kDutyCycleOff;
import static frc.robot.Konstants.TurretConstants.kManualTurretSpeed;

public class TurretJoystickCommand extends Command {

    private SK26Turret turret;
    private Supplier<Double> joystickInput;

    public TurretJoystickCommand(SK26Turret turret, Supplier<Double> joystickInput) 
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
        double input = joystickInput.get();
        if (Math.abs(input) < 0.05) 
        {
            turret.holdPosition();
            return;
        }
        double dutyCycle = input * kManualTurretSpeed;

        turret.manualRotate(dutyCycle);
    }

    @Override
    public void end(boolean interrupted) 
    {
        turret.manualRotate(kDutyCycleOff);
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
