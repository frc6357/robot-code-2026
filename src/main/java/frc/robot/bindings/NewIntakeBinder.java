package frc.robot.bindings;

import java.util.Optional;
import frc.robot.subsystems.NewIntake;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Ports.DrivePorts.*;
import static frc.robot.Ports.DriverPorts.kDownDpad;
import static frc.robot.Ports.DriverPorts.kUpDpad;

public class NewIntakeBinder implements CommandBinder
{
    Optional<NewIntake> subsystem;
    Trigger spinA;
    Trigger spinB;

    public NewIntakeBinder(Optional<NewIntake> pobSys)
    {
        subsystem = pobSys;
        spinA = kUpDpad;
        spinB = kDownDpad;
    }
}
