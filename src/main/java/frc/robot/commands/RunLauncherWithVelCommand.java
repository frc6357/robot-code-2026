// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SK26Launcher;

// public class RunLauncherWithVelCommand extends Command {

//     SK26Launcher launchermotor;
//     double targetLaunchVelocity;
    
//     public RunLauncherWithVelCommand(SK26Launcher launchermotor, double targetLaunchVelocity) {
//         this.launchermotor = launchermotor;
//         this.targetLaunchVelocity = targetLaunchVelocity;
//     }

//     @Override
//     public void initialize() {

//     }

//     @Override
//     public void execute() {
//         launchermotor.runLauncherExitVel(targetLaunchVelocity, "Launching");
//     }

//     @Override
//     public void end(boolean interrupted) {
//         //launchermotor.stopLauncher();
//         launchermotor.coastLauncher(); //makes the launcher quickly relaunchable
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

// }
