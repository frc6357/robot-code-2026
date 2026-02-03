// package frc.robot.commands.commandGroups;

// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// //import frc.robot.commands.RunLauncherCommand;
// import frc.robot.subsystems.SK26Launcher;

// import static frc.robot.Konstants.LauncherConstants.kStopLauncher;
// import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherPauseTime;
// import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherRPS;
// import static frc.robot.Konstants.LauncherConstants.kUnJamLauncherRunTime;

// public class LauncherUnJamCommandGroup extends SequentialCommandGroup {

//     SK26Launcher launchermotor;
//     double UnJamLauncherVelocity;

//     public LauncherUnJamCommandGroup(SK26Launcher launchermotor) {

//         addRequirements(launchermotor);
//         //Makes motor rotate in reverse and forward to unjam any stuck game pieces
//         addCommands(
//             Commands.sequence(
//                 //Reverses motor
//                 Commands.runOnce(() -> launchermotor.unJamLauncher(-kUnJamLauncherRPS)),
//                 Commands.waitSeconds(kUnJamLauncherRunTime),

//                 //Stops motor
//                 Commands.runOnce(() -> launchermotor.unJamLauncher(kStopLauncher)),
//                 Commands.waitSeconds(kUnJamLauncherPauseTime),

//                 //Runs motor
//                 Commands.runOnce(() -> launchermotor.unJamLauncher(kUnJamLauncherRPS)),
//                 Commands.waitSeconds(kUnJamLauncherRunTime),

//                 //Stops motor
//                 Commands.runOnce(() -> launchermotor.unJamLauncher(kStopLauncher)),
//                 Commands.waitSeconds(kUnJamLauncherPauseTime))
//                 .repeatedly() //makes the unjam command repeat until button is let go
//                 .finallyDo(() -> launchermotor.stopLauncher()) //stops the launcher once it stops repeating
//         );
//     }
    
// }
