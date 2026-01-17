// package frc.robot.commands.commandGroups;

// import static frc.robot.Konstants.IndexerConstants.kIndexerIdleRPS;
// import static frc.robot.Konstants.IndexerConstants.kIndexerUnjamForwardDuration;
// import static frc.robot.Konstants.IndexerConstants.kIndexerUnjamForwardRPS;
// import static frc.robot.Konstants.IndexerConstants.kIndexerUnjamReverseDuration;
// import static frc.robot.Konstants.IndexerConstants.kIndexerUnjamReverseRPS;
// import static frc.robot.Konstants.IndexerConstants.kIndexerUnjamWaitDuration;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.SK26Indexer;

// public class UnjamCommand extends Command
// {
//   SK26Indexer indexer;

//   public UnjamCommand(SK26Indexer indexer)
//   {
//     this.indexer = indexer;
//   }
  
//   @Override
//   public void execute() {
//     indexer.setIndexerVelocity(kIndexerUnjamReverseRPS);
//     Timer.delay(kIndexerUnjamReverseDuration);
//     indexer.setIndexerVelocity(kIndexerIdleRPS);
//     Timer.delay(kIndexerUnjamWaitDuration);
//     indexer.setIndexerVelocity(kIndexerUnjamForwardRPS);
//     Timer.delay(kIndexerUnjamForwardDuration);
//     indexer.setIndexerVelocity(kIndexerIdleRPS);
//     Timer.delay(kIndexerUnjamWaitDuration);
//   }

//   @Override
//   public void end(boolean interrupted) {
//     indexer.setIndexerVelocity(kIndexerIdleRPS);
//   }


//   @Override
//   public void initialize() {
    
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }

  
//     // public static Command getCommand(SK26Indexer indexer) {
//     //     return Commands.repeatingSequence(
//     //         Commands.race(
//     //           Commands.waitSeconds(kIndexerUnjamReverseDuration),
//     //           indexer.setIndexerVelCommand(kIndexerUnjamReverseRPS)
//     //         ),

//     //       indexer.setIndexerVelCommand(kIndexerIdleRPS),
//     //       Commands.waitSeconds(kIndexerUnjamWaitDuration),

//     //       Commands.race(
//     //           Commands.waitSeconds(kIndexerUnjamForwardDuration),
//     //           indexer.setIndexerVelCommand(kIndexerUnjamForwardRPS)
//     //       ),

//     //       indexer.setIndexerVelCommand(kIndexerIdleRPS),
//     //       Commands.waitSeconds(kIndexerUnjamWaitDuration)
//     //     );
//     // }
//   }
