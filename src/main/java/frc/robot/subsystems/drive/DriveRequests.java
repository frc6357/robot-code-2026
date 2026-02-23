package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Konstants.DriveConstants.kMaxTeleopLinAcceleration;
import static frc.robot.Konstants.DriveConstants.kMaxTeleopRotAcceleration;

import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Konstants.DriveConstants;

/**
 * Contains various swerve drive requests to be used by the drive subsystem.
 */
public class DriveRequests {
    public static SlewRateLimiter xSlewRateLimiter = new SlewRateLimiter(kMaxTeleopLinAcceleration.in(MetersPerSecondPerSecond));
    public static SlewRateLimiter ySlewRateLimiter = new SlewRateLimiter(kMaxTeleopLinAcceleration.in(MetersPerSecondPerSecond));
    public static SlewRateLimiter rotSlewRateLimiter = new SlewRateLimiter(kMaxTeleopRotAcceleration.in(RadiansPerSecondPerSecond));

    public static final SwerveRequest.FieldCentric teleopRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(DriveConstants.kMaxSpeed.times(0.05));

    public static final UnaryOperator<SwerveRequest.FieldCentric> getTeleopRequestUpdater(
        Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> rotJoystick, Supplier<Boolean> slow, Supplier<Boolean> fast
    ) {
        return (SwerveRequest.FieldCentric request) -> {
            if(slow.get()) {
                return request
                        .withVelocityX(xSlewRateLimiter.calculate(DriveConstants.kMaxSpeedSLOW.times(xJoystick.get()).in(MetersPerSecond)))
                        .withVelocityY(ySlewRateLimiter.calculate(DriveConstants.kMaxSpeedSLOW.times(yJoystick.get()).in(MetersPerSecond)))
                        .withRotationalRate(rotSlewRateLimiter.calculate(DriveConstants.kMaxAngularRateSLOW.times(rotJoystick.get()).in(RadiansPerSecond)));
            }
            if(fast.get()) {
                return request
                        .withVelocityX(xSlewRateLimiter.calculate(DriveConstants.kMaxSpeedFAST.times(xJoystick.get()).in(MetersPerSecond)))
                        .withVelocityY(ySlewRateLimiter.calculate(DriveConstants.kMaxSpeedFAST.times(yJoystick.get()).in(MetersPerSecond)))
                        .withRotationalRate(rotSlewRateLimiter.calculate(DriveConstants.kMaxAngularRateFAST.times(rotJoystick.get()).in(RadiansPerSecond)));
            }
            return request
                    .withVelocityX(xSlewRateLimiter.calculate(DriveConstants.kMaxSpeed.times(xJoystick.get()).in(MetersPerSecond)))
                    .withVelocityY(ySlewRateLimiter.calculate(DriveConstants.kMaxSpeed.times(yJoystick.get()).in(MetersPerSecond)))
                    .withRotationalRate(rotSlewRateLimiter.calculate(DriveConstants.kMaxAngularRate.times(rotJoystick.get()).in(RadiansPerSecond)));
        };
    }

    public static final SwerveRequest.RobotCentric robotCentricTeleopRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband(DriveConstants.kMaxSpeed.times(0.1));
    
    public static final UnaryOperator<SwerveRequest.RobotCentric> getRobotCentricTeleopRequestUpdater(
        Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> rotJoystick, Supplier<Boolean> slow, Supplier<Boolean> fast
    ) {
        return (SwerveRequest.RobotCentric request) -> {
            if(slow.get()) {
                return request
                        .withVelocityX(xSlewRateLimiter.calculate(DriveConstants.kMaxSpeedSLOW.times(xJoystick.get()).in(MetersPerSecond)))
                        .withVelocityY(ySlewRateLimiter.calculate(DriveConstants.kMaxSpeedSLOW.times(yJoystick.get()).in(MetersPerSecond)))
                        .withRotationalRate(rotSlewRateLimiter.calculate(DriveConstants.kMaxAngularRateSLOW.times(rotJoystick.get()).in(RadiansPerSecond)));
            }
            if(fast.get()) {
                return request
                        .withVelocityX(xSlewRateLimiter.calculate(DriveConstants.kMaxSpeedFAST.times(xJoystick.get()).in(MetersPerSecond)))
                        .withVelocityY(ySlewRateLimiter.calculate(DriveConstants.kMaxSpeedFAST.times(yJoystick.get()).in(MetersPerSecond)))
                        .withRotationalRate(rotSlewRateLimiter.calculate(DriveConstants.kMaxAngularRateFAST.times(rotJoystick.get()).in(RadiansPerSecond)));
            }
            return request
                    .withVelocityX(xSlewRateLimiter.calculate(DriveConstants.kMaxSpeed.times(xJoystick.get()).in(MetersPerSecond)))
                    .withVelocityY(ySlewRateLimiter.calculate(DriveConstants.kMaxSpeed.times(yJoystick.get()).in(MetersPerSecond)))
                    .withRotationalRate(rotSlewRateLimiter.calculate(DriveConstants.kMaxAngularRate.times(rotJoystick.get()).in(RadiansPerSecond)));
        };
    }
    
    public static final SwerveRequest.ApplyRobotSpeeds pathPlannerRequest = new SwerveRequest.ApplyRobotSpeeds();
    
    public static final UnaryOperator<SwerveRequest.ApplyRobotSpeeds> getPathPlannerRequestUpdater(
        Supplier<ChassisSpeeds> speeds, Supplier<DriveFeedforwards> feedforwards) { 
        return (SwerveRequest.ApplyRobotSpeeds request) -> {
            return request
                .withSpeeds(speeds.get())
                .withWheelForceFeedforwardsX(feedforwards.get().robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.get().robotRelativeForcesYNewtons());
        };
    }

    // public static final UnaryOperator<SwerveRequest.RobotCentric> getPathPlannerRequestUpdater(
    //     Supplier<ChassisSpeeds> speeds, Supplier<DriveFeedforwards> feedforwards) { 
    //     return (SwerveRequest.RobotCentric request) -> {
    //         return request
    //             .withVelocityX(speeds.get().vxMetersPerSecond)
    //             .withVelocityY(speeds.get().vyMetersPerSecond)
    //             .withRotationalRate(speeds.get().omegaRadiansPerSecond);
    //     };
    // }
}
