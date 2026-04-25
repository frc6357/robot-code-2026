package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Konstants.ClimbConstants.kCLimbMax;
import static frc.robot.Konstants.ClimbConstants.kClimbD;
import static frc.robot.Konstants.ClimbConstants.kClimbI;
import static frc.robot.Konstants.ClimbConstants.kClimbP;
import static frc.robot.Konstants.ClimbConstants.kClimbTolerance;
import static frc.robot.Konstants.ClimbConstants.kClimbMotorSpeed;

import frc.robot.Konstants.ClimbConstants.ClimbPosition;
import lombok.Getter;

import static frc.robot.Ports.ClimbPorts.kClimbMotor;
import static frc.robot.Ports.ClimbPorts.kClimbMotorTwo;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkFlexConfig;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Climb extends SubsystemBase
{
    private final SparkFlex climbMotorRight;
    private final SparkFlex climbMotorLeft;
    private final SparkLimitSwitch forwardLimitRight;
    private final SparkLimitSwitch reverseLimitRight;
    private final SparkLimitSwitch forwardLimitLeft;
    private final SparkLimitSwitch reverseLimitLeft;
    
    private final SparkClosedLoopController cpidRight;
    private final SparkClosedLoopController cpidLeft;

    private final RelativeEncoder cEncoderRight;
    private final RelativeEncoder cEncoderLeft;

    private final double motorRatio = 1.0;
    private final double climbFactor = 1.0;

    private double cTargetHeight = 0.0;

    @Getter
    private boolean running = false;

    @Getter
    private ClimbPosition targetClimbEnum = ClimbPosition.STOW;

    public SK26Climb()
    {
        climbMotorRight = new SparkFlex(kClimbMotor.ID, MotorType.kBrushless);
        climbMotorLeft = new SparkFlex(kClimbMotorTwo.ID, MotorType.kBrushless);
        cEncoderRight = climbMotorRight.getEncoder(); 
        cEncoderLeft = climbMotorLeft.getEncoder();
        
        forwardLimitRight = climbMotorRight.getForwardLimitSwitch();
        reverseLimitRight = climbMotorRight.getReverseLimitSwitch();
        forwardLimitLeft = climbMotorLeft.getForwardLimitSwitch();
        reverseLimitLeft = climbMotorLeft.getReverseLimitSwitch();

        SparkFlexConfig climbConfigRight = new SparkFlexConfig();
        SparkFlexConfig climbConfigLeft = new SparkFlexConfig();

        climbConfigRight.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kClimbP)
            .i(kClimbI)
            .d(kClimbD);

        climbConfigLeft.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kClimbP)
            .i(kClimbI)
            .d(kClimbD);

        climbConfigRight.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

        climbConfigLeft.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

        climbMotorRight.configure(climbConfigRight, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        climbMotorLeft.configure(climbConfigLeft, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        cEncoderRight.setPosition(0);
        cEncoderLeft.setPosition(0);
        cpidRight = climbMotorRight.getClosedLoopController();
        cpidLeft = climbMotorLeft.getClosedLoopController();
    }

    /** Runs both climb motors at the given speed. */
    public void runMotors(double motorSpeed)
    {
        climbMotorRight.set(motorSpeed);
        climbMotorLeft.set(motorSpeed);
    }

    /** Stops both climb motors. */
    public void stopMotors()
    {
        climbMotorRight.stopMotor();
        climbMotorLeft.stopMotor();
    }

    /** Checks if the climb is within tolerance of the target height. */
    public boolean climbIsAtHeight()
    {
        double tolerance = DriverStation.isEnabled() 
            ? kClimbTolerance 
            : Rotations.of(0.1).in(Degrees);
        return Math.abs(getClimbTargetPosition() - getClimbPosition()) < tolerance;
    }

    public boolean climbIsAtPosition(ClimbPosition position)
    {
        double tolerance = DriverStation.isEnabled() 
            ? kClimbTolerance 
            : Rotations.of(0.1).in(Degrees);
        return Math.abs(position.height - getClimbPosition()) < tolerance;
    }

    public double getClimbTargetPosition()
    {
        return cTargetHeight;
    }

    public double getClimbPosition()
    {
        return cEncoderRight.getPosition() * climbFactor;
    }

    public boolean isForwardLimitReached()
    {
        if(forwardLimitRight.isPressed() || forwardLimitLeft.isPressed())
        {
            return true;
        }
        
        else
        {
            return false;
        }
        
    } 

   
    public boolean isReverseLimitReached()
    {
        if (reverseLimitLeft.isPressed() || reverseLimitRight.isPressed())
        {
            return true;
        }

        else
        {
            return false;
        }
        
    }

    

    /** Sets the target height for the climb and commands both PID controllers. */
    public void setClimbHeight(double targetHeight)
    {
        cTargetHeight = targetHeight;
        double motorRotations = targetHeight * motorRatio;
        cpidRight.setSetpoint(motorRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        cpidLeft.setSetpoint(motorRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /** Holds the current position to resist gravity. */
    public void hold()
    {
        if (running)
        {
            cTargetHeight = getClimbPosition();
            running = false;
        }
        setClimbHeight(cTargetHeight);
    }

    /** Checks if the climb arm is at its max height (within tolerance). */
    public boolean cAtMaxHeight()
    {
        double currentPosition = getClimbPosition();
        Logger.recordOutput("Climb/Check Max/CurrentPosition", currentPosition);
        Logger.recordOutput("Climb/Check Max/MaxHeight", kCLimbMax);
        return Math.abs(currentPosition - kCLimbMax) < kClimbTolerance;
    }

    public void setIsRunning(boolean running) {
        this.running = running;
    }
    
    // ==================== Inline Command Factories ====================

    /**
     * Returns a command that runs the climb motors upward and stops when ended.
     * Replaces the standalone ClimbUpCommand.
     *
     * @return A command requiring this subsystem.
     */
    public Command climbUpCommand() {
        return this.runEnd(
            () -> { runMotors(kClimbMotorSpeed); setIsRunning(true); },
            this::stopMotors
        );
    }

    /**
     * Returns a command that runs the climb motors downward and stops when ended.
     * Replaces the standalone ClimbDownCommand.
     *
     * @return A command requiring this subsystem.
     */
    public Command climbDownCommand() {
        return this.runEnd(
            () -> { runMotors(-kClimbMotorSpeed); setIsRunning(true); },
            this::stopMotors
        );
    }

    /**
     * Returns a command that sets the climb to a target height using PID
     * and finishes when the height is reached.
     * Replaces the standalone ClimbButtonCommand.
     *
     * @param height The target climb height.
     * @return A command requiring this subsystem.
     */
    public Command climbToHeightCommand(double height) {
        return this.runOnce(() -> { setClimbHeight(height); setIsRunning(true); })
                   .andThen(this.run(() -> {}))
                   .until(this::climbIsAtHeight);
    }

    /**
     * Returns a command that sets the climb to a target position using PID
     * and finishes when the position is reached.
     *
     * @param position The target climb position enum.
     * @return A command requiring this subsystem.
     */
    public Command climbToHeightCommand(ClimbPosition position) {
        return climbToHeightCommand(position.height).withName("ClimbTo" + position.name());
    }

    @Override
    public void periodic() {
        logOutputs();
    }

    private void logOutputs() {
        Logger.recordOutput("Climb/Current Pos (CANCoder)", getClimbPosition());
        Logger.recordOutput("Climb/Target Pos", getClimbTargetPosition());
        Logger.recordOutput("Climb/Motor One Out", climbMotorRight.get());
        Logger.recordOutput("Climb/Motor Two Out", climbMotorLeft.get());
        Logger.recordOutput("Climb/Forward Limit Switch One", forwardLimitRight.isPressed());
        Logger.recordOutput("Climb/Forward Limit Switch Two", forwardLimitLeft.isPressed());
        Logger.recordOutput("Climb/Reverse Limit Switch One", reverseLimitRight.isPressed());
        Logger.recordOutput("Climb/Reverse Limit Switch Two", reverseLimitLeft.isPressed());
        Logger.recordOutput("Climb/Current Pos (Motor 1)", cEncoderRight.getPosition());
        Logger.recordOutput("Climb/Current Pos (Motor 2)", cEncoderLeft.getPosition());
        Logger.recordOutput("Climb/RPMs 1", cEncoderRight.getVelocity());
        Logger.recordOutput("Climb/RPMs 2", cEncoderLeft.getVelocity());
    }
}