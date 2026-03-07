package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Konstants.ClimbConstants.kCLimbMax;
import static frc.robot.Konstants.ClimbConstants.kClimbD;
import static frc.robot.Konstants.ClimbConstants.kClimbI;
import static frc.robot.Konstants.ClimbConstants.kClimbP;
import static frc.robot.Konstants.ClimbConstants.kClimbTolerance;
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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SK26Climb extends SubsystemBase
{
    private final SparkFlex climbMotor;
    private final SparkFlex climbMotor2;
    private final DigitalInput cLimitSwitch;
    private final SparkLimitSwitch forwardLimit;
    private final SparkLimitSwitch reverseLimit;
    private final SparkLimitSwitch forwardLimit2;
    private final SparkLimitSwitch reverseLimit2;
    
    private final SparkClosedLoopController cPID;
    private final SparkClosedLoopController cpid2;

    private final RelativeEncoder cEncoder;
    private final RelativeEncoder cEncoder2;

    private final double motorRatio = 1.0;   // TODO: change once we know gear ratio
    private final double climbFactor = 1.0;   // TODO: change once we know encoder-to-height conversion

    private double cTargetHeight = 0.0;
    private boolean isRunning = false;

    public SK26Climb()
    {
        climbMotor = new SparkFlex(kClimbMotor.ID, MotorType.kBrushless);
        climbMotor2 = new SparkFlex(kClimbMotorTwo.ID, MotorType.kBrushless);
        cEncoder = climbMotor.getEncoder(); 
        cEncoder2 = climbMotor2.getEncoder();
        cLimitSwitch = new DigitalInput(0);
        forwardLimit = climbMotor.getForwardLimitSwitch();
        reverseLimit = climbMotor.getReverseLimitSwitch();
        forwardLimit2 = climbMotor2.getForwardLimitSwitch();
        reverseLimit2 = climbMotor2.getReverseLimitSwitch();

        SparkFlexConfig climbConfig = new SparkFlexConfig();
        SparkFlexConfig climbConfig2 = new SparkFlexConfig();

        climbConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kClimbP)
            .i(kClimbI)
            .d(kClimbD);

        climbConfig2.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(kClimbP)
            .i(kClimbI)
            .d(kClimbD);

        climbConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

        climbConfig2.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

        climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        climbMotor2.configure(climbConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        cEncoder.setPosition(0);
        cEncoder2.setPosition(0);
        cPID = climbMotor.getClosedLoopController();
        cpid2 = climbMotor2.getClosedLoopController();
    }

    /** Runs both climb motors at the given speed. */
    public void runMotors(double motorSpeed)
    {
        climbMotor.set(motorSpeed);
        climbMotor2.set(motorSpeed);
    }

    /** Stops both climb motors. */
    public void stopMotors()
    {
        climbMotor.stopMotor();
        climbMotor2.stopMotor();
    }

    /** Checks if the climb is within tolerance of the target height. */
    public boolean climbIsAtHeight()
    {
        double tolerance = DriverStation.isTeleop() 
            ? kClimbTolerance 
            : Rotations.of(0.1).in(Degrees);
        return Math.abs(getClimbTargetPosition() - getClimbPosition()) < tolerance;
    }

    public double getClimbTargetPosition()
    {
        return cTargetHeight;
    }

    public double getClimbPosition()
    {
        return cEncoder.getPosition() * climbFactor;
    }

    public boolean isSwitchPressed()
    {
        return cLimitSwitch.get();
    }

    /** Sets the target height for the climb and commands both PID controllers. */
    public void setClimbHeight(double targetHeight)
    {
        cTargetHeight = targetHeight;
        double motorRotations = targetHeight * motorRatio;
        cPID.setSetpoint(motorRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        cpid2.setSetpoint(motorRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    /** Holds the current position to resist gravity. */
    public void hold()
    {
        if (isRunning)
        {
            cTargetHeight = getClimbPosition();
            isRunning = false;
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
        isRunning = running;
    }

    @Override
    public void periodic() {
        logOutputs();
    }

    private void logOutputs() {
        Logger.recordOutput("Climb/Current Pos (CANCoder)", getClimbPosition());
        Logger.recordOutput("Climb/Target Pos", getClimbTargetPosition());
        Logger.recordOutput("Climb/Motor One Out", climbMotor.get());
        Logger.recordOutput("Climb/Motor Two Out", climbMotor2.get());
        Logger.recordOutput("Climb/Forward Limit Switch One", forwardLimit.isPressed());
        Logger.recordOutput("Climb/Forward Limit Switch Two", forwardLimit2.isPressed());
        Logger.recordOutput("Climb/Reverse Limit Switch One", reverseLimit.isPressed());
        Logger.recordOutput("Climb/Reverse Limit Switch Two", reverseLimit2.isPressed());
        Logger.recordOutput("Climb/Current Pos (Motor 1)", cEncoder.getPosition());
        Logger.recordOutput("Climb/Current Pos (Motor 2)", cEncoder2.getPosition());
        Logger.recordOutput("Climb/RPMs 1", cEncoder.getVelocity());
        Logger.recordOutput("Climb/RPMs 2", cEncoder2.getVelocity());
    }
}