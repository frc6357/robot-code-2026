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

import com.ctre.phoenix6.controls.PositionVoltage;
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

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase
{

    SparkFlex climbMotor; // figure out if actually spark. depends on what motor is being used.
    SparkFlex climbMotor2;
    DigitalInput cLimitSwitch;
    PositionVoltage request;
    SparkLimitSwitch forwardLimit;
    SparkLimitSwitch reverseLimit;
    SparkLimitSwitch forwardLimit2;
    SparkLimitSwitch reverseLimit2;
    
    SparkClosedLoopController cPID;

    final double motorRatio = 1.0; //change once we know gear ratio
    final double climbFactor = 1.0; // change once fingure out what the conversion factor is from the encoder to the height

    double cTargetHieight = 0.0;
    double cCurrentHeight = 0.0;

    SparkFlexConfig climbConfig;
    SparkFlexConfig climbConfig2;

    public RelativeEncoder cEncoder; // would absolute work or would it need to be a relative? how many rotations will hex sharft make beofre it eches l1?
    public RelativeEncoder cEncoder2;

    SparkClosedLoopController cpid2;

    public boolean isRunning;

    public Climb()
    {
        climbMotor = new SparkFlex(kClimbMotor.ID, MotorType.kBrushless);
        climbMotor2 = new SparkFlex(kClimbMotorTwo.ID, MotorType.kBrushless);
        cEncoder = climbMotor.getEncoder(); 
        cEncoder2 = climbMotor2.getEncoder();
        cLimitSwitch = new DigitalInput(0);
        request = new PositionVoltage(0).withSlot(0);
        forwardLimit = climbMotor.getForwardLimitSwitch();
        reverseLimit = climbMotor.getReverseLimitSwitch();
        forwardLimit2 = climbMotor2.getForwardLimitSwitch();
        reverseLimit2 = climbMotor2.getReverseLimitSwitch();

        climbConfig = new SparkFlexConfig();
        climbConfig2 = new SparkFlexConfig();

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
        //climbConfig2.follow(41);


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

        //climbConfig = new TalonFXConfiguration();
        //climbConfig.Slot0.kP = kClimbP.get();
        //climbConfig.Slot0.kI = kClimbI;
        //climbConfig.Slot0.kD = kClimbD;
        //climbConfig.Slot0.kV = kClimbV;

        /*climbMotor.getConfigurator().apply(climbConfig);
        kClimbP.onChange((newkClimbP) -> {
            climbConfig.Slot0.kP = newkClimbP;
            climbMotor.getConfigurator().apply(climbConfig);
        });*/
    }


//runs the motor at a given speed
    public void runMotors(double motorSpeed)
    {
        climbMotor.set(motorSpeed);
        climbMotor2.set(motorSpeed);
    }

    // stops the motor by setting the speed to 0
    public void stopMotors()
    {
        climbMotor.stopMotor();
        climbMotor2.stopMotor();
    }

    // checks if the climb is at the target height
    public boolean climbIsAtHeight()
    {
        if (DriverStation.isTeleop())
        {
            return Math.abs(getClimbTargetPosition()- getClimbPosition()) < kClimbTolerance;
        }
        else
        {
           return Math.abs(getClimbTargetPosition() - getClimbPosition()) <  Rotations.of(0.1).in(Degrees);
        }
    }

    // etreives the saved target position for the climb
    public double getClimbTargetPosition()
    {
        return cTargetHieight;
    }

    // gets the current position of the climb
    public double getClimbPosition()
    {
        double pos = cEncoder.getPosition();
        return pos * climbFactor;
    }

    // chacks the limit switch to see if it is true or false
    public boolean isSwitchPressed()
    {
        return cLimitSwitch.get();
    }

    // sets a target height for the climb
    public void setClimbHeight(double targetHieight)
    {
        cTargetHieight = targetHieight;
        

        double motorRotations =  targetHieight * motorRatio; // / scale factor of encode to height
        cPID.setSetpoint(motorRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        cpid2.setSetpoint(motorRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    // gets the current position of the encoder
    public double getEncoderPos()
    {
        double pos = cEncoder.getPosition();
        return pos;//cEncoder.getPosition();
    }

    // hold command, may be necessary to ensure the robot doesnt get pulled back down by gravity
    public void hold()
    {
        if(isRunning == true)
        {
            cTargetHieight = getClimbPosition();
            isRunning = false;
        }
        setClimbHeight(cTargetHieight);
    }

    // checks if the climb arm is at its max height
    public boolean cAtMaxHeight()
    {
        if(getEncoderPos() == kCLimbMax)
        {
            return true;
        }

        else
        {
            return false;
        }
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Climb", this);
    }

    @Override
    public void initSendable(SendableBuilder builder)
    {
        super.initSendable(builder);
        builder.addDoubleProperty("Current Pos (CANCoder)", this::getClimbPosition, null);
        //builder.addDoubleProperty("Current Pos (Motor)", () -> climbMotor.getPosition(), null);
        builder.addDoubleProperty("Target Pos", this::getClimbTargetPosition, null);
        builder.addDoubleProperty("Motor One Out", climbMotor::get, null);   
        builder.addDoubleProperty("Motor Two Out", climbMotor2::get, null);  
        builder.addBooleanProperty("Forward Limit Switch One", forwardLimit::isPressed, null);
        builder.addBooleanProperty("Forward Limit Switch Two", forwardLimit2::isPressed, null);  
        builder.addBooleanProperty("Reverse Limit Switch One", reverseLimit::isPressed, null);
        builder.addBooleanProperty("Reverse Limit Switch Two", reverseLimit2::isPressed, null);
        builder.addDoubleProperty("Current Pos (Motor 1)", () -> cEncoder.getPosition(), null);
        builder.addDoubleProperty("Current Pos (Motor 2)", () -> cEncoder2.getPosition(), null);
        builder.addDoubleProperty("RPMs? 1", cEncoder::getVelocity, null);
        builder.addDoubleProperty("RPMs? 2", cEncoder2::getVelocity, null);
    }
}