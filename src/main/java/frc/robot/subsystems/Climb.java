package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import static frc.robot.Ports.ClimbPorts.kClimbEncoder;
import static frc.robot.Ports.ClimbPorts.kClimbMotor;
import static frc.robot.Ports.OperatorPorts.climbUpButton;

import java.util.Optional;

import static frc.robot.Konstants.ClimbConstants.kClimbP;
import static frc.robot.Konstants.ClimbConstants.kClimbTolerance;
import static frc.robot.Konstants.ClimbConstants.kClimbI;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Konstants.ClimbConstants.kCLimbMax;
import static frc.robot.Konstants.ClimbConstants.kClimbD;
import static frc.robot.Konstants.ClimbConstants.kClimbV;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climb extends SubsystemBase
{

    SparkFlex climbMotor; // figure out if actually spark. depends on what motor is being used.
    DigitalInput cLimitSwitch;
    PositionVoltage request;
    SparkLimitSwitch forwardLimit;
    SparkLimitSwitch reverseLimit;
    
    SparkClosedLoopController cPID;

    final double motorRatio = 1.0; //change once we know gear ratio
    final double climbFactor = 1.0; // change once fingure out what the conversion factor is from the encoder to the height

    double cTargetHieight = 0.0;
    double cCurrentHeight = 0.0;

    SparkFlexConfig climbConfig;

    public RelativeEncoder cEncoder; // would absolute work or would it need to be a relative? how many rotations will hex sharft make beofre it eches l1?

    SparkClosedLoopController cpid;

    public boolean isRunning;

    public Climb()
    {
        climbMotor = new SparkFlex(kClimbMotor.ID, MotorType.kBrushless);
        cEncoder = climbMotor.getEncoder(); 
        cLimitSwitch = new DigitalInput(0);
        request = new PositionVoltage(0).withSlot(0);
        forwardLimit = climbMotor.getForwardLimitSwitch();
        reverseLimit = climbMotor.getReverseLimitSwitch();

        climbConfig = new SparkFlexConfig();

        climbConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kClimbP)
        .i(kClimbI)
        .d(kClimbD)
        .maxMotion.maxVelocity(kClimbV);

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
    public void runMotor(double motorSpeed)
    {
        climbMotor.set(motorSpeed);
    }

    // stops the motor by setting the speed to 0
    public void stopMotor()
    {
        climbMotor.stopMotor();
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
        cPID.setReference(motorRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
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
        builder.addDoubleProperty("Motor out", climbMotor::get, null);     
        builder.addBooleanProperty("Forward Limit Swotch", forwardLimit::isPressed, null);  
        builder.addBooleanProperty("Reverse Limit Switch", reverseLimit::isPressed, null);
    }
}