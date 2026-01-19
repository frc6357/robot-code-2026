package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.AbsoluteEncoder;

import static frc.robot.Ports.ClimbPorts.kClimbMotor;
import static frc.robot.Konstants.ClimbConstants.kClimbP;
import static frc.robot.Konstants.ClimbConstants.kClimbTolerance;
import static frc.robot.Konstants.ClimbConstants.kClimbI;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Konstants.ClimbConstants.kCLimbMax;
import static frc.robot.Konstants.ClimbConstants.kClimbD;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climb extends SubsystemBase
{

    SparkFlex climbMotor; // figure out if actually spark. depends on what motor is being used.
    DigitalInput cLimitSwitch;

    final double motorRatio = 0.0; //change once we know gear ratio
    final double climbFactor = 0.0; // change once fingure out what the conversion factor is from the encoder to the height

    double cTargetHieight = 0.0;
    double cCurrentHeight = 0.0;

    SparkFlexConfig climbConfig;

    public AbsoluteEncoder cEncoder; // would absolute work or would it need to be a relative? how many rotations will hex sharft make beofre it eches l1?

    SparkClosedLoopController cpid;

    public boolean isRunning;

    public Climb()
    {
        climbMotor = new SparkFlex(kClimbMotor.ID, MotorType.kBrushless);
        cEncoder = climbMotor.getAbsoluteEncoder();
        cLimitSwitch = new DigitalInput(0);

        climbConfig = new SparkFlexConfig();

        climbConfig.smartCurrentLimit(80) // need to test?
            .idleMode(IdleMode.kBrake)
            .inverted(false); // determine actual direction for motor spin
        climbConfig.closedLoop.pid(kClimbP, kClimbI, kClimbD);
    }


    //runs the motor at a given speed
    public void runMotor(double motorSpeed)
    {
        climbMotor.set(motorSpeed);
    }

    // stops the motor by setting the speed to 0
    public void stopMotor()
    {
        climbMotor.set(0);
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
        return cEncoder.getPosition() * climbFactor;
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
        cpid.setReference(motorRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    // gets the current position of the encoder
    public double getEncoderPos()
    {
        return cEncoder.getPosition();
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
        if(getEncoderPos() == kCLimbMax && isSwitchPressed())
        {
            return true;
        }

        else
        {
            return false;
        }
        
    }

    public void teleopPeriodic()
    {
        SmartDashboard.putBoolean("Limit Switch", isSwitchPressed());
    }
}