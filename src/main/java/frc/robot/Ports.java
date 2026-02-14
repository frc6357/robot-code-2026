package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftTrigger;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftY;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightTrigger;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightX;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightY;
import static edu.wpi.first.wpilibj.XboxController.Button.kA;
import static edu.wpi.first.wpilibj.XboxController.Button.kB;
import static edu.wpi.first.wpilibj.XboxController.Button.kBack;
import static edu.wpi.first.wpilibj.XboxController.Button.kLeftBumper;
import static edu.wpi.first.wpilibj.XboxController.Button.kLeftStick;
import static edu.wpi.first.wpilibj.XboxController.Button.kRightBumper;
import static edu.wpi.first.wpilibj.XboxController.Button.kRightStick;
import static edu.wpi.first.wpilibj.XboxController.Button.kStart;
import static edu.wpi.first.wpilibj.XboxController.Button.kX;
import static edu.wpi.first.wpilibj.XboxController.Button.kY;
import static edu.wpi.first.wpilibj.XboxController.Button.kX;
import static frc.lib.utils.SKTrigger.INPUT_TYPE.AXIS;
import static frc.lib.utils.SKTrigger.INPUT_TYPE.BUTTON;
import static frc.lib.utils.SKTrigger.INPUT_TYPE.POV;
import static frc.robot.Konstants.kCANivoreName;
import static frc.robot.Konstants.DriveConstants.kPigeonID;
import static frc.robot.Ports.OperatorPorts.kXbutton;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.GenericHID;
import frc.lib.utils.CANPort;
import frc.lib.utils.SKTrigger;
import frc.lib.utils.filters.FilteredAxis;
import frc.lib.utils.filters.FilteredXboxController;

// Unused Imports

//import static frc.robot.utils.SKTrigger.INPUT_TYPE.*;
//import static frc.robot.utils.SKTrigger.INPUT_TYPE.AXIS;

public class Ports
{
    public static class DriverPorts
    {
        // Driver Controller set to Xbox Controller
        public static final GenericHID kDriver = new FilteredXboxController(0).getHID();
        
        // Filtered axis (translation & rotation)
        public static final FilteredAxis kLeftStickY = new FilteredAxis(() -> kDriver.getRawAxis(kLeftY.value));
        public static final FilteredAxis kLeftStickX = new FilteredAxis(() -> kDriver.getRawAxis(kLeftX.value));
        public static final FilteredAxis kRightStickX = new FilteredAxis(() -> kDriver.getRawAxis(kRightX.value)); 
        
        // ABXY:
        public static final SKTrigger kDriverAButton = new SKTrigger(kDriver, kA.value, BUTTON);
        // Light game buttons for Simon Says (Driver)
        public static final SKTrigger kDriverBbutton = new SKTrigger(kDriver, kB.value, BUTTON);
        public static final SKTrigger kDriverXbutton = new SKTrigger(kDriver, kX.value, BUTTON);
        public static final SKTrigger kDriverYbutton = new SKTrigger(kDriver, kY.value, BUTTON);

        // D-pad:
        public static final SKTrigger kUpDpad = new SKTrigger(kDriver, 0, POV);
        public static final SKTrigger kRightDpad = new SKTrigger(kDriver, 90, POV);
        public static final SKTrigger kDownDpad = new SKTrigger(kDriver, 180, POV);
        public static final SKTrigger kLeftDpad = new SKTrigger(kDriver, 270, POV);

        // Bumpers:
        public static final SKTrigger kRBbutton = new SKTrigger(kDriver, kRightBumper.value, BUTTON);
        public static final SKTrigger kLBbutton = new SKTrigger(kDriver, kLeftBumper.value, BUTTON);

        // Triggers:
        public static final SKTrigger kLTrigger = new SKTrigger(kDriver, kLeftTrigger.value, AXIS);
        public static final SKTrigger kRTrigger = new SKTrigger(kDriver, kRightTrigger.value, AXIS);

        // Menu buttons:
        public static final SKTrigger kStartbutton  = new SKTrigger(kDriver, kStart.value, BUTTON);
        public static final SKTrigger kBackbutton = new SKTrigger(kDriver, kBack.value, BUTTON);
        
        // Stick buttons:
        public static final SKTrigger kLSbutton = new SKTrigger(kDriver, kLeftStick.value, BUTTON);
        public static final SKTrigger kRSbutton = new SKTrigger(kDriver, kRightStick.value, BUTTON);
        
    }
    /**
     * Defines the button, controller, and axis IDs needed to get input from an external
     * controller
     */
    
    public static class OperatorPorts
    {
        // Operator Controller set to Xbox Controller
        public static final GenericHID kOperator = new FilteredXboxController(1).getHID();


        /**
        * Example of how to use POV buttons (D-pad)
        * public static final SKTrigger kExamplePOV = new SKTrigger(kOperator, 270, POV);
        *
        * Example of AXIS action (R/L Trigger on the controller)
        * public static final SKTrigger kExampleAXIS = new SKTrigger(kOperator, kRightTrigger.value, AXIS);
        *
        * Example of rawAxis values (Joysticks on the controller)
        * public static final FilteredAxis kExampleRawAxis = new FilteredAxis(() -> kOperator.getRawAxis(kLeftY.value));
        */

        public static final FilteredAxis kLeftStickY = new FilteredAxis(() -> kOperator.getRawAxis(kLeftY.value)); 
        public static final FilteredAxis kLeftStickX = new FilteredAxis(() -> kOperator.getRawAxis(kLeftX.value)); 

        // Filtered axis (translation & rotation)
        public static final FilteredAxis kRightStickX = new FilteredAxis(() -> kOperator.getRawAxis(kRightX.value));
        
        // ABXY:
        public static final SKTrigger kAbutton = new SKTrigger(kOperator, kA.value, BUTTON);
        public static final SKTrigger kBbutton = new SKTrigger(kOperator, kB.value, BUTTON);
        public static final SKTrigger kXbutton = new SKTrigger(kOperator, kX.value, BUTTON);
        public static final SKTrigger kYbutton = new SKTrigger(kOperator, kY.value, BUTTON);
        
        // Bumpers:
        public static final SKTrigger kLBbutton = new SKTrigger(kOperator, kLeftBumper.value, BUTTON);
        public static final SKTrigger kRBbutton = new SKTrigger(kOperator, kRightBumper.value, BUTTON);

        // D-pad:
        public static final SKTrigger kUpDpad = new SKTrigger(kOperator, 0, POV);
        public static final SKTrigger kRightDpad = new SKTrigger(kOperator, 90, POV);
        public static final SKTrigger kDownDpad = new SKTrigger(kOperator, 180, POV);
        public static final SKTrigger kLeftDpad = new SKTrigger(kOperator, 270, POV);

        // Triggers:
        public static final SKTrigger kRTrigger = new SKTrigger(kOperator, kRightTrigger.value, AXIS);
        public static final SKTrigger kLTrigger = new SKTrigger(kOperator, kLeftTrigger.value, AXIS);

        // Menu buttons:
        public static final SKTrigger kStartbutton  = new SKTrigger(kOperator, kStart.value, BUTTON);
        public static final SKTrigger kBackbutton = new SKTrigger(kOperator, kBack.value, BUTTON);

        // Stick buttons:
        public static final SKTrigger kLSbutton = new SKTrigger(kOperator, kLeftStick.value, BUTTON);
        public static final SKTrigger kRSbutton = new SKTrigger(kOperator, kRightStick.value, BUTTON);
    }

    public static class LightsPorts{
        public static final CANBus canBus = CANBus.roboRIO();
        //assign an ID of 48 to the CANdle
        public static final CANPort kCANdle = new CANPort(48, canBus.getName()); // CAN ID for the CANdle controller
    }

    public static class EndEffectorPorts
    {
        private static final String busName = "";
        //assign an ID of 48 to the CANdle
        public static final CANPort kCANdle = new CANPort(48, busName);
    }

    public static class LauncherPorts {
        
        private static final String busName = "";
        public static final CANPort kFixedLauncherMotor = new CANPort(50, busName);
        public static final CANPort kFixedLauncherMotorFollower = new CANPort(51, busName);
        public static final CANPort kTurretMotor = new CANPort(55, busName);
        public static final CANPort kTurretEncoder = new CANPort(57, busName);
    }

    // public static class ExamplePorts
    // {
    //     //bus name is null
    //     private static final String busName = "";

    //     //assign a motor ID of 49 to the example motor
    //     public static final CANPort kExampleMotor = new CANPort(59, busName); 
    // }
}
