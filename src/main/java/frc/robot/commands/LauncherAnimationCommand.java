package frc.robot.commands;

import static frc.robot.Konstants.LightsConstants.kColorRed;
import static frc.robot.Konstants.LightsConstants.kEndLauncherLightsIndex;
import static frc.robot.Konstants.LightsConstants.kStartLauncherLightsIndex;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj2.command.Command;

public class LauncherAnimationCommand extends Command{

    CANdle candle;
    double frameRate;
    ColorFlowAnimation animation;

    public LauncherAnimationCommand(CANdle candle, double frameRate) {
        this.candle = candle;
        this.frameRate = frameRate;
        animation = new ColorFlowAnimation(kStartLauncherLightsIndex, kEndLauncherLightsIndex);
        animation.Direction = AnimationDirectionValue.Forward;
        animation.FrameRate = this.frameRate;
        animation.Color = new RGBWColor(kColorRed[0], kColorRed[1], kColorRed[2], 0);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        candle.setControl(animation);
    }

    @Override
    public void end(boolean interrupted) {

    }

     @Override
     public boolean isFinished() {
         return false;
     }
    
}
