// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Scanner;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
    private Main() {}

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        int mode = 0;
        Scanner scanner = new Scanner(System.in);
        while(mode != 1 && mode != 2) {
            System.out.print("Select robot mode (1. Controlled, 2. Replay): ");
        try{
            mode = scanner.nextInt();
        }
        catch(Exception e){
            mode = 0;
            System.out.println("Invalid input. Please enter 1 or 2.");
            scanner.nextLine(); // Clear the invalid input
        }
        }

        final int selectedMode = mode;
        if(selectedMode == 1) {
            System.out.println("Starting in CONTROLLED mode.");
            scanner.close();  // Can close System.in since AdvantageKit won't need it in CONTROLLED mode
        }
        else {
            System.out.println("Starting in REPLAY mode.");
        }
        RobotBase.startRobot(() -> new Robot(selectedMode == 1 ? Robot.RobotMode.CONTROLLED : Robot.RobotMode.REPLAY));
    }
}
