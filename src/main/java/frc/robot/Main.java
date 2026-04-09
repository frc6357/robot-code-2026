// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
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
        boolean runReplay = false;

        try {
            File configFile = new File(Filesystem.getDeployDirectory(), "boot_options.json");
            ObjectMapper mapper = new ObjectMapper();
            JsonNode config = mapper.readTree(configFile);
            runReplay = config.get("RunReplay").asBoolean(false);
        } catch (IOException e) {
            System.err.println("[RobotMode] Failed to read boot_options.json for RunReplay, defaulting to CONTROLLED mode: " + e.getMessage());
            runReplay = false;
        }

        if (runReplay) {
            System.out.println("[RobotMode] Starting in REPLAY mode.");
        } else {
            System.out.println("[RobotMode] Starting in CONTROLLED mode.");
        }

        final Robot.RobotMode mode = runReplay ? Robot.RobotMode.REPLAY : Robot.RobotMode.CONTROLLED;
        RobotBase.startRobot(() -> new Robot(mode));
    }
}
