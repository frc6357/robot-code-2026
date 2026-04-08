package frc.lib.tuning;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Konstants;

public class Tuning {
    private static Tuning instance = null;
    private static boolean active = false;

    private Tuning() {
        System.out.println("[Tuning] Initializing");
        forceLoadConstants();
        checkStalePreferences();
        System.out.println("[Tuning] Finished initialization");
    }

    public static void initialize() {
        if (instance != null) {
            throw new IllegalStateException("[Tuning] Tuning has already been initialized");
        }
        active = true;
        instance = new Tuning();
    }

    public static boolean isActive() {
        return active;
    }

    public static void forceLoadConstants() {
        loadClassRecursively(Konstants.class);
    }

    private static void loadClassRecursively(Class<?> clazz) {
        for (Class<?> inner : clazz.getDeclaredClasses()) {
            try {
                Class.forName(inner.getName());
                loadClassRecursively(inner); // handle nested inner classes
            } catch (ClassNotFoundException e) {
                DriverStation.reportWarning(
                    "[Tuning] Failed to load constants class: " + inner.getName(), false
                );
            }
        }
    }

    private static void checkStalePreferences() {
        TunableNumber.checkAllForStalePreferences();
    }
}
