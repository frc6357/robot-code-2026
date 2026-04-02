package frc.lib.utils;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Centralized utility for reading boot options from boot_options.json.
 * Options are read once at startup and cached for fast access throughout the code.
 * 
 * <p>Available options:
 * <ul>
 *   <li>{@code RunReplay} - Whether to run in replay mode (read in Main.java)</li>
 *   <li>{@code TuningEnabled} - Whether to publish TunableNumbers to SmartDashboard</li>
 *   <li>{@code ReducedLogging} - Whether to reduce logging frequency for non-critical telemetry</li>
 * </ul>
 */
public final class BootOptions {
    private static boolean initialized = false;
    
    private static boolean reducedLogging = true; // Default to reduced logging for safety
    private static boolean tuningEnabled = false;
    private static boolean runReplay = false;

    private BootOptions() {
        // Utility class - no instantiation
    }

    /**
     * Initialize boot options by reading from boot_options.json.
     * This should be called early in robot startup (e.g., in Main or Robot constructor).
     * Safe to call multiple times - will only read the file once.
     */
    public static void initialize() {
        if (initialized) {
            return;
        }
        
        try {
            File configFile = new File(Filesystem.getDeployDirectory(), "boot_options.json");
            ObjectMapper mapper = new ObjectMapper();
            JsonNode config = mapper.readTree(configFile);
            
            reducedLogging = config.has("ReducedLogging") 
                ? config.get("ReducedLogging").asBoolean(true) 
                : true;
            tuningEnabled = config.has("TuningEnabled")
                ? config.get("TuningEnabled").asBoolean(false)
                : false;
            runReplay = config.has("RunReplay")
                ? config.get("RunReplay").asBoolean(false)
                : false;
                
            System.out.println("[BootOptions] Loaded: ReducedLogging=" + reducedLogging 
                + ", TuningEnabled=" + tuningEnabled 
                + ", RunReplay=" + runReplay);
        } catch (IOException e) {
            System.err.println("[BootOptions] Failed to read boot_options.json, using defaults: " + e.getMessage());
            reducedLogging = true;
            tuningEnabled = false;
            runReplay = false;
        }
        
        initialized = true;
    }

    /**
     * @return true if reduced logging is enabled (less frequent telemetry updates)
     */
    public static boolean isReducedLogging() {
        if (!initialized) {
            initialize();
        }
        return reducedLogging;
    }

    /**
     * @return true if tuning mode is enabled (TunableNumbers published to SmartDashboard)
     */
    public static boolean isTuningEnabled() {
        if (!initialized) {
            initialize();
        }
        return tuningEnabled;
    }

    /**
     * @return true if replay mode is enabled
     */
    public static boolean isRunReplay() {
        if (!initialized) {
            initialize();
        }
        return runReplay;
    }

    /**
     * Returns the logging interval divisor based on reduced logging setting.
     * Use with Logger.runEveryN() for conditional frequency reduction.
     * 
     * @param normalInterval The interval when reduced logging is OFF (e.g., 1 for every cycle)
     * @param reducedInterval The interval when reduced logging is ON (e.g., 4 for every 4th cycle)
     * @return The appropriate interval based on the ReducedLogging setting
     */
    public static int getLoggingInterval(int normalInterval, int reducedInterval) {
        return reducedLogging ? reducedInterval : normalInterval;
    }
}
