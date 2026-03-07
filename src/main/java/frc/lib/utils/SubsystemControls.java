package frc.lib.utils;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * This class holds the subsystem control values as imported from the subsystem control
 * JSON file. This was made in the 2022 season
 */
public class SubsystemControls
{

    private final boolean swerve;
    private final boolean vision;
    private final boolean climb;
    private final boolean turret;
    private final boolean launcher;
    private final boolean bangbanglauncher;
    private final boolean lights;
    private final boolean pickup;
    private final boolean indexer;
    private final boolean feeder;
    private final boolean fueldetection;

     /**  
     * @param swerve
     *            indicates if the swerve subsystem is present and should be enabled
     * @param lights
     *            indicates if the lights subsystem is present and should be enabled
     * @param pickup
     *            indicates if the pickup subsystem is present and should be enabled
     */

    public SubsystemControls(
        @JsonProperty(required = true, value = "swerve")      boolean swerve,
        @JsonProperty(required = true, value = "vision")      boolean vision,
        @JsonProperty(required = true, value = "climb")       boolean climb,

        @JsonProperty(required = true, value = "pickup")      boolean pickup,
        @JsonProperty(required = true, value = "turret")      boolean turret,
        @JsonProperty(required = true, value = "launcher")     boolean launcher,
        @JsonProperty(required = true, value = "bangbanglauncher") boolean bangbanglauncher,
        @JsonProperty(required = true, value = "lights")      boolean lights,
        @JsonProperty(required = true, value = "indexer")     boolean indexer,
        @JsonProperty(required = true, value = "feeder")      boolean feeder,
        @JsonProperty(required = true, value = "fueldetection") boolean fueldetection
    )

    {
        this.swerve = swerve;
        this.vision = vision;
        this.climb = climb;
        this.turret = turret;
        this.launcher = launcher;
        this.bangbanglauncher = bangbanglauncher;
        this.lights = lights;
        this.pickup = pickup;
        this.indexer = indexer;
        this.feeder = feeder;
        this.fueldetection = fueldetection;
    }


    /**
     * Returns true if the drive subsystem is indicated as present and should be enabled.
     * 
     * @return true if the drive subsystem is indicated as present and should be enabled; false
     *         otherwise
     */
    public boolean isSwervePresent()
    {
        return swerve;
    }
    public boolean isVisionPresent() {
        return vision;
    }
    public boolean isClimbPresent() {
        return climb;
    }
    public boolean isTurretPresent() {
        return turret;
    }
    public boolean isLauncherPresent() {
        return launcher;
    }
    public boolean isBangBangLauncherPresent() {
        return bangbanglauncher;
    }
    public boolean isLightsPresent() {
        return lights;
    }
    public boolean isPickupPresent() {
        return pickup;
    }
    public boolean isIndexerPresent() {
        return indexer;
    }
    public boolean isFeederPresent() {
        return feeder;
    }
    public boolean isFuelDetectionPresent() {
        return fueldetection;
    }
}
