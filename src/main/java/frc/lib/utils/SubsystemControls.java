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
    private final boolean turret;
    private final boolean launcher;

     /**  
     * @param swerve
     *            indicates if the swerve subsystem is present and should be enabled
     * @param lights
     *            indicates if the lights subsystem is present and should be enabled
     */
    public SubsystemControls(
        @JsonProperty(required = true, value = "swerve")      boolean swerve,
        @JsonProperty(required = true, value = "vision")      boolean vision,
        @JsonProperty(required = true, value = "turret")      boolean turret,
        @JsonProperty(required = true, value = "launcher")     boolean launcher
    )
    {
        this.swerve = swerve;
        this.vision = vision;
        this.turret = turret;
        this.launcher = launcher;
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
    public boolean isTurretPresent() {
        return turret;
    }
    public boolean isLauncherPresent() {
        return launcher;
    }
}
