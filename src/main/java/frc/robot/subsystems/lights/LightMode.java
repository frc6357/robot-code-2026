package frc.robot.subsystems.lights;

/**
 * Enum defining all available LED modes/effects.
 */
public enum LightMode {
    // Automatic — follows FMS game state + StateHandler macro state
    AUTO,

    // Basic modes
    OFF,
    SOLID_WHITE,
    SOLID_GREEN,
    SOLID_RED,
    SOLID_BLUE,
    SOLID_YELLOW,
    SOLID_ORANGE,

    // Strobe Color
    STROBE_WHITE,
    STROBE_GREEN,
    STROBE_RED,
    STROBE_BLUE,
    STROBE_YELLOW,
    STROBE_ORANGE,
    STROBE_PURPLE,
    STROBE_SKBLUE,

    // Dual Color
    DUAL_SOLID_WHITE_GREEN,
    DUAL_SOLID_WHITE_YELLOW,
    DUAL_STROBE_WHITE_GREEN,
    DUAL_STROBE_WHITE_YELLOW,
    
    // Team/Match modes
    RAINBOW,
    BREATHING_SKBLUE,
    SKBLUE_GRADIENT,
    ALLIANCE_GRADIENT,
    ALLIANCE_SOLID,
    ALLIANCE_FLASH_WHITE,
    
    // Classic effects
    FIRE,
    POLICE,
    SPARKLE,
    COLOR_CHASE,
    METEOR,
    THEATER_CHASE,
    LAVA_LAMP,
    OCEAN_WAVE,
    TWINKLE_STARS,
    CANDY_CANE,
    PLASMA,
    KNIGHT_RIDER,
    HEARTBEAT,
    LIGHTNING,
    CONFETTI,
    COMET_TRAIL,
    
    // Awesome effects
    AURORA_BOREALIS,
    GALAXY_SWIRL,
    NEON_PULSE,
    MATRIX_RAIN,
    FIREWORKS,
    BREATHING_RAINBOW,
    WAVE_COLLISION,
    DISCO_BALL,
    CYBERPUNK,
    SNAKE_GAME,
    RIPPLE,
    GRADIENT_BOUNCE,
    ELECTRIC_SPARKS,
    SUNSET,
    NORTHERN_LIGHTS,
    PACMAN,
    SOUND_WAVE,
    DNA_HELIX,
    PORTAL,
    HYPNOTIC_SPIRAL,
    PIXEL_RAIN,
    FIREFLIES,
    NYAN_CAT,
    RACING_STRIPES,
    DRIP,
    GODZILLA_CHARGING,
    
    // Interactive games
    TUG_OF_WAR,
    SIMON_SAYS
}
