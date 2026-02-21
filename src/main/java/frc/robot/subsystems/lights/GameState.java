package frc.robot.subsystems.lights;

/**
 * Enum defining the robot's game state for automatic light changes.
 */
public enum GameState {
    PRE_MATCH_NO_FMS,
    PRE_MATCH_FMS,
    AUTO,
    AUTO_TO_TELEOP,
    TELEOP,
    ENDGAME_FLASH,
    ENDGAME_SOLID,
    POST_MATCH
}
