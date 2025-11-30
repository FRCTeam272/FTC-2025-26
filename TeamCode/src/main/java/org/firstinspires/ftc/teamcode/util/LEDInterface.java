package org.firstinspires.ftc.teamcode.util;

/** Leds hardware interface */
public interface LEDInterface {
    /**
     * Sets the current LED mode.
     */
    public default void setMode(LedMode mode) {
    }

    /**
     * Possible LED modes based on robot state, IO implementations should select an appropriate
     * pattern.
     */
    public static enum LedMode {
        TEAM_COLORS, ALLIANCE_RED, ALLIANCE_BLUE, INTAKING_FRONT, INTAKING_REAR, INTAKING_THRU,
        MOTIF_DETECTED, LAUNCHER_ATSPEED, GREEN_NEXT, PURPLE_NEXT, ELEVATING, BLACK
    }
}
