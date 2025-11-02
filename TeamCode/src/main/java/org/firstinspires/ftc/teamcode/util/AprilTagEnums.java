package org.firstinspires.ftc.teamcode.util;

public enum AprilTagEnums {
    RED_GOAL(24, "RED"),
    BLUE_GOAL(20, "BLUE"),
    OBELISK_TAG_21(21, "GPP"),
    OBELISK_TAG_22(22, "PGP"),
    OBELISK_TAG_23(23, "PPG");

    private final int id;
    private final String description;

    AprilTagEnums(int id, String description) {
        this.id = id;
        this.description = description;
    }

    public int getId() {
        return id;
    }

    public String getDescription() {
        return description;
    }

    public static AprilTagEnums fromId(int id) {
        for (AprilTagEnums tag : values()) {
            if (tag.id == id) {
                return tag;
            }
        }
        return null;
    }
}
