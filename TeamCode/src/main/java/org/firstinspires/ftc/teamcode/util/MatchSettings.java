package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import java.util.HashMap;
import java.util.Objects;

/**
 * A persistent object that stores everything we know about the current match
 * state.
 *
 * @noinspection unchecked, we know what the blackboard is storing
 * from https://github.com/24358-ragnarok/Decode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/configuration/MatchSettings.java
 */
public class MatchSettings {
    private static final String MOTIF_KEY = "motif";
    private static final String ALLIANCE_COLOR_KEY = "allianceColor";
    private static final String STORED_POSE_KEY = "storedPose";
    private final HashMap<String, Object> blackboard;

    public MatchSettings(HashMap<String, Object> blackboard) {
        this.blackboard = blackboard;
    }

    /**
     * Returns an array of ArtifactColor representing the sequence for the given
     * Motif.
     * The order of colors corresponds to the Motif's letter order.
     * <p>
     * Example:
     * Motif.PPG -> [PURPLE, PURPLE, GREEN]
     * Motif.PGP -> [PURPLE, GREEN, PURPLE]
     * Motif.GPP -> [GREEN, PURPLE, PURPLE]
     *
     * @param motif The motif to convert
     * @return An array of ArtifactColor in motif order
     */
    public static ArtifactColor[] motifToArtifactColors(Motif motif) {
        if (motif == null)
            return null;
        String motifStr = motif.name();
        ArtifactColor[] colors = new ArtifactColor[3];
        for (int i = 0; i < 3; i++) {
            char c = motifStr.charAt(i);
            switch (c) {
                case 'P':
                    colors[i] = ArtifactColor.PURPLE;
                    break;
                case 'G':
                    colors[i] = ArtifactColor.GREEN;
                    break;
                default:
                    throw new IllegalArgumentException("Unknown motif character: " + c);
            }
        }
        return colors;
    }

    public AllianceColor getAllianceColor() {
        String color = (String) blackboard.get(ALLIANCE_COLOR_KEY);
        return Objects.equals(color, "red") ? AllianceColor.RED : AllianceColor.BLUE;
    }

    public void setAllianceColor(AllianceColor color) {
        if (color != null) {
            blackboard.put(ALLIANCE_COLOR_KEY, color.name().toLowerCase());
        }
    }

    public Motif getMotif() {
        String motif = (String) blackboard.get(MOTIF_KEY);
        if (motif != null) {
            return Motif.valueOf(motif.toUpperCase());
        } else {
            return Motif.UNKNOWN;
        }
    }

    public void setMotif(Motif motif) {
        if (motif != null) {
            blackboard.put(MOTIF_KEY, motif.name().toLowerCase());
        }
    }

    /**
     * Retrieves the stored robot pose from the previous OpMode.
     * in SparkFun Otos Pose format
     * @return The stored pose, or null if no pose was stored
     */
    public SparkFunOTOS.Pose2D getStoredPose() {
        double[] poseArray = (double[]) blackboard.get(STORED_POSE_KEY);
        if (poseArray != null && poseArray.length == 3) {
            return new SparkFunOTOS.Pose2D(poseArray[0], poseArray[1], Math.toDegrees(poseArray[2]));
        }
        return null;
    }

    /**
     * Stores the actual robot Roadrunner Auton end pose for use by subsequent OpModes.
     * This pose will be used as the starting position for the next OpMode.
     *
     * @param pose The actual robot pose to store
     */
    public void setStoredPose(Pose2d pose) {
        if (pose != null) {
            // Store pose as array: [x, y, heading]
            double[] poseArray = {pose.position.x, pose.position.y, pose.heading.toDouble()};
            blackboard.put(STORED_POSE_KEY, poseArray);
        }
    }

    public void clearStoredPose() {
        blackboard.remove(STORED_POSE_KEY);
    }

    public enum AllianceColor {
        RED,
        BLUE,
        UNKNOWN
    }

    public enum ArtifactColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public enum Motif {
        PPG,
        PGP,
        GPP,
        UNKNOWN
    }

}
