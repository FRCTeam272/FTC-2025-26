package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.AprilTagEnums;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionSubsystem {
    public WebcamName webCam;
    public static String motifTagSequence = "NONE";
    VisionPortal visionPortal;

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 8.5, 10.5, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setCameraPose(cameraPosition, cameraOrientation)
            .build();

    public VisionSubsystem(HardwareMap hwMap) {
        webCam = hwMap.get(WebcamName.class, "Webcam 1");

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        tagProcessor.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webCam)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .build();
    }

    public boolean isDetectingAGoalTag() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            if (tag.id == AprilTagEnums.BLUE_GOAL.getId()
                    || tag.id == AprilTagEnums.RED_GOAL.getId()) {;
                return true;
            }
        }
        return false;
    }

    public boolean isDetectingAnObeliskTag() {
        if (!tagProcessor.getDetections().isEmpty()) {
            for (AprilTagDetection tag: tagProcessor.getDetections()) {
                if (tag.id == AprilTagEnums.OBELISK_TAG_21.getId()
                        || tag.id == AprilTagEnums.OBELISK_TAG_22.getId()
                        || tag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                    return true;
                }
            }
        }
        return false;
    }


    public void scanMotifTagSequence() {
        AprilTagDetection tag;
        for (AprilTagDetection detectedTag : tagProcessor.getDetections()) {
            if (detectedTag.id == AprilTagEnums.OBELISK_TAG_21.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_22.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                tag = detectedTag;
                MatchSettings.visionState = MatchSettings.VisionState.MOTIF_ACQUIRED;
                if (tag.id == AprilTagEnums.OBELISK_TAG_21.getId()) {
                    motifTagSequence = AprilTagEnums.OBELISK_TAG_21.getDescription();
                } else if (tag.id == AprilTagEnums.OBELISK_TAG_22.getId()) {
                    motifTagSequence = AprilTagEnums.OBELISK_TAG_22.getDescription();
                } else if (tag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                    motifTagSequence = AprilTagEnums.OBELISK_TAG_23.getDescription();
                }
                break;
            }
        }
    }

    /*
     * Bearing is the angle to the tag relative to the camera's forward direction.
     * Returns Bearing in Degrees
     */
    public double getTagBearing() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return tag.ftcPose.bearing;
        }
        return 0;
    }

    public SparkFunOTOS.Pose2D getCurrentPose() {
        AprilTagDetection tag = tagProcessor.getDetections().get(0);
        double poseX = tag.ftcPose.x;
        double poseY = tag.ftcPose.y;
        double poseH = Math.toDegrees(tag.ftcPose.bearing);
        SparkFunOTOS.Pose2D currentPose = new SparkFunOTOS.Pose2D(poseX, poseY, poseH);
        return currentPose;
    }

    public double getTagHorizontalDistance() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return tag.ftcPose.range;
        }
        return -1;
    }

    public String getSequence() {
        return motifTagSequence;
    }
}
