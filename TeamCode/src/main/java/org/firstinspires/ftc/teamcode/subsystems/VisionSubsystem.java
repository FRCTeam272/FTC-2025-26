package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.AprilTagEnums;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class VisionSubsystem {

    public enum VisionMode {
        APRILTAG,
        ARTIFACT
    }

    private VisionMode visionMode = VisionMode.APRILTAG;

    private ColorBlobLocatorProcessor artifactProcessor;
    private AprilTagProcessor tagProcessor;

    private Size cameraResolutionTag = new Size(640, 480);
    private Size cameraResolutionArt = new Size(320, 240);
    private Size camRes = cameraResolutionTag;

    public WebcamName webCam;
    public final MatchSettings matchSettings;
    public static MatchSettings.Motif motif = MatchSettings.Motif.UNKNOWN;
    VisionPortal visionPortal;

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     * <p>
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     * <p>
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     * <p>
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     * <p>
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

    public VisionSubsystem(HardwareMap hwMap, MatchSettings matchSettings) {
        this.matchSettings = matchSettings;
        webCam = hwMap.get(WebcamName.class, "Webcam 1");

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

//        artifactProcessor = new ColorBlobLocatorProcessor.Builder()
//                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
//                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
//                .setDrawContours(true)
//                .setBoxFitColor(0)
//                .setCircleFitColor(Color.rgb(255, 255, 0))
//                .setBlurSize(5)
//                .setDilateSize(15)
//                .setErodeSize(15)
//                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
//                .build();

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

//        visionPortal_Artifact = new VisionPortal().Builder()
//                .addProcessor(artifactProcessor)
//                .setCamera(webCam)
//                .setCameraResolution(new Size(320, 240))
//                .build();
    }

    public void teleopFSM() {

        switch (visionMode) {
            case APRILTAG:
                camRes = cameraResolutionTag;

                if (matchSettings.getMotif() != MatchSettings.Motif.UNKNOWN) {
                    scanMotifTagSequence();
                }
                break;
            case ARTIFACT:
                camRes = cameraResolutionArt;
                break;
        }

//        visionPortal = new VisionPortal.Builder()
//                .addProcessor(tagProcessor)
//                .setCamera(webCam)
//                .setCameraResolution(camRes)
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .enableLiveView(true)
//                .build();
    }

    public boolean isDetectingAGoalTag() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            if (tag.id == AprilTagEnums.BLUE_GOAL.getId()
                    || tag.id == AprilTagEnums.RED_GOAL.getId()) {
                return true;
            }
        }
        return false;
    }

    public boolean isDetectingAnObeliskTag() {
        if (!tagProcessor.getDetections().isEmpty()) {
            for (AprilTagDetection tag : tagProcessor.getDetections()) {
                if (tag.id == AprilTagEnums.OBELISK_TAG_21.getId()
                        || tag.id == AprilTagEnums.OBELISK_TAG_22.getId()
                        || tag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                    return true;
                }
            }
        }
        return false;
    }

//    public boolean isDetectingAnArtifact() {
//        if (!artifactProcessor.getBlobs().isEmpty()) {
//            return true;
//        }
//        return false;
//    }


    public void scanMotifTagSequence() {
        AprilTagDetection tag;
        for (AprilTagDetection detectedTag : tagProcessor.getDetections()) {
            if (detectedTag.id == AprilTagEnums.OBELISK_TAG_21.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_22.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                tag = detectedTag;
                MatchSettings.visionState = MatchSettings.VisionState.MOTIF_DETECTED;
                if (tag.id == AprilTagEnums.OBELISK_TAG_21.getId()) {
                    motif = MatchSettings.Motif.PPG;
                } else if (tag.id == AprilTagEnums.OBELISK_TAG_22.getId()) {
                    motif = MatchSettings.Motif.PGP;
                } else if (tag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                    motif = MatchSettings.Motif.PPG;
                }
                matchSettings.setMotif(motif);
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

//    public double getArtifactTurnPower() {
//        // Get the list of detected blobs
//        List<ColorBlobLocatorProcessor.Blob> blobs = artifactProcessor.getBlobs();
//
//        // Filter blobs by size and shape
//        ColorBlobLocatorProcessor.Util.filterByCriteria(
//                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
//                50, 20000, blobs);
//        ColorBlobLocatorProcessor.Util.filterByCriteria(
//                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
//                0.6, 1.0, blobs);
//
//        // Check if any blobs were found
//        if (!blobs.isEmpty()) {
//            // Select the largest blob as the target
//            ColorBlobLocatorProcessor.Blob targetBlob = blobs.get(0);
//            Circle circleFit = targetBlob.getCircle();
//
//            // Calculate the horizontal error (160 is the middle of the 320pixel resolution)
//            double error = 160 - circleFit.getX();
//
//            // Calculate the turn power based on the error
//            // Multply by proportional gain for turning. A higher value means the robot will turn more aggressively.
//            double turnPower = error * 0.01;
//            return turnPower;
//        }
//        return 0;
//    }

    public SparkFunOTOS.Pose2D getCurrentPose() {
        if (!tagProcessor.getDetections().isEmpty()) {
        AprilTagDetection tag = tagProcessor.getDetections().get(0);
        double poseX = tag.ftcPose.x;
        double poseY = tag.ftcPose.y;
        double poseH = Math.toDegrees(tag.ftcPose.bearing);
        SparkFunOTOS.Pose2D currentPose = new SparkFunOTOS.Pose2D(poseX, poseY, poseH);
        return currentPose;}
        else { return new SparkFunOTOS.Pose2D(100,0,0);}
    }

    public double getTagRange() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return tag.ftcPose.range;
        } else { return -1;}
    }

    public VisionMode getVisionMode() { return visionMode; }

    public void setVisionMode(VisionMode visionMode) { this.visionMode = visionMode; }

    /**============== AUTONOMOUS ACTIONS ==============**/

    public class AutoScanMotif implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (motif == MatchSettings.Motif.UNKNOWN) {
                scanMotifTagSequence();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action autoScanMotif() { return new AutoScanMotif(); }
}
