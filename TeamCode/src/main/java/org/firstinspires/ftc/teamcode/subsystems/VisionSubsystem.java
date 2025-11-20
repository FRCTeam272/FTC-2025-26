package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.AprilTagEnums;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class VisionSubsystem extends SubsystemBase {

    public final MatchSettings matchSettings;
    public WebcamName webCam;
    public static String motifTagSequence = "NONE";
    VisionPortal visionPortal;

    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .build();






    public VisionSubsystem(HardwareMap hwMap, MatchSettings matchSettings) {
        this.matchSettings = matchSettings;
        webCam = hwMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webCam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .build();


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

//    public void scanObeliskTagSequence() {
//        AprilTagDetection tag;
//        for (AprilTagDetection detectedTag : tagProcessor.getDetections()) {
//            if (detectedTag.id == AprilTagEnums.OBELISK_TAG_21.getId()
//                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_22.getId()
//                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
//                tag = detectedTag;
//                if (tag.id == AprilTagEnums.OBELISK_TAG_21.getId()) {
//                    motifTagSequence = AprilTagEnums.OBELISK_TAG_21.getDescription();
//                } else if (tag.id == AprilTagEnums.OBELISK_TAG_22.getId()) {
//                    motifTagSequence = AprilTagEnums.OBELISK_TAG_22.getDescription();
//                } else if (tag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
//                    motifTagSequence = AprilTagEnums.OBELISK_TAG_23.getDescription();
//                }
//                break;
//            }
//        }
//    }

    /**
     * Detects a Motif AprilTag and saves it to Blackboard for retrieval
     */
    public void scanObeliskTagSequence() {
        AprilTagDetection tag;
        MatchSettings.Motif detectedMotif = MatchSettings.Motif.UNKNOWN;
        for (AprilTagDetection detectedTag : tagProcessor.getDetections()) {
            if (detectedTag.id == AprilTagEnums.OBELISK_TAG_21.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_22.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                tag = detectedTag;
                if (tag.id == AprilTagEnums.OBELISK_TAG_21.getId()) {
                    motifTagSequence = AprilTagEnums.OBELISK_TAG_21.getDescription();
                    detectedMotif = MatchSettings.Motif.GPP;
                } else if (tag.id == AprilTagEnums.OBELISK_TAG_22.getId()) {
                    motifTagSequence = AprilTagEnums.OBELISK_TAG_22.getDescription();
                    detectedMotif = MatchSettings.Motif.PGP;
                } else if (tag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                    motifTagSequence = AprilTagEnums.OBELISK_TAG_23.getDescription();
                    detectedMotif = MatchSettings.Motif.PPG;
                }
               break;
            }
        }
        matchSettings.setMotif(detectedMotif);
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

    public double getTagHorizontalDistance() {
        if (!tagProcessor.getDetections().isEmpty()) {
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return tag.ftcPose.y;
        }
        return -1;
    }

    public String getSequence() {
        return motifTagSequence;
    }
}
