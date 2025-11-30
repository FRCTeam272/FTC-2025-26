/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

import java.util.List;

public class MotifVisionSubsystem {

    public final MatchSettings matchSettings;
    public WebcamName webCam;
    public static String motifTagSequence = "NONE";

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

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;                         // Used to manage the video source.

    public MotifVisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry, MatchSettings matchSettings) {
        this.matchSettings = matchSettings;
        webCam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Initialize the Apriltag Detection process (vision portal)
        initAprilTag();

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.update();
    }

    public void scanObeliskTagSequence() {

        AprilTagDetection desiredTag;

        MatchSettings.Motif detectedMotif;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detectedTag : currentDetections) {
            if (detectedTag.id == AprilTagEnums.OBELISK_TAG_21.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_22.getId()
                    || detectedTag.id == AprilTagEnums.OBELISK_TAG_23.getId()) {
                // Yes, we want to use this tag.
                desiredTag = detectedTag;

                if (desiredTag.id == AprilTagEnums.OBELISK_TAG_21.getId()) {
                    motifTagSequence = AprilTagEnums.OBELISK_TAG_21.getDescription();
                    detectedMotif = MatchSettings.Motif.GPP;
                } else if (desiredTag.id == AprilTagEnums.OBELISK_TAG_22.getId()) {
                    motifTagSequence = AprilTagEnums.OBELISK_TAG_22.getDescription();
                    detectedMotif = MatchSettings.Motif.PGP;
                } else {
                    motifTagSequence = AprilTagEnums.OBELISK_TAG_23.getDescription();
                    detectedMotif = MatchSettings.Motif.PPG;
                }
                matchSettings.setMotif(detectedMotif);
            }
        }
    }

    public SparkFunOTOS.Pose2D scanGoalTagLocalization() { //call by drivesubsystem, not in the main teleop
        double Xpos;
        double Ypos;
        double Heading;
        SparkFunOTOS.Pose2D pose = new SparkFunOTOS.Pose2D(100,0,0);

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        //telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Only use tags that don't have Obelisk in them
                if (!detection.metadata.name.contains("Obelisk")) {
                    // If not an Obelisk Tag, get field location and send to
                    Xpos = detection.robotPose.getPosition().x;
                    Ypos = detection.robotPose.getPosition().y;
                    Heading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    pose.x = Xpos;
                    pose.y = Ypos;
                    pose.h = Heading;

//                    //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                            detection.robotPose.getPosition().x,
//                            detection.robotPose.getPosition().y,
//                            detection.robotPose.getPosition().z));
//                    //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            }
        }
        // end for() loop
        return pose;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor by using a builder.
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setCameraPose(cameraPosition, cameraOrientation)

                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(webCam)
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

    }
}
