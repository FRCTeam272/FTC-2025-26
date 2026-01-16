package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

import com.acmerobotics.dashboard.FtcDashboard;

/*
 * This OpMode uses a camera to locate a colored artifact and drive towards it.
 * It uses a ColorBlobLocatorProcessor to find the artifact and a simple
 * proportional control loop to steer the robot.
 */
@Disabled
@Autonomous(name = "Auto Drive to Artifact", group = "Concept")
public class RobotAutoDriveToArtifact extends LinearOpMode {

    // Vision variables
    private VisionPortal portal;
    private ColorBlobLocatorProcessor colorLocator;

    // Drivetrain motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    // --- TUNING CONSTANTS ---
    // These constants control the robot's driving behavior.
    // You may need to adjust them for your specific robot and lighting conditions.

    // The desired horizontal center of the artifact in the camera frame (pixels)
    private static final int TARGET_X = 160; // For a 320x240 resolution

    // Proportional gain for turning. A higher value means the robot will turn more aggressively.
    private static final double TURN_GAIN = 0.01;

    // The base forward speed of the robot.
    private static final double FORWARD_SPEED = 1;

    // The robot will stop when the artifact's radius is larger than this value (pixels).
    // This indicates the robot is close enough.
    private static final int STOP_RADIUS = 80;
    // Method to stop all drivetrain motors
    private void stopRobot() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    // Method to set power to the drivetrain motors
    private void setDrivePower(double leftPower, double rightPower) {
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower); // Assuming left front and back get the same power
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower); // Assuming right front and back get the same power
    }

    //    private ColorBlobLocatorProcessor purpleLocator;
//    private ColorBlobLocatorProcessor greenLocator;
    @Override
    public void runOpMode() {
        // --- VISION INITIALIZATION ---
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // --- DRIVETRAIN INITIALIZATION ---
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Most robots need the motors on one side to be reversed to drive forward.
        // Adjust this based on your robot's configuration.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motors to brake when power is zero
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();

        // Wait for the driver to press START
        waitForStart();

        FtcDashboard.getInstance().startCameraStream(portal, 30);

        while (opModeIsActive()) {
            // Get the list of detected blobs
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            // Filter blobs by size and shape
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 20000, blobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.6, 1.0, blobs);

            // Check if any blobs were found
            if (!blobs.isEmpty()) {
                // Select the largest blob as the target
                ColorBlobLocatorProcessor.Blob targetBlob = blobs.get(0);
                Circle circleFit = targetBlob.getCircle();

                // Check if we are close enough to the target
                if (circleFit.getRadius() > STOP_RADIUS) {
                    // Stop the robot
                    stopRobot();
                    telemetry.addLine("Target Reached!");
                } else {
                    // --- DRIVING LOGIC ---
                    // Calculate the horizontal error
                    double error = TARGET_X - circleFit.getX();

                    // Calculate the turn power based on the error
                    double turnPower = error * TURN_GAIN;

                    // Calculate the power for each side of the drivetrain
                    double leftPower = FORWARD_SPEED - turnPower/2;
                    double rightPower = FORWARD_SPEED + turnPower/2;

                    // Set motor powers
                    setDrivePower(leftPower, rightPower);

                    // Provide telemetry for debugging
                    telemetry.addLine("Driving towards target...");
                    telemetry.addData("Target X", TARGET_X);
                    telemetry.addData("Current X", "%.1f", circleFit.getX());
                    telemetry.addData("Error", "%.1f", error);
                    telemetry.addData("Radius", "%.1f", circleFit.getRadius());
                    telemetry.addData("Power", "L:%.2f R:%.2f", leftPower, rightPower);
                }
            } else {
                // No blobs detected, so stop the robot
                stopRobot();
                telemetry.addLine("No target detected.");
            }
            telemetry.update();
        }
        //Ensure the robot is stopped when the OpMode ends
        stopRobot();
    }
}
