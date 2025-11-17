package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
public class AutoAimDrive extends OpMode {
    MecanumDrive drive;
    Pose2d beginPose;

    // 48 for use testing on an 8x8 field. Change to 72 for use on 12x12 ftc field
    private static final double GOAL_X = -48.0; // inches
    private static final double GOAL_Y = -48.0; // inches

/** Helper to wrap angles to [-PI, PI] */
private static double angleWrap(double angle) {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle; }

/** Returns the absolute heading (radians) to the red goal corner */
public static double headingToBlueGoal(Pose2d pose) {
    double dx = GOAL_X - pose.position.x;
    double dy = GOAL_Y - pose.position.y;
    return Math.atan2(dy, dx);
}
 double fastpower;



@Override
    public void init() {

    Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(270));
    MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

    double fastpower = 1;

    }

    @Override
    public void loop() {

    Pose2d pose = drive.localizer.getPose();

        double lx = -gamepad1.left_stick_y / fastpower; // forward/back
        double ly = gamepad1.left_stick_x / fastpower; // strafe
        double rotCmd = gamepad1.right_stick_x / fastpower; // normal rotation

        if (gamepad1.right_trigger > 0.2) {
            // Compute angle to goal
            double targetHeading = headingToBlueGoal(pose);
            double currentHeading = pose.heading.toDouble();

            // Replace robot heading command directly with the needed rotation
            // Determine shortest rotation direction
            double headingError = angleWrap(targetHeading - currentHeading);

            // Turn in that direction at a fixed rate (scaled by error)
            rotCmd = Math.copySign(Math.min(Math.abs(headingError) / Math.PI, 1.0), headingError);
        }

        if (gamepad1.left_bumper) {
            fastpower = 3;
        } else {
            fastpower = 1;
        }

        if (gamepad1.y) { //reset Yaw
            Pose2d poseWNewHeading = new Pose2d(pose.position, Math.toRadians(270));
            drive.localizer.setPose(poseWNewHeading);
        }

// Drive field-centric using Localizer's heading

        drive.driveFieldCentric(lx, ly,rotCmd, 1,telemetry);


    }
}
