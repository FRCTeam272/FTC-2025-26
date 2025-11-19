package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

public class DriveSubsystem {

    // CREATE DEVICES ==========================\\

    SparkFunOTOS myOtos; //Declare Odometry Computer

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx rightFront;

    // CREATE VARIABLES ==========================\\

    double fastpower;
    double allianceSteering;
    double correctedYaw;

    // CREATE MATCH SETTINGS / ALLIANCE ==============\\
    MatchSettings.AllianceColor alliance;

    public final MatchSettings matchSettings;


    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, MatchSettings matchSettings) {
        this.matchSettings = matchSettings;

        alliance = matchSettings.getAllianceColor();

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // All the configuration for the OTOS is done in this helper method, check it out!
        configureOtos();

        // After configuring and resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        // SET THIS TO ALLIANCE DEPENDENT HEADING or GET FROM AUTON

        SparkFunOTOS.Pose2D storedPose = matchSettings.getStoredPose(); //get from Auton

        SparkFunOTOS.Pose2D bluePosition = new SparkFunOTOS.Pose2D(0, -24, 270); // default blue for practice
        SparkFunOTOS.Pose2D redPosition = new SparkFunOTOS.Pose2D(0, 24, 90); // default red for practice

        if (storedPose != null) {
            myOtos.setPosition(storedPose);
        } else if (alliance == MatchSettings.AllianceColor.RED) {
            myOtos.setPosition(redPosition);
        } else {
            myOtos.setPosition(bluePosition);
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        double fastpower = 2;
//        double allianceSteering = 1;
//        double correctedYaw = 0;

    }

    public void FieldCentricAllianceBased(Gamepad gamepad1, Telemetry telemetry) {

        // Speed Controls

        if (gamepad1.left_bumper) { //Turbo/Max speed!
            fastpower = 1;
        } else if (gamepad1.right_bumper) { // Turtle speed - to nudge into position for endgame
            fastpower = 6;
        } else {
            fastpower = 2; //Regular speed, initially 50%, but can bump up as needed
        }

        // Apply speed control to gamepad stick input as a denominator, remember y stick is inverted
        // Change depending on where driver stands for Alliance

        if (alliance == MatchSettings.AllianceColor.BLUE) {
            allianceSteering = -1;
        } else {
            allianceSteering = 1;
        }
        double forward = (-gamepad1.left_stick_y * allianceSteering) / fastpower;
        double strafe = (gamepad1.left_stick_x * allianceSteering) / fastpower;

        double rotate = gamepad1.right_stick_x / fastpower;

        SparkFunOTOS.Pose2D pos = myOtos.getPosition();

        //Corrected Alliance Dependent Yaw Position if needed, retains field position, but resets Angle if
        //it drifts from where it should be. Can also be reset correctly from vision, hopefully implement next!

        if (alliance == MatchSettings.AllianceColor.BLUE) {
            correctedYaw = 270;
        } else {
            correctedYaw = 90;
        }

        SparkFunOTOS.Pose2D resetYawPos = new SparkFunOTOS.Pose2D(pos.x, pos.y, correctedYaw); // Baseline 0 degree

        // Reset the yaw if the user requests it
        if (gamepad1.y) {
            myOtos.setPosition(resetYawPos);
        }

        double heading = Math.toRadians(pos.h);
        double headingDegrees = pos.h; // for telemetry

        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        leftFront.setPower(newWheelSpeeds[0]);
        rightFront.setPower(newWheelSpeeds[1]);
        leftBack.setPower(newWheelSpeeds[2]);
        rightBack.setPower(newWheelSpeeds[3]);

        telemetry.addLine("Odometry Data");
        telemetry.addData("Robot Xpos: ", pos.x);
        telemetry.addData("Robot Ypos: ", pos.y);
        telemetry.addData("Robot Heading: ", headingDegrees);
        telemetry.addData("Alliance: ", alliance);
        telemetry.addLine("Press Y to reset tracking");
        telemetry.addLine("Hold LeftBumper for TURBO speed");
        telemetry.addLine("Hold RightBumper for TURTLE speed");
        telemetry.update();

    }

    /**
     * Otos Configuration Code
     */
    private void configureOtos() {

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AngleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-1.75, 0, 180);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(0.9798387096774194);
        myOtos.setAngularScalar(0.9986);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();




    }
}
