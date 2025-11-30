package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

//look at https://github.com/FTC-9073-Knightrix/2025-2026-DECODE/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/mechanisms/TeleOpMecanumDrive.java
//and https://github.com/FTC-9073-Knightrix/2025-2026-DECODE/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/main/TeleOpMethods.java

public class AimbotDriveSubsystem {

    public enum DriveMode {
        MANUAL,
        LOCKED_ON
    }

    private DriveMode driveMode = DriveMode.MANUAL;

    // CREATE DEVICES ==========================\\

    SparkFunOTOS myOtos; //Declare Odometry Computer
    public IMU rev_imu;
    public YawPitchRollAngles orientation;
    boolean imuYawFromAuton;
    private double angleOffset;
    double startAngle;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx rightFront;

    public double finalSpeedMode = 0.0;
    public final double driveSpeed = 0.66;
    public final double fastSpeed = 1.0;
    public final double slowSpeed = 0.16;
    private boolean rightStickPrevPressed = false;

    // CREATE MATCH SETTINGS / ALLIANCE ==============\\
    MatchSettings.AllianceColor alliance;

    public final MatchSettings matchSettings;

    public AimbotDriveSubsystem(HardwareMap hardwareMap, MatchSettings matchSettings) {

        this.matchSettings = matchSettings;

        alliance = matchSettings.getAllianceColor();

        rev_imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        rev_imu.initialize(new IMU.Parameters(RevOrientation));
        imuYawFromAuton = false;



        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        // All the configuration for the OTOS is done in this helper method, check it out at the bottom!
        configureOtos();

        // After configuring and resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        // SET THIS TO ALLIANCE DEPENDENT HEADING or GET FROM AUTON

        SparkFunOTOS.Pose2D storedPose = matchSettings.getStoredPose(); //get from Auton if one ran
        matchSettings.clearStoredPose(); //and then clear it so that if you are running practice it won't get a stale position

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

    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public void runManualMecanumDrive(boolean rightBumper, boolean leftBumper, double leftStickY, double leftStickX, double rightStickX, boolean yButton) {

        //Setting boolean hold
        if(rightBumper) {
            //Slow mode
            finalSpeedMode = slowSpeed;
        } else if (leftBumper) {
            //Fast mode
            finalSpeedMode = fastSpeed;
        } else {
            //Regular
            finalSpeedMode = driveSpeed;
        }

        // Reset the yaw if the user requests it
        if (yButton) {
            rev_imu.resetYaw();
        }

        orientation = rev_imu.getRobotYawPitchRollAngles();

        double botHeading = rev_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = leftStickX * Math.cos(-botHeading) - leftStickY * Math.sin(-botHeading);
        double rotY = leftStickX * Math.sin(-botHeading) + leftStickY * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightStickX), 1);
        double frontLeftPower = (rotY + rotX + rightStickX) / denominator;
        double backLeftPower = (rotY - rotX + rightStickX) / denominator;
        double frontRightPower = (rotY - rotX - rightStickX) / denominator;
        double backRightPower = (rotY + rotX - rightStickX) / denominator;

        this.leftFront.setPower(frontLeftPower * finalSpeedMode);
        this.leftBack.setPower(backLeftPower * finalSpeedMode);
        this.rightFront.setPower(frontRightPower * finalSpeedMode);
        this.rightBack.setPower(backRightPower * finalSpeedMode);

    }

    public void runAutoAlignToTag(double bearingRadians, boolean rightBumper, boolean leftBumper, double leftStickY, double leftStickX) {

        // Proportional control constant (tune as needed)
        double kP = 0.805;
        double kI;
        double kD;
        // Clamp output to avoid excessive speed
        double maxPower = 0.7;
        double alignmentThreshold = 0.05; // radians, adjust as needed
        double turnPower = 0.0;

        if (Math.abs(bearingRadians) > alignmentThreshold) {
            turnPower = -kP * bearingRadians;
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));
        }
        // No translation, only rotation
        runManualMecanumDrive(rightBumper, leftBumper, leftStickY, leftStickX, turnPower, false);
    }

    public double getOtosBearingToGoal() {

        double goalX;
        double goalY = 72;
        SparkFunOTOS.Pose2D pose = myOtos.getPosition();

        if(alliance == MatchSettings.AllianceColor.BLUE) {
            goalX = -72;
        } else { goalX = 72; }

        double dx = goalX - pose.x;
        double dy = goalY - pose.y;

        return Math.atan2(dy, dx); //in Radians
    }

    public void setCurrentPose(SparkFunOTOS.Pose2D currentPose) {
        myOtos.setPosition(currentPose);
    }

    public boolean isRobotMoving() {
        if (myOtos.getAcceleration().x < 1 && myOtos.getAcceleration().y < 1) {
            return false;
        } else {
            return true;
        }
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

