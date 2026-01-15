package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp (name="Teleop14133_Comp2", group = "Competition")
public class Teleop14133_Comp2 extends SampleCommandTeleop {

    public MatchSettings matchSettings;
    MatchSettings.AllianceColor alliance;

    private DriveSubsystemV2 drive;
    private LauncherSubsystemV3 launcher;
    private IntakeSubsystemV2 intake;
    private VisionSubsystem vision;
    private LEDSubsystem leds;

    boolean lockPrevPressed = false;
//    double allianceSteering;


    ElapsedTime localizationTimer;

    @Override
    public void onInit() {
        matchSettings = new MatchSettings(blackboard);
        MatchSettings.isAuto = false;
        MatchSettings.launcherState = MatchSettings.LauncherState.STOPPED;
        MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        alliance = matchSettings.getAllianceColor();


//        if (alliance == MatchSettings.AllianceColor.BLUE) {
//            allianceSteering = -1;
//        } else {
//            allianceSteering = 1;
//        }

        drive = new DriveSubsystemV2(hardwareMap, matchSettings);
        launcher = new LauncherSubsystemV3(hardwareMap, telemetry,matchSettings);
        intake = new IntakeSubsystemV2(hardwareMap, telemetry, matchSettings);
        vision = new VisionSubsystem(hardwareMap, matchSettings);
        leds = new LEDSubsystem(hardwareMap,matchSettings);

        localizationTimer = new ElapsedTime();

        boolean startVision = vision.isDetectingAGoalTag();  //Dummy to make the camera stream work in init??
    }

    @Override
    public void onStart() {

        leds.startEndGameTimer();

    }

    @Override
    public void onLoop() {
//        telemetry.addData("Otos X Coordinates", drive.getOtosPose().x);
//        telemetry.addData("Otos Y Coordinates", drive.getOtosPose().y);
//        telemetry.addData("Otos H Coordinates", drive.getOtosPose().h);
//        telemetry.addData("Otos Bearing to Goal", drive.getOtosBearingToGoal());
//        telemetry.addData("Otos Range to Goal", drive.getOtosRangeToGoal());
//        telemetry.addData("Vision Pose X", vision.getCurrentPose().x);
//        telemetry.addData("Vision Pose Y", vision.getCurrentPose().y);
//        telemetry.addData("Vision Pose H", vision.getCurrentPose().h);
//        telemetry.addData("Vision Bearing to Goal", vision.getTagBearing());
//        telemetry.addData("Vision Range to Goal", vision.getTagRange());

        //launcher.printTelemetry(telemetry);
        intake.printTelemetry(telemetry);
        telemetry.update();

        runToggledDrive();

        intake.teleopFSM(gamepad2);
        launcher.teleopFSM(gamepad2);
        vision.teleopFSM();
        leds.update();


    }

    @Override
    public void onStop() {

    }

    public void runToggledDrive() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x; // * 1.1; // ! ADJUST TURNING SENSITIVITY HERE ! //
        boolean yButton = gamepad1.y;

        boolean lockPressed = gamepad1.right_stick_button;

        if (lockPressed && !lockPrevPressed) {
            if (drive.getDriveMode() == DriveSubsystemV2.DriveMode.MANUAL) {
                drive.setDriveMode(DriveSubsystemV2.DriveMode.LOCKED_ON_GOAL);
            } else {
                drive.setDriveMode(DriveSubsystemV2.DriveMode.MANUAL);
            }
            lockPrevPressed = true;
        }

//        if (gamepad1.right_trigger > 0.1) {
//           vision.setVisionMode(VisionSubsystem.VisionMode.ARTIFACT);
//            //drive.setDriveMode(DriveSubsystemV2.DriveMode.LOCKED_ON_ARTIFACT);
//        } else {
//            vision.setVisionMode(VisionSubsystem.VisionMode.APRILTAG);
//        }


        if (!lockPressed && lockPrevPressed) {
            lockPrevPressed = false;
        }

        if (drive.getDriveMode() == DriveSubsystemV2.DriveMode.LOCKED_ON_GOAL && vision.isDetectingAGoalTag()) {
            drive.runAutoAlignToTag(Math.toRadians(vision.getTagBearing()), rb, lb, forward, strafe);
            MatchSettings.visionState = MatchSettings.VisionState.GOAL_DETECTED;
        }
//        else if (drive.getDriveMode() == DriveSubsystemV2.DriveMode.LOCKED_ON_GOAL && !vision.isDetectingAGoalTag()) {
//            drive.runAutoAlignToTag(Math.toRadians(drive.getOtosBearingToGoal()), rb, lb, forward, strafe);
//            MatchSettings.visionState = MatchSettings.VisionState.NONE;
//        }
//        else if (drive.getDriveMode() == DriveSubsystemV2.DriveMode.LOCKED_ON_GOAL) {
//            drive.runAutoAlignToTag(Math.toRadians(drive.getOtosBearingToGoal()), rb, lb, forward, strafe);
//            MatchSettings.visionState = MatchSettings.VisionState.NONE;
//        }
//        else if (drive.getDriveMode() == DriveSubsystemV2.DriveMode.MANUAL  && vision.isDetectingAnArtifact()) {
//            drive.runManualMecanumDrive(rb, lb, forward,strafe, vision.getArtifactTurnPower(), false);
//            MatchSettings.visionState = MatchSettings.VisionState.ARTIFACT_DETECTED;
//        }
        else {
            drive.runManualMecanumDrive(rb, lb, forward, strafe, rotate, yButton);
            MatchSettings.visionState = MatchSettings.VisionState.NONE;
        }

        // Resets localization to Otos on occasion (test for future use in running launcher RPM)
        if(vision.isDetectingAGoalTag() && gamepad1.aWasPressed()) {
            SparkFunOTOS.Pose2D currentPose = vision.getCurrentPose();
            if (currentPose.x != 500) {
                drive.setCurrentPose(currentPose);
                localizationTimer.reset();
                telemetry.addLine("ReLocalized!");
            }
        }


    }
}
