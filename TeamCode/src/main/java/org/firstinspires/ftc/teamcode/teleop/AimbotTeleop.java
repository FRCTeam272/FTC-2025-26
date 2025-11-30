package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.IntakeFromFrontCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeFromRearCommand;
import org.firstinspires.ftc.teamcode.subsystems.AimbotDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp
public class AimbotTeleop extends SampleCommandTeleop {

    public MatchSettings matchSettings;

    private AimbotDriveSubsystem drive;
    private LauncherSubsystem launcher;
    private IntakeSubsystem intake;
    private VisionSubsystem vision;

    private IntakeFromFrontCommand intakeFromFrontCommand;
    private IntakeFromRearCommand intakeFromRearCommand;

    boolean libCode = false;
    boolean lockPrevPressed = false;

    ElapsedTime localizationTimer;

    @Override
    public void onInit() {
        matchSettings = new MatchSettings(blackboard);

        drive = new AimbotDriveSubsystem(hardwareMap, matchSettings);
        launcher = new LauncherSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry, matchSettings);
        vision = new VisionSubsystem(hardwareMap);

        intakeFromFrontCommand = new IntakeFromFrontCommand(intake);
        intakeFromRearCommand = new IntakeFromRearCommand(intake);

        localizationTimer = new ElapsedTime();
    }

    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {
        runToggledDrive();
        runVision();
    }

    @Override
    public void onStop() {

    }

    public void runToggledDrive() {
        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;
        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x * 1.1; // ! ADJUST TURNING SENSITIVITY HERE ! //
        boolean yButton = gamepad1.y;

        boolean lockPressed = gamepad1.right_stick_button;

        if (lockPressed && !lockPrevPressed) {
            if (drive.getDriveMode() == AimbotDriveSubsystem.DriveMode.MANUAL) {
                drive.setDriveMode(AimbotDriveSubsystem.DriveMode.LOCKED_ON);
            } else {
                drive.setDriveMode(AimbotDriveSubsystem.DriveMode.MANUAL);
            }
            lockPrevPressed = true;
        }

        if (!lockPressed && lockPrevPressed) {
            lockPrevPressed = false;
        }

        if (drive.getDriveMode() == AimbotDriveSubsystem.DriveMode.LOCKED_ON && vision.isDetectingAGoalTag())
        {
            drive.runAutoAlignToTag(Math.toRadians(vision.getTagBearing()), rb, lb, leftY, leftX);
        }
        else if (drive.getDriveMode() == AimbotDriveSubsystem.DriveMode.LOCKED_ON && !vision.isDetectingAGoalTag()) {
            drive.runAutoAlignToTag(drive.getOtosBearingToGoal(), rb, lb, leftY, leftX);
        } else {
            drive.runManualMecanumDrive(rb,lb,leftY,leftX,rightX,yButton);
        }
    }

    public void runVision() {


        if(vision.isDetectingAGoalTag() && localizationTimer.seconds() > 20 && !drive.isRobotMoving()); {
            SparkFunOTOS.Pose2D currentPose = vision.getCurrentPose();
            drive.setCurrentPose(currentPose);
            localizationTimer.reset();
        }
        //vision.scanMotifTagSequence();
    }
}
