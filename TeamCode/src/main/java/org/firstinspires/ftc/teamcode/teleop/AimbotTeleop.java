package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.IntakeFromFrontCommandV2;
import org.firstinspires.ftc.teamcode.commands.IntakeFromRearCommandV2;
import org.firstinspires.ftc.teamcode.commands.Launch3QuickCommand;
import org.firstinspires.ftc.teamcode.subsystems.AimbotDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp (name="AimbotTeleop", group = "Competition")
public class AimbotTeleop extends SampleCommandTeleop {

    public MatchSettings matchSettings;

    private AimbotDriveSubsystem drive;
    private LauncherSubsystem launcher;
    private IntakeSubsystem intake;
    private VisionSubsystem vision;
    private LEDSubsystem leds;

    private IntakeFromFrontCommandV2 intakeFromFrontCommandV2;
    private IntakeFromRearCommandV2 intakeFromRearCommandV2;
    private Launch3QuickCommand launch3QuickCommand;

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
        leds = new LEDSubsystem(hardwareMap,matchSettings);

        intakeFromFrontCommandV2 = new IntakeFromFrontCommandV2(intake, leds);
        intakeFromRearCommandV2 = new IntakeFromRearCommandV2(intake, leds);
        launch3QuickCommand = new Launch3QuickCommand(intake,launcher, leds);

        localizationTimer = new ElapsedTime();
    }

    @Override
    public void onStart() {

        leds.startEndGameTimer();

        //Intake Controls

        //Dpad Up - Intake From Front
        g2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(intakeFromFrontCommandV2);

        // Dpad Down - Intake From Rear
        g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(intakeFromRearCommandV2);

        // Dpad Right - Stop Intake
        g2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            intake.stopAll();
            leds.setIntakingStopped();
        });

        // Dpad Left - Pass Thru From Front
        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            intake.thruFrontAll();
            leds.setIntakingThru();
        });

        // Launch Controls

        // B - Quick and Simple Launch 3
        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(launch3QuickCommand);

        // A - Launcher Far Zone
        g2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            launcher.setTargetRPM(2500);
            launcher.spinUp();
        });

        // X - Launcher Mid Zone
        g2.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            launcher.setTargetRPM(2100);
            launcher.spinUp();
        });

        // Y - Launcher Near Zone

        // B - Launcher Stop
        g2.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            launcher.eStop();
        });

    }

    @Override
    public void onLoop() {
        runToggledDrive();
        runVision();


        if(launcher.isAtTargetSpeed()) {
            leds.setLauncherAtSpeed();
        } else {
            leds.setLauncherNotAtSpeed();
        }

        leds.update();
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

        // Resets localization to Otos on occasion (test for future use in running launcher RPM)
        if(vision.isDetectingAGoalTag() && localizationTimer.seconds() > 20 && !drive.isRobotMoving()) {
            SparkFunOTOS.Pose2D currentPose = vision.getCurrentPose();
            drive.setCurrentPose(currentPose);
            localizationTimer.reset();
        }
        //vision.scanMotifTagSequence();
    }
}
