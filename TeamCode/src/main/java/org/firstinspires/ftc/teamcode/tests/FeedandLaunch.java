package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp (name = "FeedandLaunch", group = "Tests")
public class FeedandLaunch extends SampleCommandTeleop {

    public MatchSettings matchSettings;
    private IntakeSubsystemV2 intake;
    private LauncherSubsystemV3 launcher;
    private LEDSubsystem leds;
    private VisionSubsystem vision;

    @Override
    public void onInit() {
        matchSettings = new MatchSettings(blackboard);
        intake = new IntakeSubsystemV2(hardwareMap, telemetry, matchSettings);
        launcher = new LauncherSubsystemV3(hardwareMap, telemetry,matchSettings);
        leds = new LEDSubsystem(hardwareMap,matchSettings);
        vision = new VisionSubsystem(hardwareMap, matchSettings);
    }

    @Override
    public void onStart() {
        launcher.setTargetRPM(0);
    }

    @Override
    public void onLoop() {

        if(gamepad1.dpadUpWasReleased()){
            launcher.setTargetRPM(launcher.getTargetRPM() + 25);
        }

        if(gamepad1.dpadDownWasReleased()) {
            launcher.setTargetRPM(launcher.getTargetRPM() - 25);
        }

        if(gamepad1.dpad_left) {
            MatchSettings.launcherState = MatchSettings.LauncherState.SPINNING;
        }

        if(gamepad1.dpad_right) {
            launcher.eStop();
            MatchSettings.launcherState = MatchSettings.LauncherState.STOPPED;
        }

        if(gamepad1.right_bumper) {
            intake.inboundRear();
            intake.inboundMidRear();
            intake.inboundMidFront();
            intake.outboundTransfer();
            MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_REAR;
        }

        if (intake.artifactLaunched() || gamepad1.x) {
            intake.stopIntake();
            intake.stopTransfer();
            MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        }

        if (MatchSettings.launcherState != MatchSettings.LauncherState.STOPPED) {
            launcher.applyPIDF();
        }
        //telemetry.addData("Range to AprilTag", vision.getTagRange());

        //intake.printTelemetry(telemetry);
        telemetry.addData("TagRange",vision.getTagRange());
        launcher.printTelemetry(telemetry);



        leds.update();

    }

    @Override
    public void onStop() {

    }
}
