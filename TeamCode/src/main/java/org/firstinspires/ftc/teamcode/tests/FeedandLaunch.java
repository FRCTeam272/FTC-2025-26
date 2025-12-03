package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp (name = "FeedandLaunch", group = "Tests")
public class FeedandLaunch extends SampleCommandTeleop {

    public MatchSettings matchSettings;
    private IntakeSubsystemV2 intake;
    private LauncherSubsystemV2 launcher;
    private LEDSubsystem leds;
    private VisionSubsystem vision;

    @Override
    public void onInit() {
        matchSettings = new MatchSettings(blackboard);
        intake = new IntakeSubsystemV2(hardwareMap, telemetry, matchSettings);
        launcher = new LauncherSubsystemV2(hardwareMap, telemetry,matchSettings);
        leds = new LEDSubsystem(hardwareMap,matchSettings);
        vision = new VisionSubsystem(hardwareMap, matchSettings);
    }

    @Override
    public void onStart() {
        launcher.setTargetRPM(1000);
    }

    @Override
    public void onLoop() {

        if(gamepad1.dpad_up){
            launcher.setTargetRPM(launcher.getTargetRPM() + 25);
        }

        if(gamepad1.dpad_down) {
            launcher.setTargetRPM(launcher.getTargetRPM() - 25);
        }

        if(gamepad1.dpad_left) {
            launcher.spinUp();
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
            intake.stopAll();
            intake.stopTransfer();
            MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        }
        //telemetry.addData("Range to AprilTag", vision.getTagRange());

        //intake.printTelemetry(telemetry);
        launcher.printTelemetry(telemetry);
        telemetry.addLine("Beambreak",intake.artifactLaunched());

        leds.update();

    }

    @Override
    public void onStop() {

    }
}
