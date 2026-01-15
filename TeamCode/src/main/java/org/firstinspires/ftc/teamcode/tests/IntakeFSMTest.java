package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV2;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystemV3;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@Disabled
@TeleOp (name = "IntakeFSMTest", group = "Tests")
public class IntakeFSMTest extends SampleCommandTeleop {

    public MatchSettings matchSettings;
    private IntakeSubsystemV2 intake;
    private LauncherSubsystemV3 launcher;
    private LEDSubsystem leds;

    @Override
    public void onInit() {

        matchSettings = new MatchSettings(blackboard);
        intake = new IntakeSubsystemV2(hardwareMap, telemetry, matchSettings);
        launcher = new LauncherSubsystemV3(hardwareMap, telemetry,matchSettings);
        leds = new LEDSubsystem(hardwareMap,matchSettings);
    }

    @Override
    public void onStart() {
        leds.startEndGameTimer();
    }

    @Override
    public void onLoop() {

        intake.teleopFSM(gamepad2);
        launcher.teleopFSM(gamepad2);
        leds.update();

        intake.printTelemetry(telemetry);

//        // Print Instructions every loop
//        telemetry.addLine("INTAKE CONTROLS");
//        telemetry.addLine("Press B to Inbound All");
//        telemetry.addLine("Press DpadUP to Intake from Front");
//        telemetry.addLine("Press DpadDOWN to Intake from Rear");
//        telemetry.addLine("Press DpadLEFT to PASS THRU from Front");
//        telemetry.addLine("Press DpadRIGHT to stop Intake");
//        telemetry.addLine();
//        telemetry.addLine("Press Y to Outbound Transfer");
//        telemetry.addLine("Press X to Stop Transfer");
//        telemetry.addLine("Intake will run until stopped by Button");

    }

    @Override
    public void onStop() {

    }
}
