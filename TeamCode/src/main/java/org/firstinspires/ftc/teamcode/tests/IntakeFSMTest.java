package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.IntakeFromFrontCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeFromRearCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV2;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp (name = "IntakeServoSensorTest", group = "Tests")
public class IntakeFSMTest extends SampleCommandTeleop {

    public MatchSettings matchSettings;
    private IntakeSubsystemV2 intake;

    @Override
    public void onInit() {

        matchSettings = new MatchSettings(blackboard);
        intake = new IntakeSubsystemV2(hardwareMap, telemetry, matchSettings);
    }

    @Override
    public void onStart() {



    }

    @Override
    public void onLoop() {

        intake.teleopFSM(gamepad2);

        intake.printTelemetry(telemetry);

        // Print Instructions every loop
        telemetry.addLine("INTAKE CONTROLS");
        telemetry.addLine("Press B to Inbound All");
        telemetry.addLine("Press DpadUP to Intake from Front");
        telemetry.addLine("Press DpadDOWN to Intake from Rear");
        telemetry.addLine("Press DpadLEFT to PASS THRU from Front");
        telemetry.addLine("Press DpadRIGHT to stop Intake");
        telemetry.addLine();
        telemetry.addLine("Press Y to Outbound Transfer");
        telemetry.addLine("Press X to Stop Transfer");
        telemetry.addLine("Intake will run until stopped by Button");

    }

    @Override
    public void onStop() {

    }
}
