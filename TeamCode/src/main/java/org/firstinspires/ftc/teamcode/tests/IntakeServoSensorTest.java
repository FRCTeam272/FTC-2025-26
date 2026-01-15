package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemV2;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@Disabled
@TeleOp (name = "IntakeServoSensorTest", group = "Tests")
public class IntakeServoSensorTest extends SampleCommandTeleop {

    public MatchSettings matchSettings;
    private IntakeSubsystemV2 intake;

//    private IntakeFromFrontCommand intakeFromFrontCommand;
//    private IntakeFromRearCommand intakeFromRearCommand;

    @Override
    public void onInit() {

        matchSettings = new MatchSettings(blackboard);
        intake = new IntakeSubsystemV2(hardwareMap, telemetry, matchSettings);

//        intakeFromFrontCommand = new IntakeFromFrontCommand(intake);
//        intakeFromRearCommand = new IntakeFromRearCommand(intake);

    }

    @Override
    public void onStart() {

        g1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            intake.stopIntake();
        });

        g1.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            intake.inboundFront();
            intake.inboundMidFront();
            intake.inboundMidRear();
            intake.inboundRear();
        });

        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            intake.thruFrontAll();
        });

//        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(intakeFromFrontCommand);
//
//        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(intakeFromRearCommand);

        g1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            intake.outboundTransfer();
        });

        g1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            intake.stopTransfer();
        });

    }

    @Override
    public void onLoop() {

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
