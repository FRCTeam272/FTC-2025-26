package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp
public class IntakeServoTest extends SampleCommandTeleop {
    private IntakeSubsystem intake;

    @Override
    public void onInit() {

        intake = new IntakeSubsystem(hardwareMap, telemetry);

    }

    @Override
    public void onStart() {

        g1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            intake.intakeFromFront();
        });

        g1.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            intake.intakeFromRear();
        });

        g1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            intake.stop();
        });

    }

    @Override
    public void onLoop() {

        // Print Instructions every loop
        telemetry.addLine("INTAKE CONTROLS");
        telemetry.addLine("Press Y to Intake from Front");
        telemetry.addLine("Press A to Intake from Rear");
        telemetry.addLine("Press X to Stop");
    }

    @Override
    public void onStop() {

    }
}
