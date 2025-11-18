package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeFromFrontCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeFromRearCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;


@TeleOp
public class DriveFCwSubsystems extends SampleCommandTeleop {
    public MatchSettings matchSettings;

    private DriveSubsystem drive;
    private IntakeSubsystem intake;

    private IntakeFromFrontCommand intakeFromFrontCommand;
    private IntakeFromRearCommand intakeFromRearCommand;


    @Override
    public void onInit() {
        matchSettings = new MatchSettings(blackboard);

        drive = new DriveSubsystem(hardwareMap, telemetry, matchSettings);
        intake = new IntakeSubsystem(hardwareMap, telemetry, matchSettings);

        intakeFromFrontCommand = new IntakeFromFrontCommand(intake);
        intakeFromRearCommand = new IntakeFromRearCommand(intake);

    }

    @Override
    public void onStart() {

        g2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(intakeFromFrontCommand);

        g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(intakeFromRearCommand);

        g2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> {
            intake.stop();
        });

        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            intake.thruAll();
        });

    }

    @Override
    public void onLoop() {
        drive.FieldCentricAllianceBased(gamepad1, telemetry);

        intake.printTelemetry(telemetry);

        // Print Instructions every loop
        telemetry.addLine("INTAKE CONTROLS");
        telemetry.addLine("Press DpadUP to Intake from Front");
        telemetry.addLine("Press DpadDOWN to Intake from Rear");
        telemetry.addLine("Press DpadLEFT to PASS THRU from Front");
        telemetry.addLine("Press DpadRIGHT to stop Intake");
        telemetry.addLine("Intake will run until stopped by Button OR Sensor");
    }

    @Override
    public void onStop() {

    }
}
