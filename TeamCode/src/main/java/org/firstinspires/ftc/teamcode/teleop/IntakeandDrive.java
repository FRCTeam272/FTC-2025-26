package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp
public class IntakeandDrive extends SampleCommandTeleop {

    double driveSpeed = 1;

    @Override
    public void onInit() {
        drive.setDefaultCommand(drive.getDriveFieldCentric(()->g1.getLeftY(),()->-g1.getLeftX(), ()->-g1.getRightX(), driveSpeed));

    }

    @Override
    public void onStart() {

        // DRIVE SPEED CONTROLS
        if (g1.gamepad.right_bumper) {
            driveSpeed = .5;
        } else if (g1.gamepad.right_bumper && g1.gamepad.left_bumper) {
            driveSpeed =.25;
        } else {
            driveSpeed = 1;
        }

        g1.getGamepadButton(GamepadKeys.Button.Y).whenActive(()->drive.drive.resetHeadingRelative());

        // INTAKE COMMANDS
        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            intake.intakeFromFront();
        });

        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            intake.intakeFromRear();
        });

        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            intake.inboundAll();
        });

        g1.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            intake.stop();
        });



    }

    @Override
    public void onLoop() {

    }

    @Override
    public void onStop() {

    }
}
