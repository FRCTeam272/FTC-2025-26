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
        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(() -> {
            driveSpeed = .5;
        });
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(() -> {
            driveSpeed = .2;
        });

        g1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(() -> {
            driveSpeed = 1;
        });
        g1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(() -> {
            driveSpeed = 1;
        });

        // DRIVE YAW RESET

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
