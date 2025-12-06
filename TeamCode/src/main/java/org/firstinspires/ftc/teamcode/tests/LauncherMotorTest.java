package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

/** FIRST CONFIRM THAT LAUNCHER MOTORS ARE SET THE RIGHT DIRECTION
 *  - unplug one motor and run this opmode to confirm direction
 *
 * THEN - Tune the PIDF loop with other opmode
 *
 * THEN - Use this opmode to test launch RPMs, can increase the start speed as needed
 */

@Disabled
@TeleOp (name = "LauncherMotorTest", group = "Tests")
public class LauncherMotorTest extends SampleCommandTeleop {

    private LauncherSubsystem launcher;

    @Override
    public void onInit() {

        launcher = new LauncherSubsystem(hardwareMap,telemetry);
        launcher.setTargetRPM(1200); //set slow for initial testing

    }

    @Override
    public void onStart() {

        //DPAD_UP = RPM + 100; DPAD_DOWN = RPM - 100;
        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> launcher.setTargetRPM(launcher.getLauncherRPM() - 100));
        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> launcher.setTargetRPM(launcher.getLauncherRPM() + 100));

        //X = SPIN UP SHOOTER
        g1.getGamepadButton(GamepadKeys.Button.X).whileHeld(launcher::spinUp);
        g1.getGamepadButton(GamepadKeys.Button.Y).whileHeld(launcher::eStop);

    }

    @Override
    public void onLoop() {

        // Print intake telemetry every loop
        launcher.printTelemetry(telemetry);

        telemetry.addLine("LAUNCHER CONTROLS");
        telemetry.addLine("Press X to spin up launcher");
        telemetry.addLine("Press again or Hold X to update target RPM");
        telemetry.addLine("Press dpad Up to increase RPM");
        telemetry.addLine("Press dpad Down to decrease RPM");
        telemetry.addLine("Press Y to stop launcher");
        telemetry.addLine();

    }

    @Override
    public void onStop() {

    }
}
