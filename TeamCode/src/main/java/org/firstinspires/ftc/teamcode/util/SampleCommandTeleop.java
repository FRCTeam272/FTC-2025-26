package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

// Thanks to https://github.com/14468-undefined/14468-DECODE-V2/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/ColorfulTelemetry.java

public abstract class SampleCommandTeleop extends LinearOpMode {

    public GamepadEx g1;
    public GamepadEx g2;
    //public BaseRobot robot;
    public IntakeSubsystem intake;
    public DriveSubsystem drive;

    @Override
    public void runOpMode() throws InterruptedException {
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);



        //robot = new BaseRobot(hardwareMap, new Pose2d(0,0,0));
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        drive = new DriveSubsystem(hardwareMap, new Pose2d(0,0,0));

        onInit();
        waitForStart();
        onStart();
        while(opModeIsActive() && !isStopRequested()){
            onLoop();
            g1.readButtons();
            g2.readButtons();
            CommandScheduler.getInstance().run();

        }
        onStop();
        CommandScheduler.getInstance().reset();
    }

    /**
     * This method is run once upon the initialization of the Teleop
     */
    public abstract void onInit();

    /**
     * This method is run once upon start of the Teleop
     */
    public abstract void onStart();

    /**
     * This method is repeated as long as the Teleop is active
     */
    public abstract void onLoop();

    /**
     * This method is run if the Teleop is manually stopped
     */
    public abstract void onStop();

}
