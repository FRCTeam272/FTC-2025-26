package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LauncherSubsystem extends SubsystemBase {

    //===========MOTORS==========\\
    private final MotorEx launcherRight;
    private final MotorEx launcherLeft;

    //============Servos===========\\
    //private final Servo hood;

    //=========== PIDF CONTROL ===========\\
    private final PIDFController launcherPID;
    private double targetRPM = 0;
    private double currentRPM = 0;

    // PIDF constants (tune these)
    public static double kP = 0.0008;
    public static double kI = 0.0001;
    public static double kD = 0.0001;
    public static double kF = 0.0002;

    private final double TICKS_PER_REV = 28.0; //for 6k rpm

    //=========== TELEMETRY ===========\\
    private final Telemetry telemetry;

    public LauncherSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        launcherRight = new MotorEx(hardwareMap, "launcherRight");
        launcherLeft = new MotorEx(hardwareMap, "launcherLeft");

        launcherRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        launcherLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        launcherRight.setInverted(false);
        launcherLeft.setInverted(true);

        // Initialize PIDF
        launcherPID = new PIDFController(kP, kI, kD, kF);
        launcherPID.setTolerance(50);
    }

    //============== CONTROL METHODS ==============\\

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    // called repeatedly to spin up and regulate shooter speed
    public void spinUp() {

        launcherPID.setPIDF(kP, kI, kD, kF);
        //get current velocity
        currentRPM = ((launcherLeft.getVelocity() + launcherRight.getVelocity()) / 2.0) * (60.0 / TICKS_PER_REV);

        //compute feedforward
        double ff = kF * targetRPM;
        double pidOutput = launcherPID.calculate(currentRPM, targetRPM);

        // 0<power<1
        double totalOutput = Range.clip(ff + pidOutput, 0, 1);

        //set power
        launcherRight.set(totalOutput);
        launcherLeft.set(totalOutput);
    }

    public void stop() {

        launcherRight.set(0);
        launcherLeft.set(0);
    }

    public boolean atSpeed() {
        double avgVelocity = (launcherRight.getVelocity() + launcherLeft.getVelocity()) / 2.0;
        double targetVelocity = rpmToTicksPerSecond(targetRPM);
        return Math.abs(avgVelocity - targetVelocity) < 100;
    }

    //============== UTIL ==============\\

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private double tpsToRPM(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }

    @Override
    public void periodic() {

    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addLine("SHOOTER SUBSYSTEM");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Right RPM", tpsToRPM(launcherRight.getVelocity()));
        telemetry.addData("Left RPM", tpsToRPM(launcherLeft.getVelocity()));
        telemetry.addData("At Speed?", atSpeed());
        telemetry.update();
    }
}
