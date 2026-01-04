package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.firstinspires.ftc.teamcode.util.MovingAverage;

@Config
public class LauncherSubsystemV3 {
    //===========MOTORS==========\\
    private DcMotorEx launcherRight;
    private DcMotorEx launcherLeft;

    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    MovingAverage launcherSpeedFilter;

    // --- Launcher Constants ---
    public static double TARGET_RPM = 0;         // desired launcher RPM

    // --- PIDF Coefficients ---
    public static double F = 16.5; //feedforward tune first! (16.5 without inertia wheel)
    public static double P = 8; // (8 without inertia wheel)

    public LauncherSubsystemV3(HardwareMap hardwareMap, Telemetry telemetry, MatchSettings matchSettings) {

        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");

        launcherRight.setDirection(DcMotorEx.Direction.FORWARD);
        launcherLeft.setDirection(DcMotorEx.Direction.REVERSE);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherSpeedFilter = new MovingAverage(5);

        // Apply initial PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //telemetry.addLine("shooter Init Done");
    }

    public void teleopFSM(Gamepad gamepad2){

        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        if(currentGamepad2.right_trigger >= 0.3 && previousGamepad2.right_trigger < 0.3) {
            setTargetRPM(getTargetRPM() + 25);

        }

        if(currentGamepad2.left_trigger >= 0.3 && previousGamepad2.left_trigger < 0.3) {
            setTargetRPM(getTargetRPM() - 25);

        }


        switch (MatchSettings.launcherState) {
            case STOPPED:
                if(currentGamepad2.y) {
                    setTargetRPM(Constants.launcherConstants.CLOSE_ZONE_LAUNCH_RPM);
                    MatchSettings.launcherState= MatchSettings.LauncherState.SPINNING;
                }
                else if(currentGamepad2.b) {
                    setTargetRPM(Constants.launcherConstants.MID_ZONE_LAUNCH_RPM);
                    MatchSettings.launcherState= MatchSettings.LauncherState.SPINNING;
                }
                else if(currentGamepad2.a) {
                    setTargetRPM(Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM);
                    MatchSettings.launcherState= MatchSettings.LauncherState.SPINNING;
                }
                break;

            case SPINNING:
                if(currentGamepad2.x){
                    setTargetRPM(0);
                    MatchSettings.launcherState= MatchSettings.LauncherState.STOPPED;
                }
                else if(currentGamepad2.y){
                    setTargetRPM(Constants.launcherConstants.CLOSE_ZONE_LAUNCH_RPM);
                }
                else if(currentGamepad2.b){
                    setTargetRPM(Constants.launcherConstants.MID_ZONE_LAUNCH_RPM);
                }
                else if(currentGamepad2.a){
                    setTargetRPM(Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM);
                }
                break;
            default:
                MatchSettings.launcherState= MatchSettings.LauncherState.STOPPED;
        }


        applyPIDF();
    }


    /** Applies current launcher velocity PIDF coefficients */
    public void applyPIDF() {
        launcherSpeedFilter.addReading(getLauncherRPM());
        MatchSettings.launcherAtSpeed = isAtTargetSpeed();

        // Set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        launcherRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        launcherLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        launcherRight.setVelocity(TARGET_RPM);
        launcherLeft.setVelocity(TARGET_RPM);

    }

    // --- Constants Control ---
    /**
     * Changes the target RPM of the shooter
     * @param targetRPM Set to the target RPM of the launcher
     */
    public void setTargetRPM(double targetRPM) {
        TARGET_RPM = targetRPM;
    }

    /**
     * Returns the target RPM of the launcher, used to check if velo
     * is within tolerance
     * @return returns the target RPM of the launcher
     */
    public double getTargetRPM() {
        return TARGET_RPM;
    }

    /** Stops all launcher motion immediately. */
    public void eStop() {
        setTargetRPM(0);

        launcherLeft.setPower(0);
        launcherLeft.setVelocity(0);

        launcherRight.setPower(0);
        launcherRight.setVelocity(0);
    }

    /**
     * Gets launcher current velocity
     * @return Returns current launcher RPM based on the
     *         motor rpm, ticks per rev, and gear ratio
     */
    public double getLauncherRPM() {
        double leftVelocity = launcherLeft.getVelocity(); // ticks/s of motor
        double rightVelocity = launcherRight.getVelocity(); // ticks/s of motor

        double averageVelocity = (leftVelocity+rightVelocity)/2.0;

        return averageVelocity;
    }

    public boolean isAtTargetSpeed() {
        //return ((getLauncherRPM() > (getTargetRPM() - 75)) && (getLauncherRPM() < (getTargetRPM() + 75)) && getLauncherRPM() != 0);
        return (launcherSpeedFilter.getAverage() > (getTargetRPM() - 50) && (launcherSpeedFilter.getAverage() < (getTargetRPM() + 50)) && getLauncherRPM() != 0);
    }

    public boolean isNotAtTargetSpeed() {
        return !isAtTargetSpeed();
    }

    public void printTelemetry(Telemetry telemetry) {
        double error = TARGET_RPM - getLauncherRPM();

        telemetry.addLine("LAUNCHER SUBSYSTEM");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", getLauncherRPM());
        telemetry.addData("Smoothed RPM", launcherSpeedFilter.getAverage());
        telemetry.addData("At Speed?", isAtTargetSpeed());
        telemetry.addData("Error", error);
        telemetry.update();
    }

    //============== AUTONOMOUS ACTIONS ==============\\


    // Spin up launcher wheel for Auton
    public class AutoSpinUp implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            applyPIDF();
            return true;
        }
    }
    public Action autoSpinUp() { return new AutoSpinUp(); }

    //---------------------------------------------------------------

    // Set Launcher RPM FAR for Auton
    public class AutoSetRPMFar implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            setTargetRPM(Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM - 75);
            return false;
        }
    }
    public Action autoSetRPMFar() { return  new AutoSetRPMFar(); }

    //---------------------------------------------------------------

    // Set Launcher RPM MID for Auton
    public class AutoSetRPMNear implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            setTargetRPM(Constants.launcherConstants.CLOSE_ZONE_LAUNCH_RPM + 85);
            return false;
        }
    }
    public Action autoSetRPMNear() { return  new AutoSetRPMNear(); }

    //---------------------------------------------------------------

    // Check if launcher is as speed for Auton
    public class AutoCheckAtSpeed implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            if(!isAtTargetSpeed()) {
                return true;
            } else {
            return false; }
        }
    }
    public Action autoCheckAtSpeed() { return  new AutoCheckAtSpeed(); }

    //---------------------------------------------------------------

    // Stop launcher wheel for Auton
    public class AutoStop implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            eStop();
            return false;
        }
    }
    public Action autoStop() { return new AutoStop(); }

    //---------------------------------------------------------------


}