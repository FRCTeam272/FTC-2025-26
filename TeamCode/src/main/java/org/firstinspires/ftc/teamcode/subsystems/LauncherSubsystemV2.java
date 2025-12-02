package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

public class LauncherSubsystemV2 {
    //===========MOTORS==========\\
    private DcMotorEx launcherRight;
    private DcMotorEx launcherLeft;

    // --- Shooter Constants ---
    public static double TARGET_RPM = 2500.0;         // desired shooter RPM
    private static double MOTOR_RPM = 6000;          // motor RPM (based on max motor rpm)
    private static double GEAR_RATIO = 1;            // gear ratio from motor to shooter
    private static double TICKS_PER_REV = 28;       // motor encoder ticks per revolution
    private boolean active;

    // --- PIDF Coefficients ---
    //working values on November 14th, 2025
    public static double kP = 20;
    public static double kI = 0.0;
    public static double kD = 5.0;
    public static double kF = 24.0;

    /**
     * Initialises the shooter in the hardwareMap, sets default shooter values
     * @param hardwareMap pulls HardwareMap from teleOp class
     *                    to initialise motor
     * telemetry Allows the class to add telemetry to the phone
     * @param defaultTargetRPM Sets the default target RPM of the shooter
     *                        Set to the initial target RPM of your
     *                        shooter
     * @param defaultMotorRPM Sets the default RPM of the motor
     *                       Set to the RPM of the motor being used
     * @param defaultGearRatio Sets the default shooter gear ratio
     *                        Set to the gear ratio between the motor and shooterwheel
     * @param defaultTicks Sets the default ticks of the motor
     *                    Set to the encoder ticks of your motor
     */
    public LauncherSubsystemV2(HardwareMap hardwareMap, double defaultTargetRPM,
                               double defaultMotorRPM, double defaultGearRatio, double defaultTicks, MatchSettings matchSettings) {

        //telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());



        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");

        launcherRight.setDirection(DcMotorEx.Direction.FORWARD);
        launcherLeft.setDirection(DcMotorEx.Direction.REVERSE);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configs defaults
        setTargetRPM(defaultTargetRPM);
        setMotorRPM(defaultMotorRPM);
        setGearRatio(defaultGearRatio);
        setTicksPerRev(defaultTicks);
        active = Math.abs(getTargetRPM()) > 0;

        // Apply initial PIDF coefficients
        applyPIDF();

        //telemetry.addLine("shooter Init Done");
    }

    /// Only use if the constants in this file are correct
    public LauncherSubsystemV2(HardwareMap hardwareMap, Telemetry telemetry, MatchSettings matchSettings) {
        this(hardwareMap, TARGET_RPM, MOTOR_RPM, GEAR_RATIO, TICKS_PER_REV, matchSettings);
    }

    public void teleopFSM(Gamepad gamepad2){
        switch (MatchSettings.launcherState) {
            case STOPPED:
                if(gamepad2.x) {
                    spinUp();
                    setTargetRPM(Constants.launcherConstants.MID_ZONE_LAUNCH_RPM);
                    MatchSettings.launcherState= MatchSettings.LauncherState.SPINNING;
                }
                else if(gamepad2.a) {
                    spinUp();
                    setTargetRPM(Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM);
                    MatchSettings.launcherState= MatchSettings.LauncherState.SPINNING;
                }
                break;

            case SPINNING:
                if(gamepad2.b){
                    eStop();
                    MatchSettings.launcherState= MatchSettings.LauncherState.STOPPED;
                }
                else if(gamepad2.x){
                    setTargetRPM(Constants.launcherConstants.MID_ZONE_LAUNCH_RPM);
                    spinUp();
                }
                else if(gamepad2.a){
                    setTargetRPM(Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM);
                    spinUp();
                }
                break;
            default:
                MatchSettings.launcherState= MatchSettings.LauncherState.STOPPED;
        }

        if(gamepad2.a){
            setTargetRPM(Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM);
        }
        if(gamepad2.x){
            setTargetRPM(Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM);
        }
    }

    // --- PIDF ---
    /**
     * Sets shooter PIDF coefficients manually
     * @param kf Set to a low value, just enough that the shooter wheel
     *           begins to rotate
     * @param kp Increase kP after kF until the shooter wheel reaches the target speed
     * @param kd Try changing the target speed of the shooter from a low value
     *           to a high value and vise versa. Use this to reduce the
     *           oscillations when changing speeds
     * @param ki Most times this won't need to be tuned
     */
    public void setShooterPIDF(double kf, double kp, double kd, double ki) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        applyPIDF();
    }

    /** Applies current shooter velocity PIDF coefficients */
    public void applyPIDF() {
        launcherLeft.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        launcherRight.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    // --- Constants Control ---
    /**
     * Changes the target RPM of the shooter
     * @param targetRPM Set to the target RPM of the shooter
     */
    public void setTargetRPM(double targetRPM) {
        TARGET_RPM = targetRPM;
    }

    /**
     * Returns the target RPM of the shooter, used to check if velo
     * is within tolerance
     * @return returns the target RPM of the shooter
     */
    public double getTargetRPM() {
        return TARGET_RPM;
    }

    /**
     * Changes the RPM of the motor
     * @param motorRPM Set to the RPM of the motor
     *
     */
    public void setMotorRPM(double motorRPM) {
        MOTOR_RPM = motorRPM;
    }

    /**
     * Changes the gear ratio between the motor and the shooter
     * @param gearRatio Set to the gear ratio used between the
     *                  motor and shooter
     *      1.0 is a 1:1 gear ratio
     *      2.5 is a 2.5:1 gear increase
     *      0.5 is a 0.5:1 gear reduction
     */
    public void setGearRatio(double gearRatio) {
        GEAR_RATIO = gearRatio;
    }

    /**
     * Returns the current gear ratio of the shooter
     * @return returns the current GEAR_RATIO of the shooter system
     */
    public double getGearRatio() {
        return GEAR_RATIO;
    }

    /**
     * Changes the Ticks Per Revolution of the motor
     * Called Encoder Resolution on Rev website
     * @param TicksPerRev Set to the Ticks per rev of the motor
     *                    being used
     */
    public void setTicksPerRev(double TicksPerRev) {
        TICKS_PER_REV = TicksPerRev;
    }

    /**
     * Returns the current Ticks Per Rev of the shooter
     * @return returns the TICKS_PER_REV of the shooter flywheel
     */
    public double getTicksPerRev() {
        return TICKS_PER_REV;
    }

    /**
     * Calculates ticks per second based on target RPM
     * Sets the target velocity
     * */
    public void spinUp() {
        double targetTicksPerSec = ((TARGET_RPM / GEAR_RATIO) * TICKS_PER_REV) / 60;
        launcherLeft.setVelocity(targetTicksPerSec);
        launcherRight.setVelocity(targetTicksPerSec);


        active = Math.abs(getTargetRPM()) > 0;
    }

    /** Stops all shooter motion immediately. */
    public void eStop() {
        launcherLeft.setPower(0);
        launcherLeft.setVelocity(0);

        launcherRight.setPower(0);
        launcherRight.setVelocity(0);
    }

    /**
     * Gets shooter current velocity
     * @return Returns current shooter RPM based on the
     *         motor rpm, ticks per rev, and gear ratio
     */
    public double getLauncherRPM() {
        double leftCurrTicksPerSec = launcherLeft.getVelocity(); // ticks/s of motor
        double rightCurrTicksPerSec = launcherRight.getVelocity(); // ticks/s of motor

        double averageTPS = (leftCurrTicksPerSec+rightCurrTicksPerSec)/2.0;
        double currMotorRPM = (averageTPS * 60.0) / TICKS_PER_REV;
        double currLauncherRPM = currMotorRPM * GEAR_RATIO;

        return currLauncherRPM;
    }

    /**
     * Gets shooter motor current velocity
     * @return Returns motor voltage
     */
    public double getMotorVoltage() {
        double leftAmps = launcherLeft.getCurrent(CurrentUnit.AMPS);
        double rightAmps = launcherRight.getCurrent(CurrentUnit.AMPS);
        return (leftAmps + rightAmps)/2.0;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isAtTargetSpeed() {
        return ((getLauncherRPM() > (getTargetRPM() - 200)) && (getLauncherRPM() < (getTargetRPM() + 100)) && getLauncherRPM() != 0);
    }

    public boolean isNotAtTargetSpeed() {
        return !isAtTargetSpeed();
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addLine("LAUNCHER SUBSYSTEM");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", getLauncherRPM());
        telemetry.addData("At Speed?", isAtTargetSpeed());
        telemetry.update();
    }

    //============== AUTONOMOUS ACTIONS ==============\\


    // Spin up launcher wheel for Auton
    public class AutoSpinUp implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            spinUp();
            return false;
        }
    }
    public Action autoSpinUp() { return new AutoSpinUp(); }

    //---------------------------------------------------------------

    // Set Launcher RPM FAR for Auton
    public class AutoSetRPMFar implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            setTargetRPM(Constants.launcherConstants.FAR_ZONE_LAUNCH_RPM);
            return false;
        }
    }
    public Action autoSetRPMFar() { return  new AutoSetRPMFar(); }

    //---------------------------------------------------------------

    // Set Launcher RPM MID for Auton
    public class AutoSetRPMMid implements Action {
        @Override
        public boolean run (@NonNull TelemetryPacket packet) {
            setTargetRPM(Constants.launcherConstants.MID_ZONE_LAUNCH_RPM);
            return false;
        }
    }
    public Action autoSetRPMMid() { return  new AutoSetRPMMid(); }

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