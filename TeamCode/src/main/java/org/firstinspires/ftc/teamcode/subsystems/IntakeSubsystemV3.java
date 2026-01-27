package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;

public class IntakeSubsystemV3 {

    // ================== MOTORS ================== \\
    private final DcMotor intakeFront;
    private final DcMotor intakeRear;

    // ================== SERVOS ================== \\
    private final CRServo leftTransfer;
    private final CRServo rightTransfer;

    // ================== SENSORS ================== \\
    private final RevColorSensorV3 midColorSens;
    private final DigitalChannel launcherBeamBreak;
    private final DigitalChannel frontBeamBreak;
    private final DigitalChannel rearBeamBreak;

    double possessionDistanceMid = Constants.intakeConstants.DISTANCE_FOR_POSSESSION_MID;
    double intaking = Constants.intakeConstants.INTAKE_POWER;
    double outtaking = Constants.intakeConstants.REVERSE_INTAKE_POWER;

    int artifactHoldPosition = -50; //tune and test

    boolean possession = false; // Variable telling whether we have possession of a game piece or not

    // Possession and Color Variables for each auto intake round. Starts true for preload. Resets and reads during intake!
    boolean possessionFront = true;
    boolean possessionMid = true;
    boolean possessionRear = true;

    String intakeLoadDirection = "TBD";
    int launchCounter = 0;

    private static final int RGB_COMPONENTS = 3;
    private final double[] rgbValues = new double[RGB_COMPONENTS];

    MatchSettings.ArtifactColor colorInSlotFront = MatchSettings.ArtifactColor.UNKNOWN;
    MatchSettings.ArtifactColor colorInSlotMid = MatchSettings.ArtifactColor.UNKNOWN;
    MatchSettings.ArtifactColor colorInSlotRear = MatchSettings.ArtifactColor.UNKNOWN;

    private static final double HSV_GREEN_MIN_H = 150;
    private static final double HSV_GREEN_MAX_H = 163;
    private static final double HSV_PURPLE_MIN_H = 165;
    private static final double HSV_PURPLE_MAX_H = 240;
    private static final double HSV_MIN_SAT = 0.3;
    private static final double HSV_MIN_VAL = 0.1;

    //=========== TELEMETRY ===========\\
    private final Telemetry telemetry;

    // CREATE MATCH SETTINGS / MOTIF ==============\\

    public final MatchSettings matchSettings;

    //FSM tracker variables
    private boolean initialized = false;
    private boolean midTransferred = false;
    private boolean launchBBTripped = false;
    private ElapsedTime launchTimer = new ElapsedTime();

    private ElapsedTime autoTimer = new ElapsedTime();
    private double autoCancelSeconds = 28;

    public IntakeSubsystemV3(HardwareMap hardwareMap, Telemetry telemetry, MatchSettings matchSettings) {

        this.telemetry = telemetry;
        this.matchSettings = matchSettings;

        if (MatchSettings.isAuto) {
            colorInSlotFront = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotMid = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotRear = MatchSettings.ArtifactColor.GREEN;
        }

        // ================== MOTORS ================== \\
        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");
        intakeRear = hardwareMap.get(DcMotor.class, "intakeRear");

        intakeFront.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRear.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ================== SERVOS ================== \\
        leftTransfer = hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");

        leftTransfer.setDirection(CRServo.Direction.REVERSE);
        rightTransfer.setDirection(CRServo.Direction.FORWARD);

        // ================== SENSORS ================== \\
        midColorSens = hardwareMap.get(RevColorSensorV3.class, "midColor");

        midColorSens.setGain(10);

        launcherBeamBreak = hardwareMap.get(DigitalChannel.class, "launcherBB");
        frontBeamBreak = hardwareMap.get(DigitalChannel.class, "frontBB");
        rearBeamBreak = hardwareMap.get(DigitalChannel.class, "rearBB");

        launcherBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        frontBeamBreak.setMode(DigitalChannel.Mode.INPUT);
        rearBeamBreak.setMode(DigitalChannel.Mode.INPUT);


    }

    public void teleopFSM(Gamepad gamepad2) {

        //cancel intaking/transfer & STOP button
        if (gamepad2.dpad_right) {
            stopIntake();
            stopTransfer();
            initialized = false;
            MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        }

        if (gamepad2.dpad_up) {
            stopIntake();
            stopTransfer();
            initialized = false;
            MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_FRONT;
        }

        if (gamepad2.dpad_down) {
            stopIntake();
            stopTransfer();
            initialized = false;
            MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_REAR;
        }

        if (gamepad2.dpad_left) {
            stopIntake();
            stopTransfer();
            initialized = false;
            MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_THRU;
        }

        if (gamepad2.left_bumper) {
            stopIntake();
            stopTransfer();
            initialized = false;
            MatchSettings.intakeState = MatchSettings.IntakeState.LAUNCHING_1_SIMPLE;
        }

        if (gamepad2.right_bumper) {
            stopIntake();
            stopTransfer();
            initialized = false;
            MatchSettings.intakeState = MatchSettings.IntakeState.LAUNCHING_3_FAST;
        }

        switch (MatchSettings.intakeState) {
            case STOPPED:
                initialized = false;
                // waiting for input
                if (gamepad2.dpad_up) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_FRONT;
                } else if (gamepad2.dpad_down) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_REAR;
                } else if (gamepad2.dpad_left) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_THRU;
                } else if (gamepad2.left_bumper) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.LAUNCHING_1_SIMPLE;
                } else if (gamepad2.right_bumper) {
                    MatchSettings.intakeState = MatchSettings.IntakeState.LAUNCHING_3_FAST;
                }
                break;
            case INTAKING_FRONT:
                if (!initialized) { // powers on intake, if it is not on
                    clearIntakeLoadColors();
                    clearIntakePossessions();
                    inboundFront(); // turn on all front intake motor
                    outboundRear();
                    inboundTransfer();
                    intakeLoadDirection = "FRONT";
                    midTransferred = false;
                    initialized = true;
                }
                if (!rearPossession() && !possessionRear) { //check for rear possession
                    if (midPossession()) {
                        // Check for color passing through Mid
                        if (colorInSlotRear == MatchSettings.ArtifactColor.UNKNOWN) {
                            colorInSlotRear = colorDetected(midColorSens);
                        }
                    }
                }
                if (rearPossession() && !possessionRear && !midPossession()) {
                    possessionRear = true;
                    stopRear();
                }
                if (possessionRear && midPossession() && !possessionMid) {
                    colorInSlotMid = colorDetected(midColorSens);
                    //holdArtifactRear();
                    possessionMid = true;
                }
                if (possessionRear && possessionMid && frontPossession()) {
                    stopIntake();
                    stopTransfer();
                    possessionFront = true;
                    initialized = false;
                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                }
                break;
            case INTAKING_REAR:
                if (!initialized) { // powers on intake, if it is not on
                    clearIntakeLoadColors();
                    clearIntakePossessions();
                    inboundRear(); // turn on all rear intaking servos
                    outboundFront();
                    inboundTransfer();
                    intakeLoadDirection = "REAR";
                    midTransferred = false;
                    initialized = true;
                }
                if (!frontPossession() && !possessionFront) { //check for rear possession
                    if (midPossession()) {
                        // Check for color passing through Mid
                        if (colorInSlotFront == MatchSettings.ArtifactColor.UNKNOWN) {
                            colorInSlotFront = colorDetected(midColorSens);
                        }
                    }
                }
                if (frontPossession() && !possessionFront && !midPossession()) {
                    possessionFront = true;
                    stopFront();
                }
                if (possessionFront && midPossession() && !possessionMid) {
                    colorInSlotMid = colorDetected(midColorSens);
                    possessionMid = true;
                }
                if (possessionFront && possessionMid && rearPossession()) {
                    stopIntake();
                    stopTransfer();
                    possessionRear = true;
                    initialized = false;
                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                }
                break;
            case INTAKING_THRU:
                inboundThru();
                inboundTransfer();
                break;
            case LAUNCHING_1_SIMPLE:
                if (!initialized) {
                    initialized = true;
                    inboundFront();
                    inboundRear();
                    outboundTransfer();
                }
                if (artifactLaunched() ) {
                    stopIntake();
                    stopTransfer();
                    initialized = false;
                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                }
                break;
            case LAUNCHING_3_FAST:
                if (!initialized) {
                    initialized = true;

                    inboundFront();
                    inboundRear();
                    outboundTransfer();
                    launchTimer.reset();

                }
                if (launchTimer.seconds() > 3) {
                    stopIntake();
                    stopTransfer();
                    initialized = false;
                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                }
                break;

            default: // should never be reached
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
        }
    }

    //============== CONTROL METHODS ==============\\

    // COLOR SENSOR METHODS ==========================\\


    // Checks the launcher beam break to see if an artifact has passed through
    public boolean artifactLaunched() {
        if (launchTimer.seconds() > 0.55) {
            launchBBTripped = false;
        }
        if (!launcherBeamBreak.getState() && !launchBBTripped) {
            launchBBTripped = true;
            launchTimer.reset();
            return false;
        } else if (launchBBTripped && launchTimer.seconds() > 0.4 ) {
            launchBBTripped = false;
            return true;
        } else {
            return false;
        }
    }

    private MatchSettings.ArtifactColor colorDetected(RevColorSensorV3 sensor) { // Returns color detected if there is one
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        boolean isGreen = (hue >= HSV_GREEN_MIN_H && hue <= HSV_GREEN_MAX_H);
        boolean isPurple = (hue >= HSV_PURPLE_MIN_H && hue <= HSV_PURPLE_MAX_H);
        boolean strongColor = sat >= HSV_MIN_SAT && val >= HSV_MIN_VAL;

        if (strongColor && isGreen) {
            return MatchSettings.ArtifactColor.GREEN;
        } else if (strongColor && isPurple) {
            return MatchSettings.ArtifactColor.PURPLE;
        } else {
            return MatchSettings.ArtifactColor.UNKNOWN;
        }
    }

    public void clearIntakeLoadColors() {
        colorInSlotFront = MatchSettings.ArtifactColor.UNKNOWN;
        colorInSlotMid = MatchSettings.ArtifactColor.UNKNOWN;
        colorInSlotRear = MatchSettings.ArtifactColor.UNKNOWN;
    }

    public void setRearColor() {
        colorInSlotRear = colorDetected(midColorSens);
    }

    public void setMidColor() {
        colorInSlotMid = colorDetected(midColorSens);
    }

    public void setFrontColor() {
        colorInSlotFront = colorDetected(midColorSens);
    }

    // DISTANCE SENSOR METHODS ==========================\\

    private boolean possessionDetectedMid(RevColorSensorV3 sensor) { //for bulk read before launching
        possession = sensor.getDistance(DistanceUnit.CM) < possessionDistanceMid;
        return possession;
    }


    public void readIntakePossessions() { // bulk read all sensors for initial possession values prior to launching
        possessionFront = !frontBeamBreak.getState();
        possessionMid = possessionDetectedMid(midColorSens);
        possessionRear = !rearBeamBreak.getState();
    }

    public void clearIntakePossessions() { // clear Intake load possession values
        possessionFront = false;
        possessionMid = false;
        possessionRear = false;
    }

    public boolean frontPossession() { //returns true if an artifact breaks beam
        return !frontBeamBreak.getState();
    }

    public boolean midPossession() { // returns true if there is an artifact in distance
        possession = midColorSens.getDistance(DistanceUnit.CM) < possessionDistanceMid;
        return possession;
    }

    public boolean rearPossession() { //returns true if an artifact breaks beam
        return !rearBeamBreak.getState();
    }

    // IN-BOUND METHODS ==========================\\
    public void inboundFront() {
        intakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeFront.setPower(intaking);
    }

    public void inboundRear() {
        intakeRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRear.setPower(intaking);
    }

    public void inboundThru() {
        intakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeFront.setPower(intaking);
        intakeRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRear.setPower(outtaking);
    }

    public void inboundTransfer() {
        leftTransfer.setPower(intaking);
        rightTransfer.setPower(intaking);
    }

    // OUT-BOUND METHODS ==========================\\
    public void outboundFront() {
        intakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeFront.setPower(outtaking/2);
    }

    public void outboundRear() {
        intakeRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRear.setPower(outtaking/2);
    }

    public void outboundTransfer() {
        leftTransfer.setPower(outtaking);
        rightTransfer.setPower(outtaking);
    }

    public void holdArtifactFront() {
        intakeFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeFront.setTargetPosition(artifactHoldPosition);
        intakeFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeFront.setPower(outtaking);
    }

    public void holdArtifactRear() {
        intakeRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeRear.setTargetPosition(artifactHoldPosition);
        intakeRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeRear.setPower(outtaking);
    }

    // STOPPING METHODS ==========================\\
    public void stopFront() {
        intakeFront.setPower(0);
    }

    public void stopRear() {
        intakeRear.setPower(0);
    }

    public void stopIntake() {
        stopFront();
        stopRear();
    }

    public void stopTransfer() {
        leftTransfer.setPower(0);
        rightTransfer.setPower(0);
    }

    // TRANSFER TO LAUNCHER METHODS ==========================\\

    public void printTelemetry(Telemetry telemetry) { // Only for Testing. DO NOT constantly read sensors during teleop
        telemetry.addLine("INTAKE SUBSYSTEM");
        telemetry.addData("intakeFront Position", intakeFront.getCurrentPosition());
        telemetry.addData("intakeRear Position", intakeRear.getCurrentPosition());
        telemetry.addData("launch counter", launchCounter);
        telemetry.addData("Mid Sensor Distance", midColorSens.getDistance(DistanceUnit.CM));
        telemetry.addData("Mid Color Detected", colorDetected(midColorSens));
        telemetry.addData("Launcher Beam Break", !launcherBeamBreak.getState());
        telemetry.addData("Rear Beam Break New", !rearBeamBreak.getState());
        telemetry.addData("Front Beam Break", !frontBeamBreak.getState());
        telemetry.update();
    }

    public MatchSettings getMatchSettings() {
        return matchSettings;
    }

    /**
     * ============== AUTONOMOUS ACTIONS ==============
     **/

    // Automatically intakes 3 artifacts from front. Times out after 5 seconds
    public class AutoIntake3Front implements Action {
        //check if initialized
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on intake, if it is not on
            if (!initialized) { // turn on all front intaking servos
                MatchSettings.intakeState = MatchSettings.IntakeState.INTAKING_FRONT;
                clearIntakeLoadColors();
                clearIntakePossessions();
                inboundFront();
                inboundTransfer();
                timer.reset();
                midTransferred = false;
                initialized = true; //so that it skips this part next rerun
            }

            if (timer.time() > 2) { //stop intakes if it's been intaking longer than ## seconds
                stopIntake();
                stopTransfer();
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false;
            }

            if (!possessionRear) { //check for rear possession
                if (!midTransferred) {
                    if (midPossession()) {
                        // Check for color passing through Mid
                        if (colorInSlotRear == MatchSettings.ArtifactColor.UNKNOWN) {
                            colorInSlotRear = colorDetected(midColorSens);
                        }
                        holdArtifactRear();
                        midTransferred = true;
                    }
                }
                if (rearPossession()) {
                    possessionRear = true;
                }
                return true;  // rerun if sensor doesn't read possession
            }

            if (possessionRear && !possessionMid) { //check for mid possession
                if (midPossession()) {
                    possessionMid = true;
                    colorInSlotMid = colorDetected(midColorSens);
                }
                return true; //rerun if sensor doesn't read anything
            }

            if (possessionMid && !possessionFront) { // check for front possession
                if (frontPossession()) {
                    possessionFront = true;
                    stopIntake(); //otherwise, turn off motors
                    stopTransfer();
                    MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                    return false;
                }
            }

            return false; // rerun if sensors don't read anything
        }
    }

    public Action autoIntake3Front() {
        return new AutoIntake3Front();
    }

    //---------------------------------------------------------------

    //temp for auto before the launcher is final - spits out balls from the intake for 3 seconds
    public class AutoSpitOut implements Action {
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                inboundFront();
                outboundRear();
                inboundTransfer();
                timer.reset();
                initialized = true;
                return true;
            } else if (timer.seconds() < 2) {
                return true;
            } else {
                stopIntake();
                stopTransfer();
                return false;
            }
        }
    }

    public Action autoSpitOut() {
        return new AutoSpitOut();
    }

    //---------------------------------------------------------------

    // Launches 3 artifacts by time
    public class AutoLaunch3Fast implements Action {
        // checks if the action has been initialized
        private boolean initialized = false;
        private ElapsedTime timerAction = new ElapsedTime();

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            telemetry.addLine("launching 3 by Timer");
            telemetry.update();

            if (!initialized && autoTimer.seconds() < autoCancelSeconds) {
                MatchSettings.intakeState = MatchSettings.IntakeState.LAUNCHING_3_FAST;

                timerAction.reset();
                // turn on servos to feed whatever up
                inboundFront();
                inboundRear();
                outboundTransfer();
                initialized = true; //so that it skips this part next rerun
                return true;
            } else if (timerAction.seconds() > 3 || autoTimer.seconds() >= autoCancelSeconds) {
                stopIntake();
                stopTransfer();
                MatchSettings.intakeState = MatchSettings.IntakeState.STOPPED;
                return false; //if the whole thing is taking too long
            } else {
                return true;
            }
        }
    }
    public Action autoLaunch3Fast() {
        return new AutoLaunch3Fast();
    }

    //---------------------------------------------------------------

    //Sets slot colors based on which position on the field intaking from
    //Close to goal position
    public class AutoCloseColors implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            colorInSlotFront = MatchSettings.ArtifactColor.GREEN;
            colorInSlotMid = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotRear = MatchSettings.ArtifactColor.PURPLE;
            return false;
        }
    }public Action autoCloseColors() { return new AutoCloseColors(); }

    //---------------------------------------------------------------

    //Sets slot colors based on which position on the field intaking from
    //Close to goal position
    public class AutoMidColors implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            colorInSlotFront = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotMid = MatchSettings.ArtifactColor.GREEN;
            colorInSlotRear = MatchSettings.ArtifactColor.PURPLE;
            return false;
        }
    }public Action autoMidColors() { return new AutoMidColors(); }

    //Sets slot colors based on which position on the field intaking from
    //Close to goal position
    public class AutoFarColors implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            colorInSlotFront = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotMid = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotRear = MatchSettings.ArtifactColor.GREEN;
            return false;
        }
    }public Action autoFarColors() { return new AutoFarColors(); }

    //---------------------------------------------------------------

    //Sets slot colors based on which position on the field intaking from
    //Close to goal position
    public class AutoWallColors implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            colorInSlotFront = MatchSettings.ArtifactColor.PURPLE;
            colorInSlotMid = MatchSettings.ArtifactColor.GREEN;
            colorInSlotRear = MatchSettings.ArtifactColor.PURPLE;
            return false;
        }
    }public Action autoWallColors() { return new AutoWallColors(); }

    //---------------------------------------------------------------

    //Resets Auton Timer at the start of Auto - so that launching can be canceled

    public class AutoResetAutoTimer implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            autoTimer.reset();
            return false;
        }
    }public Action autoResetAutoTimer() { return new AutoResetAutoTimer(); }
}
