package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MatchSettings;
import org.opencv.core.Mat;

public class IntakeSubsystem extends SubsystemBase {

    // ================== SERVOS ================== \\
    private final CRServo frontIntake;
    private final CRServo frontMidIntake;
    private final CRServo rearMidIntake;
    private final CRServo rearIntake;
    private final CRServo leftTransfer;
    private final CRServo rightTransfer;

    // ================== SENSORS ================== \\
    private final RevColorSensorV3 frontColorSens;
    private final RevColorSensorV3 midColorSens;
    private final RevColorSensorV3 rearColorSens;

    double possessionDistance = Constants.intakeConstants.DISTANCE_FOR_POSSESSION;
    double intaking = Constants.intakeConstants.INTAKE_POWER;
    double outtaking = Constants.intakeConstants.REVERSE_INTAKE_POWER;

    MatchSettings.ArtifactColor colorInSlot = MatchSettings.ArtifactColor.UNKNOWN;
    boolean possession = false; // Variable telling whether we have possession of a game piece or not

    // Possession and Color Variables for each intake round. Remember to reset!
    boolean possessionFront = false;
    boolean possessionMid = false;
    boolean possessionRear = false;

    String secondArtifactLaunched = "TBD";

    MatchSettings.ArtifactColor colorInSlotFront = MatchSettings.ArtifactColor.UNKNOWN;
    MatchSettings.ArtifactColor colorInSlotMid = MatchSettings.ArtifactColor.UNKNOWN;
    MatchSettings.ArtifactColor colorInSlotRear = MatchSettings.ArtifactColor.UNKNOWN;

    private static final double HSV_GREEN_MIN_H = 60;
    private static final double HSV_GREEN_MAX_H = 180;
    private static final double HSV_PURPLE_MIN_H = 250;
    private static final double HSV_PURPLE_MAX_H = 320;
    private static final double HSV_MIN_SAT = 0.3;
    private static final double HSV_MIN_VAL = 0.1;

    //=========== TELEMETRY ===========\\
    private final Telemetry telemetry;

    // CREATE MATCH SETTINGS / MOTIF ==============\\
    MatchSettings.Motif motif;

    public final MatchSettings matchSettings;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, MatchSettings matchSettings) {

        this.telemetry = telemetry;
        this.matchSettings = matchSettings;

        // ================== SERVOS ================== \\
        frontIntake = hardwareMap.get(CRServo.class, "frontIntake");
        frontMidIntake = hardwareMap.get(CRServo.class, "frontMidIntake");
        rearMidIntake = hardwareMap.get(CRServo.class, "rearMidIntake");
        rearIntake = hardwareMap.get(CRServo.class, "rearIntake");
        leftTransfer = hardwareMap.get(CRServo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(CRServo.class, "rightTransfer");

        frontIntake.setDirection(CRServo.Direction.REVERSE);
        frontMidIntake.setDirection(CRServo.Direction.REVERSE);
        rearMidIntake.setDirection(CRServo.Direction.FORWARD);
        rearIntake.setDirection(CRServo.Direction.FORWARD);
        leftTransfer.setDirection(CRServo.Direction.FORWARD);
        rightTransfer.setDirection(CRServo.Direction.FORWARD);

        // ================== SENSORS ================== \\
        frontColorSens = hardwareMap.get(RevColorSensorV3.class, "frontColor");
        midColorSens = hardwareMap.get(RevColorSensorV3.class, "midColor");
        rearColorSens = hardwareMap.get(RevColorSensorV3.class, "rearColor");

    }

    //============== CONTROL METHODS ==============\\

    // COLOR SENSOR METHODS ==========================\\

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

    public void readIntakeLoadColors() { // bulk read all sensors and set initial color variables prior to launching
        colorInSlotFront = colorDetected(frontColorSens);
        colorInSlotMid = colorDetected(midColorSens);
        colorInSlotRear = colorDetected(rearColorSens);
    }

    public void clearIntakeLoadColors() {
        colorInSlotFront = MatchSettings.ArtifactColor.UNKNOWN;
        colorInSlotMid = MatchSettings.ArtifactColor.UNKNOWN;
        colorInSlotRear = MatchSettings.ArtifactColor.UNKNOWN;
    }

    // DISTANCE SENSOR METHODS ==========================\\

    private boolean possessionDetected(RevColorSensorV3 sensor) { //for bulk read before launching
        possession = sensor.getDistance(DistanceUnit.CM) < possessionDistance;
        return possession;
    }

    public void readIntakePossessions() { // bulk read all sensors for initial possession values prior to launching
        possessionFront = possessionDetected(frontColorSens);
        possessionMid = possessionDetected(midColorSens);
        possessionRear = possessionDetected(rearColorSens);
    }

    public void clearIntakePossessions() { // clear Intake load possession values
        possessionFront = false;
        possessionMid = false;
        possessionRear = false;
    }

    public boolean frontPossession() { // returns true if there is an artifact in distance
        possession = frontColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return possession;
    }

    public boolean midPossession() { // returns true if there is an artifact in distance
        possession = midColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return possession;
    }

    public boolean rearPossession() { // returns true if there is an artifact in distance
        possession = rearColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return possession;
    }

    public boolean notFrontPossession() { // returns true if there is not an artifact in distance
        possession = frontColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return !possession;
    }

    public boolean notMidPossession() { // returns true if there is not an artifact in distance
        possession = midColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return !possession;
    }

    public boolean notRearPossession() { // returns true if there is not an artifact in distance
        possession = rearColorSens.getDistance(DistanceUnit.CM) < possessionDistance;
        return !possession;
    }

    // IN-BOUND METHODS ==========================\\
    public void inboundFront() {
        frontIntake.setPower(intaking);
    }

    public void inboundMidFront() {
        frontMidIntake.setPower(intaking);
    }

    public void inboundMidRear() {
        rearMidIntake.setPower(intaking);
    }

    public void inboundRear() {
        rearIntake.setPower(intaking);
    }

    public void inboundTransfer() {
        leftTransfer.setPower(intaking);
        rightTransfer.setPower(intaking);
    }

    public void thruFrontAll() {
        frontIntake.setPower(intaking);
        frontMidIntake.setPower(intaking);
        rearMidIntake.setPower(outtaking);
        rearIntake.setPower(outtaking);
    }

    // OUT-BOUND METHODS ==========================\\
    public void outboundFront() {
        frontIntake.setPower(outtaking);
    }

    public void outboundMidFront() {
        frontMidIntake.setPower(outtaking);
    }

    public void outboundMidRear() {
        rearMidIntake.setPower(outtaking);
    }

    public void outboundRear() {
        rearIntake.setPower(outtaking);
    }

    public void outboundTransfer() {
        leftTransfer.setPower(outtaking);
        rightTransfer.setPower(outtaking);
    }

    // STOPPING METHODS ==========================\\
    public void stopFront() {
        frontIntake.setPower(0);
    }

    public void stopMidFront() {
        frontMidIntake.setPower(0);
    }

    public void stopMidRear() {
        rearMidIntake.setPower(0);
    }

    public void stopRear() {
        rearIntake.setPower(0);
    }

    public void stopTransfer() {
        leftTransfer.setPower(0);
        rightTransfer.setPower(0);
    }

    public void stopAll() {
        frontIntake.setPower(0);
        frontMidIntake.setPower(0);
        rearMidIntake.setPower(0);
        rearIntake.setPower(0);
    }

    // TRANSFER TO LAUNCHER METHODS ==========================\\


    public void printTelemetry(Telemetry telemetry) {
        telemetry.addLine("INTAKE SUBSYSTEM");
        telemetry.addData("Front Sensor Distance", frontColorSens.getDistance(DistanceUnit.CM));
        telemetry.addData("Mid Sensor Distance", midColorSens.getDistance(DistanceUnit.CM));
        telemetry.addData("Rear Sensor Distance", rearColorSens.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    /**============== AUTONOMOUS ACTIONS ==============**/

    // Automatically intakes 3 artifacts. Times out after 5 seconds
    public class AutoIntake3Front implements Action {
        //check if initialized
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on intake, if it is not on
            if (!initialized) { // turn on all front intaking servos
                inboundFront();
                inboundMidFront();
                outboundMidRear();
                timer.reset();
                initialized = true; //so that it skips this part next rerun
            }

            if (timer.time() > 5) { //stop intakes if it's been intaking longer than 5 seconds
                stopAll();
                return false;
            }

            if (!rearPossession()) { //check for rear possession
                return true;  // rerun if sensor doesn't read anything
            } else {
                stopMidRear(); //otherwise stop checking, stop servo, and move on
            }

            if (!midPossession()) { //check for mid possession
                return true; //rerun if sensor doesn't read anything
            } else {
                stopMidFront(); //otherwise stop checking, stop servo, and move on
            }

            if (!frontPossession()) { // check for front possession
                return true; // rerun in sensor doesn't read anything
            } else {
                stopFront(); //otherwise, turn off servo
                return false; //and return since action is complete
            }
        }
    }
    public Action autoIntake3Front() { return new AutoIntake3Front(); }

    //---------------------------------------------------------------

    // Bulk reads for initial possession and colors. Launches Mid Artifact if there is one.
    public class AutoLaunch1st implements Action {
        // checks if the action has been initialized
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(!initialized) {
                // start by clearing possession/color values and then bulk read
                // for initial values for this intake load
                clearIntakePossessions();
                clearIntakeLoadColors();
                readIntakePossessions();
                readIntakeLoadColors();
                secondArtifactLaunched = "TBD";

                // check for possession in midSlot and skip the rest of the action if nothing there
                if (!possessionMid) {
                    return false;
                }
                timer.reset();
                inboundMidFront();
                inboundMidRear();
                outboundTransfer();
                initialized = true; //so that it skips this part next rerun
            }
            if(timer.time() < 0.5) {
                return true;
            } else {
                stopAll();
            }
            if(timer.time() < 1) {
                return true;
            } else {
                stopTransfer();
                return false;
            }
        }
    }
    public Action autoLaunch1st() { return new AutoLaunch1st(); }

    //---------------------------------------------------------------

    // Launches a 2nd artifact. Chooses between front and rear depending on Motif
    public class AutoLaunch2nd implements Action {
        // checks if the action has been initialized
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();
        private String launchSlot = "TBD";
        private MatchSettings.Motif motif = null;
        private MatchSettings.ArtifactColor desiredColor = MatchSettings.ArtifactColor.UNKNOWN;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(!initialized) {
                if(!possessionFront && !possessionRear) { // check to see if there's an Artifact to launch
                    return false;
                }
                if (!possessionMid) { // set desired color, 2nd motif slot unless there was no mid artifact
                    desiredColor = matchSettings.firstArtifactNeeded();
                } else {
                    desiredColor = matchSettings.secondArtifactNeeded();
                }
                if (possessionRear && colorInSlotRear == desiredColor) { // choose a launch slot
                    launchSlot = "Rear";
                } else if (possessionFront && colorInSlotFront == desiredColor) {
                    launchSlot = "Front";
                } else if (possessionRear) {
                    launchSlot = "Rear";
                } else {
                    launchSlot = "Front";
                }
                secondArtifactLaunched = launchSlot;
                timer.reset();
                // turn on front or rear servos depending on which artifact to be launched
                if (launchSlot == "Rear") {
                    inboundMidRear();
                    inboundRear();
                } else {
                    inboundFront();
                    inboundMidFront();
                }
                initialized = true; //so that it skips this part next rerun
            }
            if(timer.time() < 0.5) {
                return true;
            }
            if (launchSlot == "Rear"){
                stopRear();
                inboundMidFront();
            } else {
                stopFront();
                inboundMidRear();
            }
            outboundTransfer();
            if(timer.time() < 1) {
                return true;
            } else {
                stopTransfer();
                stopAll();
                return false;
            }
        }
    }
    public Action autoLaunch2nd() { return new AutoLaunch2nd(); }

    //---------------------------------------------------------------

    // Launches a 3rd artifact. Chooses between front and rear depending on where remaining Artifact is
    public class AutoLaunch3rd implements Action {
        // checks if the action has been initialized
        private boolean initialized = false;
        private ElapsedTime timer = new ElapsedTime();
        private String launchSlot = "TBD";

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if(!initialized) {
                if(!possessionFront && !possessionRear) { // check to see if there's an Artifact to launch
                    return false;
                }
                if (possessionRear && secondArtifactLaunched == "Front") {
                    launchSlot = "Rear";
                } else if (possessionFront && secondArtifactLaunched == "Rear"){
                    launchSlot = "Front";
                } else {
                    return false;
                }
                timer.reset();
                // turn on front or rear servos depending on which artifact to be launched
                if (launchSlot == "Rear") {
                    inboundMidRear();
                    inboundRear();
                } else {
                    inboundFront();
                    inboundMidFront();
                }
                initialized = true; //so that it skips this part next rerun
            }
            if(timer.time() < 0.5) {
                return true;
            }
            if (launchSlot == "Rear"){
                stopRear();
                inboundMidFront();
            } else {
                stopFront();
                inboundMidRear();
            }
            outboundTransfer();
            if(timer.time() < 1) {
                return true;
            } else {
                stopTransfer();
                stopAll();
                return false;
            }
        }
    }
    public Action autoLaunch3rd() { return new AutoLaunch3rd(); }

}
