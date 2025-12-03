package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;

public final class Constants {

    @Config
    public static final class DriveConstants{

    }

    @Config
    public static final class FieldConstants{

    }
    @Config
    public static final class intakeConstants{
        public static double INTAKE_POWER = 1;
        public static double REVERSE_INTAKE_POWER = -1;
        public static double DISTANCE_FOR_POSSESSION_FRONT = 4.75; //TUNE!!!!! max range is 10
        public static double DISTANCE_FOR_POSSESSION_MID = 2.5;
        public static double DISTANCE_FOR_POSSESSION_REAR = 5.75; //TUNE!!!!! max range is 10
    }

    @Config
    public static final class launcherConstants{
        public static double FAR_ZONE_LAUNCH_RPM = 1950;
        public static double MID_ZONE_LAUNCH_RPM = 1900;
        public static double CLOSE_ZONE_LAUNCH_RPM = 1950;
    }

    @Config
    public static class Util {
        public static double round(double in, int places) {
            return ((int) (in * Math.pow(10, places))) / (double) Math.pow(10, places);
        }

        public static double angleToBlueGoal(double x, double y) { //in Radians
            double dx = -72 - x;
            double dy = -72 - y;
            return Math.atan2(dy, dx);
        }

        public static double angleToRedGoal(double x, double y) { //in Radians
            double dx = -72 - x;
            double dy = 72 - y;
            return Math.atan2(dy, dx);
        }
    }

    public static class ColorSensor {

        public static double[] GREEN_TARGET = {70, 200, 150};
        public static double[] PURPLE_TARGET = {120, 150, 220};
        public static double CONFIDENCE_THRESHOLD = 60.0; // Acceptable distance threshold
    }


}
