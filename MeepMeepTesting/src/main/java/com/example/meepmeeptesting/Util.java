package com.example.meepmeeptesting;

public final class Util {

    public static double angleToBlueGoalDegrees(double x, double y) { //in Degrees
        double dx = -72 - x;
        double dy = -72 - y;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public static double angleToRedGoalDegrees(double x, double y) { //in Degrees
        double dx = -72 - x;
        double dy = 72 - y;
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public static double angleToMotifDegrees(double x, double y) { //in Degrees
        double dx = -72 - x;
        double dy = 0 - y;
        return Math.toDegrees(Math.atan2(dy, dx));
    }
}
