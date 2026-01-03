package org.firstinspires.ftc.teamcode.util;

import java.util.LinkedList;
import java.util.Queue;

public class MovingAverage {
    private Queue<Double> readings = new LinkedList<>();
    private int windowSize;
    private double sum = 0.0;

    public MovingAverage(int size) {
        this.windowSize = size;
    }

    public void addReading(double value) {
        readings.add(value);
        sum += value;
        if (readings.size() > windowSize) {
            sum -= readings.poll(); // poll() removes and returns the head of the queue
        }
    }

    public double getAverage() {
        if (readings.isEmpty()) {
            return 0.0;
        }
        return sum / readings.size();
    }

    // In your Robot TeleOp or Autonomous:
    // MovingAverage distanceFilter = new MovingAverage(5); // Averages last 5 readings
    // double currentDistance = ...; // Get sensor reading
    // distanceFilter.addReading(currentDistance);
    // double smoothedDistance = distanceFilter.getAverage();
    // // Use smoothedDistance for decisions
}
