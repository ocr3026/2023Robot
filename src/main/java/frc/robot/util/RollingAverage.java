package frc.robot.util;

import java.util.LinkedList;
import java.util.Queue;

public class RollingAverage {
    int listLength = 1;
    double rollingSum = 0;

    Queue<Double> queue = new LinkedList<Double>();

    public RollingAverage(int length) {
        listLength = length;
    }

    public void add(double value) {
        queue.add(value);
        rollingSum += value;
        if (queue.size() > listLength) {
            rollingSum -= queue.remove();
        }
    }

    public double average() {
        return rollingSum / queue.size();
    }
}
