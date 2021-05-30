package org.ros.math;

public class RosMath {
    private RosMath() {
    }

    public static double clamp(double value, double minmum, double maximum) {
        if (value < minmum) {
            return minmum;
        }
        if (value > maximum) {
            return maximum;
        }
        return value;
    }

    public static float clamp(float value, float minmum, float maximum) {
        if (value < minmum) {
            return minmum;
        }
        if (value > maximum) {
            return maximum;
        }
        return value;
    }

    public static int clamp(int value, int minmum, int maximum) {
        if (value < minmum) {
            return minmum;
        }
        if (value > maximum) {
            return maximum;
        }
        return value;
    }

    public static long clamp(long value, long minmum, long maximum) {
        if (value < minmum) {
            return minmum;
        }
        if (value > maximum) {
            return maximum;
        }
        return value;
    }
}
