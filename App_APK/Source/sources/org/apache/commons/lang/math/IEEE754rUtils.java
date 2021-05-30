package org.apache.commons.lang.math;

public class IEEE754rUtils {
    public static double min(double[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            double min = array[0];
            for (int i = 1; i < array.length; i++) {
                min = min(array[i], min);
            }
            return min;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static float min(float[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            float min = array[0];
            for (int i = 1; i < array.length; i++) {
                min = min(array[i], min);
            }
            return min;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static double min(double a, double b, double c) {
        return min(min(a, b), c);
    }

    public static double min(double a, double b) {
        if (Double.isNaN(a)) {
            return b;
        }
        if (Double.isNaN(b)) {
            return a;
        }
        return Math.min(a, b);
    }

    public static float min(float a, float b, float c) {
        return min(min(a, b), c);
    }

    public static float min(float a, float b) {
        if (Float.isNaN(a)) {
            return b;
        }
        if (Float.isNaN(b)) {
            return a;
        }
        return Math.min(a, b);
    }

    public static double max(double[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            double max = array[0];
            for (int j = 1; j < array.length; j++) {
                max = max(array[j], max);
            }
            return max;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static float max(float[] array) {
        if (array == null) {
            throw new IllegalArgumentException("The Array must not be null");
        } else if (array.length != 0) {
            float max = array[0];
            for (int j = 1; j < array.length; j++) {
                max = max(array[j], max);
            }
            return max;
        } else {
            throw new IllegalArgumentException("Array cannot be empty.");
        }
    }

    public static double max(double a, double b, double c) {
        return max(max(a, b), c);
    }

    public static double max(double a, double b) {
        if (Double.isNaN(a)) {
            return b;
        }
        if (Double.isNaN(b)) {
            return a;
        }
        return Math.max(a, b);
    }

    public static float max(float a, float b, float c) {
        return max(max(a, b), c);
    }

    public static float max(float a, float b) {
        if (Float.isNaN(a)) {
            return b;
        }
        if (Float.isNaN(b)) {
            return a;
        }
        return Math.max(a, b);
    }
}
