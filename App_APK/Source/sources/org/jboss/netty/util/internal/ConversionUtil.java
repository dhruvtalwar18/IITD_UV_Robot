package org.jboss.netty.util.internal;

import java.util.ArrayList;
import java.util.List;

public final class ConversionUtil {
    private static final String[] INTEGERS = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15"};

    public static int toInt(Object value) {
        if (value instanceof Number) {
            return ((Number) value).intValue();
        }
        return Integer.parseInt(String.valueOf(value));
    }

    public static boolean toBoolean(Object value) {
        if (value instanceof Boolean) {
            return ((Boolean) value).booleanValue();
        }
        if (!(value instanceof Number)) {
            String s = String.valueOf(value);
            if (s.length() == 0) {
                return false;
            }
            try {
                if (Integer.parseInt(s) != 0) {
                    return true;
                }
                return false;
            } catch (NumberFormatException e) {
                char upperCase = Character.toUpperCase(s.charAt(0));
                return upperCase == 'T' || upperCase == 'Y';
            }
        } else if (((Number) value).intValue() != 0) {
            return true;
        } else {
            return false;
        }
    }

    public static String[] toStringArray(Object value) {
        if (value instanceof String[]) {
            return (String[]) value;
        }
        if (!(value instanceof Iterable)) {
            return String.valueOf(value).split("[, \\t\\n\\r\\f\\e\\a]");
        }
        List<String> answer = new ArrayList<>();
        for (Object v : (Iterable) value) {
            if (v == null) {
                answer.add((Object) null);
            } else {
                answer.add(String.valueOf(v));
            }
        }
        return (String[]) answer.toArray(new String[answer.size()]);
    }

    public static String toString(int value) {
        if (value < 0 || value >= INTEGERS.length) {
            return Integer.toString(value);
        }
        return INTEGERS[value];
    }

    private ConversionUtil() {
    }
}
