package org.jboss.netty.util.internal;

import java.util.regex.Pattern;

public final class SystemPropertyUtil {
    public static String get(String key) {
        try {
            return System.getProperty(key);
        } catch (Exception e) {
            return null;
        }
    }

    public static String get(String key, String def) {
        String value = get(key);
        if (value == null) {
            return def;
        }
        return value;
    }

    public static int get(String key, int def) {
        String value = get(key);
        if (value != null && Pattern.matches("-?[0-9]+", value)) {
            return Integer.parseInt(value);
        }
        return def;
    }

    private SystemPropertyUtil() {
    }
}
