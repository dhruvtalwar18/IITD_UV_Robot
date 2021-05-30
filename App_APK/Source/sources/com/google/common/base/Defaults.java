package com.google.common.base;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import org.bytedeco.javacpp.opencv_stitching;

public final class Defaults {
    private static final Map<Class<?>, Object> DEFAULTS;

    private Defaults() {
    }

    static {
        Map<Class<?>, Object> map = new HashMap<>();
        put(map, Boolean.TYPE, false);
        put(map, Character.TYPE, 0);
        put(map, Byte.TYPE, (byte) 0);
        put(map, Short.TYPE, (short) 0);
        put(map, Integer.TYPE, 0);
        put(map, Long.TYPE, 0L);
        put(map, Float.TYPE, Float.valueOf(0.0f));
        put(map, Double.TYPE, Double.valueOf(opencv_stitching.Stitcher.ORIG_RESOL));
        DEFAULTS = Collections.unmodifiableMap(map);
    }

    private static <T> void put(Map<Class<?>, Object> map, Class<T> type, T value) {
        map.put(type, value);
    }

    public static <T> T defaultValue(Class<T> type) {
        return DEFAULTS.get(type);
    }
}
