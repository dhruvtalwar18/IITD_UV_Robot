package com.github.rosjava.android_remocons.common_tools.apps;

import java.util.LinkedHashMap;

public class AppParameters extends LinkedHashMap<String, Object> {
    public Object get(String key, Object def) {
        return super.containsKey(key) ? super.get(key) : def;
    }
}
