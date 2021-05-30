package com.github.rosjava.android_remocons.common_tools.apps;

import java.util.LinkedHashMap;

public class AppRemappings extends LinkedHashMap<String, String> {
    public String get(String from) {
        return super.containsKey(from) ? (String) super.get(from) : from;
    }

    public String get(String from, String to) {
        return super.containsKey(from) ? (String) super.get(from) : to;
    }
}
