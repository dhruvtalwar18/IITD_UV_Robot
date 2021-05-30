package org.apache.commons.lang.text;

import java.util.Map;

public abstract class StrLookup {
    private static final StrLookup NONE_LOOKUP = new MapStrLookup((Map) null);
    private static final StrLookup SYSTEM_PROPERTIES_LOOKUP;

    public abstract String lookup(String str);

    static {
        StrLookup lookup;
        try {
            lookup = new MapStrLookup(System.getProperties());
        } catch (SecurityException e) {
            lookup = NONE_LOOKUP;
        }
        SYSTEM_PROPERTIES_LOOKUP = lookup;
    }

    public static StrLookup noneLookup() {
        return NONE_LOOKUP;
    }

    public static StrLookup systemPropertiesLookup() {
        return SYSTEM_PROPERTIES_LOOKUP;
    }

    public static StrLookup mapLookup(Map map) {
        return new MapStrLookup(map);
    }

    protected StrLookup() {
    }

    static class MapStrLookup extends StrLookup {
        private final Map map;

        MapStrLookup(Map map2) {
            this.map = map2;
        }

        public String lookup(String key) {
            Object obj;
            if (this.map == null || (obj = this.map.get(key)) == null) {
                return null;
            }
            return obj.toString();
        }
    }
}
