package org.apache.commons.httpclient.util;

public class LangUtils {
    public static final int HASH_OFFSET = 37;
    public static final int HASH_SEED = 17;

    private LangUtils() {
    }

    public static int hashCode(int seed, int hashcode) {
        return (seed * 37) + hashcode;
    }

    public static int hashCode(int seed, Object obj) {
        return hashCode(seed, obj != null ? obj.hashCode() : 0);
    }

    public static int hashCode(int seed, boolean b) {
        return hashCode(seed, (int) b);
    }

    public static boolean equals(Object obj1, Object obj2) {
        if (obj1 == null) {
            return obj2 == null;
        }
        return obj1.equals(obj2);
    }
}
