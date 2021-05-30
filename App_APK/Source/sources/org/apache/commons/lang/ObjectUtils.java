package org.apache.commons.lang;

import java.io.Serializable;

public class ObjectUtils {
    public static final Null NULL = new Null();

    public static Object defaultIfNull(Object object, Object defaultValue) {
        return object != null ? object : defaultValue;
    }

    public static boolean equals(Object object1, Object object2) {
        if (object1 == object2) {
            return true;
        }
        if (object1 == null || object2 == null) {
            return false;
        }
        return object1.equals(object2);
    }

    public static int hashCode(Object obj) {
        if (obj == null) {
            return 0;
        }
        return obj.hashCode();
    }

    public static String identityToString(Object object) {
        if (object == null) {
            return null;
        }
        StringBuffer buffer = new StringBuffer();
        identityToString(buffer, object);
        return buffer.toString();
    }

    public static void identityToString(StringBuffer buffer, Object object) {
        if (object != null) {
            buffer.append(object.getClass().getName());
            buffer.append('@');
            buffer.append(Integer.toHexString(System.identityHashCode(object)));
            return;
        }
        throw new NullPointerException("Cannot get the toString of a null identity");
    }

    public static StringBuffer appendIdentityToString(StringBuffer buffer, Object object) {
        if (object == null) {
            return null;
        }
        if (buffer == null) {
            buffer = new StringBuffer();
        }
        buffer.append(object.getClass().getName());
        buffer.append('@');
        buffer.append(Integer.toHexString(System.identityHashCode(object)));
        return buffer;
    }

    public static String toString(Object obj) {
        return obj == null ? "" : obj.toString();
    }

    public static String toString(Object obj, String nullStr) {
        return obj == null ? nullStr : obj.toString();
    }

    public static Object min(Comparable c1, Comparable c2) {
        return (c1 == null || c2 == null) ? c1 != null ? c1 : c2 : c1.compareTo(c2) < 1 ? c1 : c2;
    }

    public static Object max(Comparable c1, Comparable c2) {
        return (c1 == null || c2 == null) ? c1 != null ? c1 : c2 : c1.compareTo(c2) >= 0 ? c1 : c2;
    }

    public static class Null implements Serializable {
        private static final long serialVersionUID = 7092611880189329093L;

        Null() {
        }

        private Object readResolve() {
            return ObjectUtils.NULL;
        }
    }
}
