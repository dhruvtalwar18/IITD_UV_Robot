package org.apache.commons.lang.builder;

import java.lang.reflect.AccessibleObject;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.math.BigDecimal;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

public class EqualsBuilder {
    private boolean isEquals = true;

    public static boolean reflectionEquals(Object lhs, Object rhs) {
        return reflectionEquals(lhs, rhs, false, (Class) null, (String[]) null);
    }

    public static boolean reflectionEquals(Object lhs, Object rhs, Collection excludeFields) {
        return reflectionEquals(lhs, rhs, ReflectionToStringBuilder.toNoNullStringArray(excludeFields));
    }

    public static boolean reflectionEquals(Object lhs, Object rhs, String[] excludeFields) {
        return reflectionEquals(lhs, rhs, false, (Class) null, excludeFields);
    }

    public static boolean reflectionEquals(Object lhs, Object rhs, boolean testTransients) {
        return reflectionEquals(lhs, rhs, testTransients, (Class) null, (String[]) null);
    }

    public static boolean reflectionEquals(Object lhs, Object rhs, boolean testTransients, Class reflectUpToClass) {
        return reflectionEquals(lhs, rhs, testTransients, reflectUpToClass, (String[]) null);
    }

    public static boolean reflectionEquals(Object lhs, Object rhs, boolean testTransients, Class reflectUpToClass, String[] excludeFields) {
        Class testClass;
        Object obj = lhs;
        Object obj2 = rhs;
        if (obj == obj2) {
            return true;
        }
        if (obj == null || obj2 == null) {
            Class cls = reflectUpToClass;
            return false;
        }
        Class lhsClass = lhs.getClass();
        Class rhsClass = rhs.getClass();
        if (lhsClass.isInstance(obj2)) {
            testClass = lhsClass;
            if (!rhsClass.isInstance(obj)) {
                testClass = rhsClass;
            }
        } else if (rhsClass.isInstance(obj)) {
            testClass = rhsClass;
            if (!lhsClass.isInstance(obj2)) {
                testClass = lhsClass;
            }
        } else {
            Class cls2 = reflectUpToClass;
            return false;
        }
        EqualsBuilder equalsBuilder = new EqualsBuilder();
        try {
            reflectionAppend(lhs, rhs, testClass, equalsBuilder, testTransients, excludeFields);
            while (true) {
                if (testClass.getSuperclass() != null) {
                    if (testClass == reflectUpToClass) {
                        break;
                    }
                    try {
                        Class testClass2 = testClass.getSuperclass();
                        reflectionAppend(lhs, rhs, testClass2, equalsBuilder, testTransients, excludeFields);
                        testClass = testClass2;
                    } catch (IllegalArgumentException e) {
                        return false;
                    }
                } else {
                    Class cls3 = reflectUpToClass;
                    break;
                }
            }
            return equalsBuilder.isEquals();
        } catch (IllegalArgumentException e2) {
            Class cls4 = reflectUpToClass;
            return false;
        }
    }

    private static void reflectionAppend(Object lhs, Object rhs, Class clazz, EqualsBuilder builder, boolean useTransients, String[] excludeFields) {
        Field[] fields = clazz.getDeclaredFields();
        List excludedFieldList = excludeFields != null ? Arrays.asList(excludeFields) : Collections.EMPTY_LIST;
        AccessibleObject.setAccessible(fields, true);
        for (int i = 0; i < fields.length && builder.isEquals; i++) {
            Field f = fields[i];
            if (!excludedFieldList.contains(f.getName()) && f.getName().indexOf(36) == -1 && ((useTransients || !Modifier.isTransient(f.getModifiers())) && !Modifier.isStatic(f.getModifiers()))) {
                try {
                    builder.append(f.get(lhs), f.get(rhs));
                } catch (IllegalAccessException e) {
                    throw new InternalError("Unexpected IllegalAccessException");
                }
            }
        }
    }

    public EqualsBuilder appendSuper(boolean superEquals) {
        if (!this.isEquals) {
            return this;
        }
        this.isEquals = superEquals;
        return this;
    }

    public EqualsBuilder append(Object lhs, Object rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        boolean z = false;
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        }
        if (!lhs.getClass().isArray()) {
            if (lhs instanceof BigDecimal) {
                if (((BigDecimal) lhs).compareTo(rhs) == 0) {
                    z = true;
                }
                this.isEquals = z;
            } else {
                this.isEquals = lhs.equals(rhs);
            }
        } else if (lhs.getClass() != rhs.getClass()) {
            setEquals(false);
        } else if (lhs instanceof long[]) {
            append((long[]) lhs, (long[]) rhs);
        } else if (lhs instanceof int[]) {
            append((int[]) lhs, (int[]) rhs);
        } else if (lhs instanceof short[]) {
            append((short[]) lhs, (short[]) rhs);
        } else if (lhs instanceof char[]) {
            append((char[]) lhs, (char[]) rhs);
        } else if (lhs instanceof byte[]) {
            append((byte[]) lhs, (byte[]) rhs);
        } else if (lhs instanceof double[]) {
            append((double[]) lhs, (double[]) rhs);
        } else if (lhs instanceof float[]) {
            append((float[]) lhs, (float[]) rhs);
        } else if (lhs instanceof boolean[]) {
            append((boolean[]) lhs, (boolean[]) rhs);
        } else {
            append((Object[]) lhs, (Object[]) rhs);
        }
        return this;
    }

    public EqualsBuilder append(long lhs, long rhs) {
        if (!this.isEquals) {
            return this;
        }
        this.isEquals = lhs == rhs;
        return this;
    }

    public EqualsBuilder append(int lhs, int rhs) {
        if (!this.isEquals) {
            return this;
        }
        this.isEquals = lhs == rhs;
        return this;
    }

    public EqualsBuilder append(short lhs, short rhs) {
        if (!this.isEquals) {
            return this;
        }
        this.isEquals = lhs == rhs;
        return this;
    }

    public EqualsBuilder append(char lhs, char rhs) {
        if (!this.isEquals) {
            return this;
        }
        this.isEquals = lhs == rhs;
        return this;
    }

    public EqualsBuilder append(byte lhs, byte rhs) {
        if (!this.isEquals) {
            return this;
        }
        this.isEquals = lhs == rhs;
        return this;
    }

    public EqualsBuilder append(double lhs, double rhs) {
        if (!this.isEquals) {
            return this;
        }
        return append(Double.doubleToLongBits(lhs), Double.doubleToLongBits(rhs));
    }

    public EqualsBuilder append(float lhs, float rhs) {
        if (!this.isEquals) {
            return this;
        }
        return append(Float.floatToIntBits(lhs), Float.floatToIntBits(rhs));
    }

    public EqualsBuilder append(boolean lhs, boolean rhs) {
        if (!this.isEquals) {
            return this;
        }
        this.isEquals = lhs == rhs;
        return this;
    }

    public EqualsBuilder append(Object[] lhs, Object[] rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        } else if (lhs.length != rhs.length) {
            setEquals(false);
            return this;
        } else {
            for (int i = 0; i < lhs.length && this.isEquals; i++) {
                append(lhs[i], rhs[i]);
            }
            return this;
        }
    }

    public EqualsBuilder append(long[] lhs, long[] rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        } else if (lhs.length != rhs.length) {
            setEquals(false);
            return this;
        } else {
            for (int i = 0; i < lhs.length && this.isEquals; i++) {
                append(lhs[i], rhs[i]);
            }
            return this;
        }
    }

    public EqualsBuilder append(int[] lhs, int[] rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        } else if (lhs.length != rhs.length) {
            setEquals(false);
            return this;
        } else {
            for (int i = 0; i < lhs.length && this.isEquals; i++) {
                append(lhs[i], rhs[i]);
            }
            return this;
        }
    }

    public EqualsBuilder append(short[] lhs, short[] rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        } else if (lhs.length != rhs.length) {
            setEquals(false);
            return this;
        } else {
            for (int i = 0; i < lhs.length && this.isEquals; i++) {
                append(lhs[i], rhs[i]);
            }
            return this;
        }
    }

    public EqualsBuilder append(char[] lhs, char[] rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        } else if (lhs.length != rhs.length) {
            setEquals(false);
            return this;
        } else {
            for (int i = 0; i < lhs.length && this.isEquals; i++) {
                append(lhs[i], rhs[i]);
            }
            return this;
        }
    }

    public EqualsBuilder append(byte[] lhs, byte[] rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        } else if (lhs.length != rhs.length) {
            setEquals(false);
            return this;
        } else {
            for (int i = 0; i < lhs.length && this.isEquals; i++) {
                append(lhs[i], rhs[i]);
            }
            return this;
        }
    }

    public EqualsBuilder append(double[] lhs, double[] rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        } else if (lhs.length != rhs.length) {
            setEquals(false);
            return this;
        } else {
            for (int i = 0; i < lhs.length && this.isEquals; i++) {
                append(lhs[i], rhs[i]);
            }
            return this;
        }
    }

    public EqualsBuilder append(float[] lhs, float[] rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        } else if (lhs.length != rhs.length) {
            setEquals(false);
            return this;
        } else {
            for (int i = 0; i < lhs.length && this.isEquals; i++) {
                append(lhs[i], rhs[i]);
            }
            return this;
        }
    }

    public EqualsBuilder append(boolean[] lhs, boolean[] rhs) {
        if (!this.isEquals || lhs == rhs) {
            return this;
        }
        if (lhs == null || rhs == null) {
            setEquals(false);
            return this;
        } else if (lhs.length != rhs.length) {
            setEquals(false);
            return this;
        } else {
            for (int i = 0; i < lhs.length && this.isEquals; i++) {
                append(lhs[i], rhs[i]);
            }
            return this;
        }
    }

    public boolean isEquals() {
        return this.isEquals;
    }

    /* access modifiers changed from: protected */
    public void setEquals(boolean isEquals2) {
        this.isEquals = isEquals2;
    }
}
