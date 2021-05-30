package org.apache.commons.lang.p000enum;

import java.util.Iterator;
import java.util.List;
import java.util.Map;

/* renamed from: org.apache.commons.lang.enum.EnumUtils  reason: invalid package */
public class EnumUtils {
    public static Enum getEnum(Class enumClass, String name) {
        return Enum.getEnum(enumClass, name);
    }

    public static ValuedEnum getEnum(Class enumClass, int value) {
        return (ValuedEnum) ValuedEnum.getEnum(enumClass, value);
    }

    public static Map getEnumMap(Class enumClass) {
        return Enum.getEnumMap(enumClass);
    }

    public static List getEnumList(Class enumClass) {
        return Enum.getEnumList(enumClass);
    }

    public static Iterator iterator(Class enumClass) {
        return Enum.getEnumList(enumClass).iterator();
    }
}
