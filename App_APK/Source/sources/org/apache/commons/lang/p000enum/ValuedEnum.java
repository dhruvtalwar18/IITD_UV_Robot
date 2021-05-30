package org.apache.commons.lang.p000enum;

import org.apache.commons.lang.ClassUtils;

/* renamed from: org.apache.commons.lang.enum.ValuedEnum  reason: invalid package */
public abstract class ValuedEnum extends Enum {
    private static final long serialVersionUID = -7129650521543789085L;
    private final int iValue;

    protected ValuedEnum(String name, int value) {
        super(name);
        this.iValue = value;
    }

    protected static Enum getEnum(Class enumClass, int value) {
        if (enumClass != null) {
            for (ValuedEnum enumeration : Enum.getEnumList(enumClass)) {
                if (enumeration.getValue() == value) {
                    return enumeration;
                }
            }
            return null;
        }
        throw new IllegalArgumentException("The Enum Class must not be null");
    }

    public final int getValue() {
        return this.iValue;
    }

    public int compareTo(Object other) {
        return this.iValue - ((ValuedEnum) other).iValue;
    }

    public String toString() {
        if (this.iToString == null) {
            String shortName = ClassUtils.getShortClassName(getEnumClass());
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(shortName);
            stringBuffer.append("[");
            stringBuffer.append(getName());
            stringBuffer.append("=");
            stringBuffer.append(getValue());
            stringBuffer.append("]");
            this.iToString = stringBuffer.toString();
        }
        return this.iToString;
    }
}
