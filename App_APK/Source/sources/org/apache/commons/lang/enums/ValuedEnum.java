package org.apache.commons.lang.enums;

import java.lang.reflect.InvocationTargetException;
import org.apache.commons.lang.ClassUtils;

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
        if (other == this) {
            return 0;
        }
        if (other.getClass() == getClass()) {
            return this.iValue - ((ValuedEnum) other).iValue;
        }
        if (other.getClass().getName().equals(getClass().getName())) {
            return this.iValue - getValueInOtherClassLoader(other);
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Different enum class '");
        stringBuffer.append(ClassUtils.getShortClassName((Class) other.getClass()));
        stringBuffer.append("'");
        throw new ClassCastException(stringBuffer.toString());
    }

    private int getValueInOtherClassLoader(Object other) {
        try {
            return ((Integer) other.getClass().getMethod("getValue", (Class[]) null).invoke(other, (Object[]) null)).intValue();
        } catch (IllegalAccessException | NoSuchMethodException | InvocationTargetException e) {
            throw new IllegalStateException("This should not happen");
        }
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
