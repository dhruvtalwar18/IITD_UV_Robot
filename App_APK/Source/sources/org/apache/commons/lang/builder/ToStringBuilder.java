package org.apache.commons.lang.builder;

import org.apache.commons.lang.BooleanUtils;
import org.apache.commons.lang.ObjectUtils;

public class ToStringBuilder {
    private static ToStringStyle defaultStyle = ToStringStyle.DEFAULT_STYLE;
    private final StringBuffer buffer;
    private final Object object;
    private final ToStringStyle style;

    public static ToStringStyle getDefaultStyle() {
        return defaultStyle;
    }

    public static String reflectionToString(Object object2) {
        return ReflectionToStringBuilder.toString(object2);
    }

    public static String reflectionToString(Object object2, ToStringStyle style2) {
        return ReflectionToStringBuilder.toString(object2, style2);
    }

    public static String reflectionToString(Object object2, ToStringStyle style2, boolean outputTransients) {
        return ReflectionToStringBuilder.toString(object2, style2, outputTransients, false, (Class) null);
    }

    public static String reflectionToString(Object object2, ToStringStyle style2, boolean outputTransients, Class reflectUpToClass) {
        return ReflectionToStringBuilder.toString(object2, style2, outputTransients, false, reflectUpToClass);
    }

    public static void setDefaultStyle(ToStringStyle style2) {
        if (style2 != null) {
            defaultStyle = style2;
            return;
        }
        throw new IllegalArgumentException("The style must not be null");
    }

    public ToStringBuilder(Object object2) {
        this(object2, getDefaultStyle(), (StringBuffer) null);
    }

    public ToStringBuilder(Object object2, ToStringStyle style2) {
        this(object2, style2, (StringBuffer) null);
    }

    public ToStringBuilder(Object object2, ToStringStyle style2, StringBuffer buffer2) {
        style2 = style2 == null ? getDefaultStyle() : style2;
        buffer2 = buffer2 == null ? new StringBuffer(512) : buffer2;
        this.buffer = buffer2;
        this.style = style2;
        this.object = object2;
        style2.appendStart(buffer2, object2);
    }

    public ToStringBuilder append(boolean value) {
        this.style.append(this.buffer, (String) null, value);
        return this;
    }

    public ToStringBuilder append(boolean[] array) {
        this.style.append(this.buffer, (String) null, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(byte value) {
        this.style.append(this.buffer, (String) null, value);
        return this;
    }

    public ToStringBuilder append(byte[] array) {
        this.style.append(this.buffer, (String) null, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(char value) {
        this.style.append(this.buffer, (String) null, value);
        return this;
    }

    public ToStringBuilder append(char[] array) {
        this.style.append(this.buffer, (String) null, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(double value) {
        this.style.append(this.buffer, (String) null, value);
        return this;
    }

    public ToStringBuilder append(double[] array) {
        this.style.append(this.buffer, (String) null, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(float value) {
        this.style.append(this.buffer, (String) null, value);
        return this;
    }

    public ToStringBuilder append(float[] array) {
        this.style.append(this.buffer, (String) null, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(int value) {
        this.style.append(this.buffer, (String) null, value);
        return this;
    }

    public ToStringBuilder append(int[] array) {
        this.style.append(this.buffer, (String) null, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(long value) {
        this.style.append(this.buffer, (String) null, value);
        return this;
    }

    public ToStringBuilder append(long[] array) {
        this.style.append(this.buffer, (String) null, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(Object obj) {
        this.style.append(this.buffer, (String) null, obj, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(Object[] array) {
        this.style.append(this.buffer, (String) null, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(short value) {
        this.style.append(this.buffer, (String) null, value);
        return this;
    }

    public ToStringBuilder append(short[] array) {
        this.style.append(this.buffer, (String) null, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, boolean value) {
        this.style.append(this.buffer, fieldName, value);
        return this;
    }

    public ToStringBuilder append(String fieldName, boolean[] array) {
        this.style.append(this.buffer, fieldName, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, boolean[] array, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, array, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder append(String fieldName, byte value) {
        this.style.append(this.buffer, fieldName, value);
        return this;
    }

    public ToStringBuilder append(String fieldName, byte[] array) {
        this.style.append(this.buffer, fieldName, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, byte[] array, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, array, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder append(String fieldName, char value) {
        this.style.append(this.buffer, fieldName, value);
        return this;
    }

    public ToStringBuilder append(String fieldName, char[] array) {
        this.style.append(this.buffer, fieldName, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, char[] array, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, array, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder append(String fieldName, double value) {
        this.style.append(this.buffer, fieldName, value);
        return this;
    }

    public ToStringBuilder append(String fieldName, double[] array) {
        this.style.append(this.buffer, fieldName, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, double[] array, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, array, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder append(String fieldName, float value) {
        this.style.append(this.buffer, fieldName, value);
        return this;
    }

    public ToStringBuilder append(String fieldName, float[] array) {
        this.style.append(this.buffer, fieldName, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, float[] array, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, array, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder append(String fieldName, int value) {
        this.style.append(this.buffer, fieldName, value);
        return this;
    }

    public ToStringBuilder append(String fieldName, int[] array) {
        this.style.append(this.buffer, fieldName, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, int[] array, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, array, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder append(String fieldName, long value) {
        this.style.append(this.buffer, fieldName, value);
        return this;
    }

    public ToStringBuilder append(String fieldName, long[] array) {
        this.style.append(this.buffer, fieldName, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, long[] array, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, array, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder append(String fieldName, Object obj) {
        this.style.append(this.buffer, fieldName, obj, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, Object obj, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, obj, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder append(String fieldName, Object[] array) {
        this.style.append(this.buffer, fieldName, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, Object[] array, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, array, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder append(String fieldName, short value) {
        this.style.append(this.buffer, fieldName, value);
        return this;
    }

    public ToStringBuilder append(String fieldName, short[] array) {
        this.style.append(this.buffer, fieldName, array, (Boolean) null);
        return this;
    }

    public ToStringBuilder append(String fieldName, short[] array, boolean fullDetail) {
        this.style.append(this.buffer, fieldName, array, BooleanUtils.toBooleanObject(fullDetail));
        return this;
    }

    public ToStringBuilder appendAsObjectToString(Object object2) {
        ObjectUtils.appendIdentityToString(getStringBuffer(), object2);
        return this;
    }

    public ToStringBuilder appendSuper(String superToString) {
        if (superToString != null) {
            this.style.appendSuper(this.buffer, superToString);
        }
        return this;
    }

    public ToStringBuilder appendToString(String toString) {
        if (toString != null) {
            this.style.appendToString(this.buffer, toString);
        }
        return this;
    }

    public Object getObject() {
        return this.object;
    }

    public StringBuffer getStringBuffer() {
        return this.buffer;
    }

    public ToStringStyle getStyle() {
        return this.style;
    }

    public String toString() {
        if (getObject() == null) {
            getStringBuffer().append(getStyle().getNullText());
        } else {
            this.style.appendEnd(getStringBuffer(), getObject());
        }
        return getStringBuffer().toString();
    }
}
