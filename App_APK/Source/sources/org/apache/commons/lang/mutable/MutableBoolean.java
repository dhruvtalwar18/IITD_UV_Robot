package org.apache.commons.lang.mutable;

import java.io.Serializable;
import org.apache.commons.lang.BooleanUtils;

public class MutableBoolean implements Mutable, Serializable, Comparable {
    private static final long serialVersionUID = -4830728138360036487L;
    private boolean value;

    public MutableBoolean() {
    }

    public MutableBoolean(boolean value2) {
        this.value = value2;
    }

    public MutableBoolean(Boolean value2) {
        this.value = value2.booleanValue();
    }

    public boolean booleanValue() {
        return this.value;
    }

    public int compareTo(Object obj) {
        if (this.value == ((MutableBoolean) obj).value) {
            return 0;
        }
        return this.value ? 1 : -1;
    }

    public boolean equals(Object obj) {
        if (!(obj instanceof MutableBoolean) || this.value != ((MutableBoolean) obj).booleanValue()) {
            return false;
        }
        return true;
    }

    public Object getValue() {
        return BooleanUtils.toBooleanObject(this.value);
    }

    public int hashCode() {
        return (this.value ? Boolean.TRUE : Boolean.FALSE).hashCode();
    }

    public void setValue(boolean value2) {
        this.value = value2;
    }

    public void setValue(Object value2) {
        setValue(((Boolean) value2).booleanValue());
    }

    public String toString() {
        return String.valueOf(this.value);
    }
}
