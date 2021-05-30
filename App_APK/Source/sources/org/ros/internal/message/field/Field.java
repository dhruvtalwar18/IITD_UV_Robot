package org.ros.internal.message.field;

import org.jboss.netty.buffer.ChannelBuffer;

public abstract class Field {
    protected final boolean isConstant;
    protected final String name;
    protected final FieldType type;

    public abstract void deserialize(ChannelBuffer channelBuffer);

    public abstract String getJavaTypeName();

    public abstract <T> T getValue();

    public abstract void serialize(ChannelBuffer channelBuffer);

    public abstract void setValue(Object obj);

    protected Field(FieldType type2, String name2, boolean isConstant2) {
        this.name = name2;
        this.type = type2;
        this.isConstant = isConstant2;
    }

    public String getName() {
        return this.name;
    }

    public FieldType getType() {
        return this.type;
    }

    public boolean isConstant() {
        return this.isConstant;
    }

    public String getMd5String() {
        if (isConstant()) {
            return String.format("%s %s=%s\n", new Object[]{getType().getMd5String(), getName(), getValue()});
        }
        return String.format("%s %s\n", new Object[]{getType().getMd5String(), getName()});
    }

    public int hashCode() {
        int i = 0;
        int result = ((((1 * 31) + (this.isConstant ? 1231 : 1237)) * 31) + (this.name == null ? 0 : this.name.hashCode())) * 31;
        if (this.type != null) {
            i = this.type.hashCode();
        }
        return result + i;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        Field other = (Field) obj;
        if (this.isConstant != other.isConstant) {
            return false;
        }
        if (this.name == null) {
            if (other.name != null) {
                return false;
            }
        } else if (!this.name.equals(other.name)) {
            return false;
        }
        if (this.type == null) {
            if (other.type != null) {
                return false;
            }
        } else if (!this.type.equals(other.type)) {
            return false;
        }
        return true;
    }
}
