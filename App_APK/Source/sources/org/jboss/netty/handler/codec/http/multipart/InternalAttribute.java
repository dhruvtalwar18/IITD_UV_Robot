package org.jboss.netty.handler.codec.http.multipart;

import java.util.ArrayList;
import java.util.List;
import org.apache.xmlrpc.serializer.TypeSerializerImpl;
import org.jboss.netty.handler.codec.http.multipart.InterfaceHttpData;

public class InternalAttribute implements InterfaceHttpData {
    protected List<String> value = new ArrayList();

    public InterfaceHttpData.HttpDataType getHttpDataType() {
        return InterfaceHttpData.HttpDataType.InternalAttribute;
    }

    public List<String> getValue() {
        return this.value;
    }

    public void addValue(String value2) {
        if (value2 != null) {
            this.value.add(value2);
            return;
        }
        throw new NullPointerException(TypeSerializerImpl.VALUE_TAG);
    }

    public void addValue(String value2, int rank) {
        if (value2 != null) {
            this.value.add(rank, value2);
            return;
        }
        throw new NullPointerException(TypeSerializerImpl.VALUE_TAG);
    }

    public void setValue(String value2, int rank) {
        if (value2 != null) {
            this.value.set(rank, value2);
            return;
        }
        throw new NullPointerException(TypeSerializerImpl.VALUE_TAG);
    }

    public int hashCode() {
        return getName().hashCode();
    }

    public boolean equals(Object o) {
        if (!(o instanceof Attribute)) {
            return false;
        }
        return getName().equalsIgnoreCase(((Attribute) o).getName());
    }

    public int compareTo(InterfaceHttpData arg0) {
        if (arg0 instanceof InternalAttribute) {
            return compareTo((InternalAttribute) arg0);
        }
        throw new ClassCastException("Cannot compare " + getHttpDataType() + " with " + arg0.getHttpDataType());
    }

    public int compareTo(InternalAttribute o) {
        return getName().compareToIgnoreCase(o.getName());
    }

    public int size() {
        int size = 0;
        for (String elt : this.value) {
            size += elt.length();
        }
        return size;
    }

    public String toString() {
        StringBuilder result = new StringBuilder();
        for (String elt : this.value) {
            result.append(elt);
        }
        return result.toString();
    }

    public String getName() {
        return "InternalAttribute";
    }
}
