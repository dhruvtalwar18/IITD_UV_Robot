package org.apache.commons.httpclient;

import java.io.Serializable;
import org.apache.commons.httpclient.util.LangUtils;

public class NameValuePair implements Serializable {
    private String name;
    private String value;

    public NameValuePair() {
        this((String) null, (String) null);
    }

    public NameValuePair(String name2, String value2) {
        this.name = null;
        this.value = null;
        this.name = name2;
        this.value = value2;
    }

    public void setName(String name2) {
        this.name = name2;
    }

    public String getName() {
        return this.name;
    }

    public void setValue(String value2) {
        this.value = value2;
    }

    public String getValue() {
        return this.value;
    }

    public String toString() {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("name=");
        stringBuffer.append(this.name);
        stringBuffer.append(", ");
        stringBuffer.append("value=");
        stringBuffer.append(this.value);
        return stringBuffer.toString();
    }

    public boolean equals(Object object) {
        if (object == null) {
            return false;
        }
        if (this == object) {
            return true;
        }
        if (!(object instanceof NameValuePair)) {
            return false;
        }
        NameValuePair that = (NameValuePair) object;
        if (!LangUtils.equals(this.name, that.name) || !LangUtils.equals(this.value, that.value)) {
            return false;
        }
        return true;
    }

    public int hashCode() {
        return LangUtils.hashCode(LangUtils.hashCode(17, (Object) this.name), (Object) this.value);
    }
}
