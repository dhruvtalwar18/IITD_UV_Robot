package org.ros.message;

import com.google.common.base.Preconditions;
import org.apache.commons.httpclient.cookie.CookieSpec;

public class MessageIdentifier {
    private String name;
    private String pkg;
    private String type;

    public static MessageIdentifier of(String pkg2, String name2) {
        Preconditions.checkNotNull(pkg2);
        Preconditions.checkNotNull(name2);
        return new MessageIdentifier(pkg2, name2);
    }

    public static MessageIdentifier of(String type2) {
        Preconditions.checkNotNull(type2);
        if (type2.contains(CookieSpec.PATH_DELIM)) {
            return new MessageIdentifier(type2);
        }
        throw new IllegalArgumentException(String.format("Type name is invalid or not fully qualified: \"%s\"", new Object[]{type2}));
    }

    private MessageIdentifier(String type2) {
        this.type = type2;
    }

    private MessageIdentifier(String pkg2, String name2) {
        this.pkg = pkg2;
        this.name = name2;
    }

    public String getType() {
        if (this.type == null) {
            StringBuilder stringBuilder = new StringBuilder(this.pkg.length() + this.name.length() + 1);
            stringBuilder.append(this.pkg);
            stringBuilder.append(CookieSpec.PATH_DELIM);
            stringBuilder.append(this.name);
            this.type = stringBuilder.toString();
        }
        return this.type;
    }

    private void splitType() {
        String[] packageAndName = this.type.split(CookieSpec.PATH_DELIM, 2);
        this.pkg = packageAndName[0];
        this.name = packageAndName[1];
    }

    public String getPackage() {
        if (this.pkg == null) {
            splitType();
        }
        return this.pkg;
    }

    public String getName() {
        if (this.name == null) {
            splitType();
        }
        return this.name;
    }

    public String toString() {
        return String.format("MessageIdentifier<%s>", new Object[]{this.type});
    }

    public int hashCode() {
        return (1 * 31) + (this.type == null ? 0 : this.type.hashCode());
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        MessageIdentifier other = (MessageIdentifier) obj;
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
