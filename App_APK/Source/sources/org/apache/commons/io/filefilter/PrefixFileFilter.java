package org.apache.commons.io.filefilter;

import java.io.File;
import java.io.Serializable;
import java.util.List;
import org.apache.commons.io.IOCase;

public class PrefixFileFilter extends AbstractFileFilter implements Serializable {
    private final IOCase caseSensitivity;
    private final String[] prefixes;

    public PrefixFileFilter(String prefix) {
        this(prefix, IOCase.SENSITIVE);
    }

    public PrefixFileFilter(String prefix, IOCase caseSensitivity2) {
        if (prefix != null) {
            this.prefixes = new String[]{prefix};
            this.caseSensitivity = caseSensitivity2 == null ? IOCase.SENSITIVE : caseSensitivity2;
            return;
        }
        throw new IllegalArgumentException("The prefix must not be null");
    }

    public PrefixFileFilter(String[] prefixes2) {
        this(prefixes2, IOCase.SENSITIVE);
    }

    public PrefixFileFilter(String[] prefixes2, IOCase caseSensitivity2) {
        if (prefixes2 != null) {
            this.prefixes = prefixes2;
            this.caseSensitivity = caseSensitivity2 == null ? IOCase.SENSITIVE : caseSensitivity2;
            return;
        }
        throw new IllegalArgumentException("The array of prefixes must not be null");
    }

    public PrefixFileFilter(List prefixes2) {
        this(prefixes2, IOCase.SENSITIVE);
    }

    public PrefixFileFilter(List prefixes2, IOCase caseSensitivity2) {
        if (prefixes2 != null) {
            this.prefixes = (String[]) prefixes2.toArray(new String[prefixes2.size()]);
            this.caseSensitivity = caseSensitivity2 == null ? IOCase.SENSITIVE : caseSensitivity2;
            return;
        }
        throw new IllegalArgumentException("The list of prefixes must not be null");
    }

    public boolean accept(File file) {
        String name = file.getName();
        for (String checkStartsWith : this.prefixes) {
            if (this.caseSensitivity.checkStartsWith(name, checkStartsWith)) {
                return true;
            }
        }
        return false;
    }

    public boolean accept(File file, String name) {
        for (String checkStartsWith : this.prefixes) {
            if (this.caseSensitivity.checkStartsWith(name, checkStartsWith)) {
                return true;
            }
        }
        return false;
    }

    public String toString() {
        StringBuffer buffer = new StringBuffer();
        buffer.append(super.toString());
        buffer.append("(");
        if (this.prefixes != null) {
            for (int i = 0; i < this.prefixes.length; i++) {
                if (i > 0) {
                    buffer.append(",");
                }
                buffer.append(this.prefixes[i]);
            }
        }
        buffer.append(")");
        return buffer.toString();
    }
}
