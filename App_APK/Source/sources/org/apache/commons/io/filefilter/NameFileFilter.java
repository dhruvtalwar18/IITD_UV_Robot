package org.apache.commons.io.filefilter;

import java.io.File;
import java.io.Serializable;
import java.util.List;
import org.apache.commons.io.IOCase;

public class NameFileFilter extends AbstractFileFilter implements Serializable {
    private final IOCase caseSensitivity;
    private final String[] names;

    public NameFileFilter(String name) {
        this(name, (IOCase) null);
    }

    public NameFileFilter(String name, IOCase caseSensitivity2) {
        if (name != null) {
            this.names = new String[]{name};
            this.caseSensitivity = caseSensitivity2 == null ? IOCase.SENSITIVE : caseSensitivity2;
            return;
        }
        throw new IllegalArgumentException("The wildcard must not be null");
    }

    public NameFileFilter(String[] names2) {
        this(names2, (IOCase) null);
    }

    public NameFileFilter(String[] names2, IOCase caseSensitivity2) {
        if (names2 != null) {
            this.names = names2;
            this.caseSensitivity = caseSensitivity2 == null ? IOCase.SENSITIVE : caseSensitivity2;
            return;
        }
        throw new IllegalArgumentException("The array of names must not be null");
    }

    public NameFileFilter(List names2) {
        this(names2, (IOCase) null);
    }

    public NameFileFilter(List names2, IOCase caseSensitivity2) {
        if (names2 != null) {
            this.names = (String[]) names2.toArray(new String[names2.size()]);
            this.caseSensitivity = caseSensitivity2 == null ? IOCase.SENSITIVE : caseSensitivity2;
            return;
        }
        throw new IllegalArgumentException("The list of names must not be null");
    }

    public boolean accept(File file) {
        String name = file.getName();
        for (String checkEquals : this.names) {
            if (this.caseSensitivity.checkEquals(name, checkEquals)) {
                return true;
            }
        }
        return false;
    }

    public boolean accept(File file, String name) {
        for (String checkEquals : this.names) {
            if (this.caseSensitivity.checkEquals(name, checkEquals)) {
                return true;
            }
        }
        return false;
    }

    public String toString() {
        StringBuffer buffer = new StringBuffer();
        buffer.append(super.toString());
        buffer.append("(");
        if (this.names != null) {
            for (int i = 0; i < this.names.length; i++) {
                if (i > 0) {
                    buffer.append(",");
                }
                buffer.append(this.names[i]);
            }
        }
        buffer.append(")");
        return buffer.toString();
    }
}
