package org.apache.commons.io.filefilter;

import java.io.File;
import java.io.Serializable;
import java.util.List;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.io.IOCase;

public class WildcardFileFilter extends AbstractFileFilter implements Serializable {
    private final IOCase caseSensitivity;
    private final String[] wildcards;

    public WildcardFileFilter(String wildcard) {
        this(wildcard, (IOCase) null);
    }

    public WildcardFileFilter(String wildcard, IOCase caseSensitivity2) {
        if (wildcard != null) {
            this.wildcards = new String[]{wildcard};
            this.caseSensitivity = caseSensitivity2 == null ? IOCase.SENSITIVE : caseSensitivity2;
            return;
        }
        throw new IllegalArgumentException("The wildcard must not be null");
    }

    public WildcardFileFilter(String[] wildcards2) {
        this(wildcards2, (IOCase) null);
    }

    public WildcardFileFilter(String[] wildcards2, IOCase caseSensitivity2) {
        if (wildcards2 != null) {
            this.wildcards = wildcards2;
            this.caseSensitivity = caseSensitivity2 == null ? IOCase.SENSITIVE : caseSensitivity2;
            return;
        }
        throw new IllegalArgumentException("The wildcard array must not be null");
    }

    public WildcardFileFilter(List wildcards2) {
        this(wildcards2, (IOCase) null);
    }

    public WildcardFileFilter(List wildcards2, IOCase caseSensitivity2) {
        if (wildcards2 != null) {
            this.wildcards = (String[]) wildcards2.toArray(new String[wildcards2.size()]);
            this.caseSensitivity = caseSensitivity2 == null ? IOCase.SENSITIVE : caseSensitivity2;
            return;
        }
        throw new IllegalArgumentException("The wildcard list must not be null");
    }

    public boolean accept(File dir, String name) {
        for (String wildcardMatch : this.wildcards) {
            if (FilenameUtils.wildcardMatch(name, wildcardMatch, this.caseSensitivity)) {
                return true;
            }
        }
        return false;
    }

    public boolean accept(File file) {
        String name = file.getName();
        for (String wildcardMatch : this.wildcards) {
            if (FilenameUtils.wildcardMatch(name, wildcardMatch, this.caseSensitivity)) {
                return true;
            }
        }
        return false;
    }

    public String toString() {
        StringBuffer buffer = new StringBuffer();
        buffer.append(super.toString());
        buffer.append("(");
        if (this.wildcards != null) {
            for (int i = 0; i < this.wildcards.length; i++) {
                if (i > 0) {
                    buffer.append(",");
                }
                buffer.append(this.wildcards[i]);
            }
        }
        buffer.append(")");
        return buffer.toString();
    }
}
