package org.apache.commons.io.filefilter;

import java.io.File;
import java.io.Serializable;
import java.util.List;
import org.apache.commons.io.FilenameUtils;

public class WildcardFilter extends AbstractFileFilter implements Serializable {
    private final String[] wildcards;

    public WildcardFilter(String wildcard) {
        if (wildcard != null) {
            this.wildcards = new String[]{wildcard};
            return;
        }
        throw new IllegalArgumentException("The wildcard must not be null");
    }

    public WildcardFilter(String[] wildcards2) {
        if (wildcards2 != null) {
            this.wildcards = wildcards2;
            return;
        }
        throw new IllegalArgumentException("The wildcard array must not be null");
    }

    public WildcardFilter(List wildcards2) {
        if (wildcards2 != null) {
            this.wildcards = (String[]) wildcards2.toArray(new String[wildcards2.size()]);
            return;
        }
        throw new IllegalArgumentException("The wildcard list must not be null");
    }

    public boolean accept(File dir, String name) {
        if (dir != null && new File(dir, name).isDirectory()) {
            return false;
        }
        for (String wildcardMatch : this.wildcards) {
            if (FilenameUtils.wildcardMatch(name, wildcardMatch)) {
                return true;
            }
        }
        return false;
    }

    public boolean accept(File file) {
        if (file.isDirectory()) {
            return false;
        }
        for (String wildcardMatch : this.wildcards) {
            if (FilenameUtils.wildcardMatch(file.getName(), wildcardMatch)) {
                return true;
            }
        }
        return false;
    }
}
