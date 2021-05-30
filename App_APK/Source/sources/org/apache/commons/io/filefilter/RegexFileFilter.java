package org.apache.commons.io.filefilter;

import java.io.File;
import java.io.Serializable;
import java.util.regex.Pattern;
import org.apache.commons.io.IOCase;

public class RegexFileFilter extends AbstractFileFilter implements Serializable {
    private final Pattern pattern;

    public RegexFileFilter(String pattern2) {
        if (pattern2 != null) {
            this.pattern = Pattern.compile(pattern2);
            return;
        }
        throw new IllegalArgumentException("Pattern is missing");
    }

    public RegexFileFilter(String pattern2, IOCase caseSensitivity) {
        if (pattern2 != null) {
            int flags = 0;
            if (caseSensitivity != null && !caseSensitivity.isCaseSensitive()) {
                flags = 2;
            }
            this.pattern = Pattern.compile(pattern2, flags);
            return;
        }
        throw new IllegalArgumentException("Pattern is missing");
    }

    public RegexFileFilter(String pattern2, int flags) {
        if (pattern2 != null) {
            this.pattern = Pattern.compile(pattern2, flags);
            return;
        }
        throw new IllegalArgumentException("Pattern is missing");
    }

    public RegexFileFilter(Pattern pattern2) {
        if (pattern2 != null) {
            this.pattern = pattern2;
            return;
        }
        throw new IllegalArgumentException("Pattern is missing");
    }

    public boolean accept(File dir, String name) {
        return this.pattern.matcher(name).matches();
    }
}
