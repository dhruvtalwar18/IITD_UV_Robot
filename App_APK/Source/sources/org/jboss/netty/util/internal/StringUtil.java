package org.jboss.netty.util.internal;

import java.util.Formatter;

public final class StringUtil {
    public static final String NEWLINE;

    private StringUtil() {
    }

    static {
        String newLine;
        try {
            newLine = new Formatter().format("%n", new Object[0]).toString();
        } catch (Exception e) {
            newLine = "\n";
        }
        NEWLINE = newLine;
    }

    public static String stripControlCharacters(Object value) {
        if (value == null) {
            return null;
        }
        return stripControlCharacters(value.toString());
    }

    public static String stripControlCharacters(String value) {
        if (value == null) {
            return null;
        }
        boolean hasControlChars = false;
        int i = value.length() - 1;
        while (true) {
            if (i < 0) {
                break;
            } else if (Character.isISOControl(value.charAt(i))) {
                hasControlChars = true;
                break;
            } else {
                i--;
            }
        }
        if (!hasControlChars) {
            return value;
        }
        StringBuilder buf = new StringBuilder(value.length());
        boolean suppressingControlChars = false;
        int i2 = 0;
        while (i2 < value.length() && Character.isISOControl(value.charAt(i2))) {
            i2++;
        }
        while (i2 < value.length()) {
            if (Character.isISOControl(value.charAt(i2))) {
                suppressingControlChars = true;
            } else {
                if (suppressingControlChars) {
                    suppressingControlChars = false;
                    buf.append(' ');
                }
                buf.append(value.charAt(i2));
            }
            i2++;
        }
        return buf.toString();
    }
}
