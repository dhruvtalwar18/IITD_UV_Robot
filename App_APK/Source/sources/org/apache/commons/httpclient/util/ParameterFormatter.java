package org.apache.commons.httpclient.util;

import org.apache.commons.httpclient.NameValuePair;
import org.apache.commons.io.IOUtils;

public class ParameterFormatter {
    private static final char[] SEPARATORS = {'(', ')', '<', '>', '@', ',', ';', ':', IOUtils.DIR_SEPARATOR_WINDOWS, '\"', IOUtils.DIR_SEPARATOR_UNIX, '[', ']', '?', '=', '{', '}', ' ', 9};
    private static final char[] UNSAFE_CHARS = {'\"', IOUtils.DIR_SEPARATOR_WINDOWS};
    private boolean alwaysUseQuotes = true;

    private static boolean isOneOf(char[] chars, char ch) {
        for (char c : chars) {
            if (ch == c) {
                return true;
            }
        }
        return false;
    }

    private static boolean isUnsafeChar(char ch) {
        return isOneOf(UNSAFE_CHARS, ch);
    }

    private static boolean isSeparator(char ch) {
        return isOneOf(SEPARATORS, ch);
    }

    public boolean isAlwaysUseQuotes() {
        return this.alwaysUseQuotes;
    }

    public void setAlwaysUseQuotes(boolean alwaysUseQuotes2) {
        this.alwaysUseQuotes = alwaysUseQuotes2;
    }

    public static void formatValue(StringBuffer buffer, String value, boolean alwaysUseQuotes2) {
        if (buffer == null) {
            throw new IllegalArgumentException("String buffer may not be null");
        } else if (value != null) {
            int i = 0;
            if (alwaysUseQuotes2) {
                buffer.append('\"');
                while (i < value.length()) {
                    char ch = value.charAt(i);
                    if (isUnsafeChar(ch)) {
                        buffer.append(IOUtils.DIR_SEPARATOR_WINDOWS);
                    }
                    buffer.append(ch);
                    i++;
                }
                buffer.append('\"');
                return;
            }
            int offset = buffer.length();
            boolean unsafe = false;
            while (i < value.length()) {
                char ch2 = value.charAt(i);
                if (isSeparator(ch2)) {
                    unsafe = true;
                }
                if (isUnsafeChar(ch2)) {
                    buffer.append(IOUtils.DIR_SEPARATOR_WINDOWS);
                }
                buffer.append(ch2);
                i++;
            }
            if (unsafe) {
                buffer.insert(offset, '\"');
                buffer.append('\"');
            }
        } else {
            throw new IllegalArgumentException("Value buffer may not be null");
        }
    }

    public void format(StringBuffer buffer, NameValuePair param) {
        if (buffer == null) {
            throw new IllegalArgumentException("String buffer may not be null");
        } else if (param != null) {
            buffer.append(param.getName());
            String value = param.getValue();
            if (value != null) {
                buffer.append("=");
                formatValue(buffer, value, this.alwaysUseQuotes);
            }
        } else {
            throw new IllegalArgumentException("Parameter may not be null");
        }
    }

    public String format(NameValuePair param) {
        StringBuffer buffer = new StringBuffer();
        format(buffer, param);
        return buffer.toString();
    }
}
