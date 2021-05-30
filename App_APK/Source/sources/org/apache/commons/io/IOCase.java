package org.apache.commons.io;

import java.io.Serializable;

public final class IOCase implements Serializable {
    public static final IOCase INSENSITIVE = new IOCase("Insensitive", false);
    public static final IOCase SENSITIVE = new IOCase("Sensitive", true);
    public static final IOCase SYSTEM = new IOCase("System", true ^ FilenameUtils.isSystemWindows());
    private static final long serialVersionUID = -6343169151696340687L;
    private final String name;
    private final transient boolean sensitive;

    public static IOCase forName(String name2) {
        if (SENSITIVE.name.equals(name2)) {
            return SENSITIVE;
        }
        if (INSENSITIVE.name.equals(name2)) {
            return INSENSITIVE;
        }
        if (SYSTEM.name.equals(name2)) {
            return SYSTEM;
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Invalid IOCase name: ");
        stringBuffer.append(name2);
        throw new IllegalArgumentException(stringBuffer.toString());
    }

    private IOCase(String name2, boolean sensitive2) {
        this.name = name2;
        this.sensitive = sensitive2;
    }

    private Object readResolve() {
        return forName(this.name);
    }

    public String getName() {
        return this.name;
    }

    public boolean isCaseSensitive() {
        return this.sensitive;
    }

    public int checkCompareTo(String str1, String str2) {
        if (str1 != null && str2 != null) {
            return this.sensitive ? str1.compareTo(str2) : str1.compareToIgnoreCase(str2);
        }
        throw new NullPointerException("The strings must not be null");
    }

    public boolean checkEquals(String str1, String str2) {
        if (str1 != null && str2 != null) {
            return this.sensitive ? str1.equals(str2) : str1.equalsIgnoreCase(str2);
        }
        throw new NullPointerException("The strings must not be null");
    }

    public boolean checkStartsWith(String str, String start) {
        return str.regionMatches(!this.sensitive, 0, start, 0, start.length());
    }

    public boolean checkEndsWith(String str, String end) {
        int endLen = end.length();
        return str.regionMatches(!this.sensitive, str.length() - endLen, end, 0, endLen);
    }

    public boolean checkRegionMatches(String str, int strStartIndex, String search) {
        return str.regionMatches(!this.sensitive, strStartIndex, search, 0, search.length());
    }

    /* access modifiers changed from: package-private */
    public String convertCase(String str) {
        if (str == null) {
            return null;
        }
        return this.sensitive ? str : str.toLowerCase();
    }

    public String toString() {
        return this.name;
    }
}
