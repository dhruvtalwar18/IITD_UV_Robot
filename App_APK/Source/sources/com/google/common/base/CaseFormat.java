package com.google.common.base;

import com.google.common.annotations.GwtCompatible;

@GwtCompatible
public enum CaseFormat {
    LOWER_HYPHEN(CharMatcher.is('-'), "-"),
    LOWER_UNDERSCORE(CharMatcher.is('_'), "_"),
    LOWER_CAMEL(CharMatcher.inRange('A', 'Z'), ""),
    UPPER_CAMEL(CharMatcher.inRange('A', 'Z'), ""),
    UPPER_UNDERSCORE(CharMatcher.is('_'), "_");
    
    private final CharMatcher wordBoundary;
    private final String wordSeparator;

    private CaseFormat(CharMatcher wordBoundary2, String wordSeparator2) {
        this.wordBoundary = wordBoundary2;
        this.wordSeparator = wordSeparator2;
    }

    public String to(CaseFormat format, String s) {
        if (format == null) {
            throw new NullPointerException();
        } else if (s == null) {
            throw new NullPointerException();
        } else if (format == this) {
            return s;
        } else {
            switch (this) {
                case LOWER_UNDERSCORE:
                    switch (format) {
                        case UPPER_UNDERSCORE:
                            return Ascii.toUpperCase(s);
                        case LOWER_HYPHEN:
                            return s.replace('_', '-');
                    }
                case UPPER_UNDERSCORE:
                    int i = AnonymousClass1.$SwitchMap$com$google$common$base$CaseFormat[format.ordinal()];
                    if (i == 1) {
                        return Ascii.toLowerCase(s);
                    }
                    if (i == 3) {
                        return Ascii.toLowerCase(s.replace('_', '-'));
                    }
                    break;
                case LOWER_HYPHEN:
                    switch (format) {
                        case LOWER_UNDERSCORE:
                            return s.replace('-', '_');
                        case UPPER_UNDERSCORE:
                            return Ascii.toUpperCase(s.replace('-', '_'));
                    }
            }
            int i2 = 0;
            StringBuilder out = null;
            int j = -1;
            while (true) {
                int indexIn = this.wordBoundary.indexIn(s, j + 1);
                j = indexIn;
                if (indexIn != -1) {
                    if (i2 == 0) {
                        out = new StringBuilder(s.length() + (this.wordSeparator.length() * 4));
                        out.append(format.normalizeFirstWord(s.substring(i2, j)));
                    } else {
                        out.append(format.normalizeWord(s.substring(i2, j)));
                    }
                    out.append(format.wordSeparator);
                    i2 = j + this.wordSeparator.length();
                } else if (i2 == 0) {
                    return format.normalizeFirstWord(s);
                } else {
                    out.append(format.normalizeWord(s.substring(i2)));
                    return out.toString();
                }
            }
        }
    }

    private String normalizeFirstWord(String word) {
        if (AnonymousClass1.$SwitchMap$com$google$common$base$CaseFormat[ordinal()] != 4) {
            return normalizeWord(word);
        }
        return Ascii.toLowerCase(word);
    }

    private String normalizeWord(String word) {
        switch (this) {
            case LOWER_UNDERSCORE:
                return Ascii.toLowerCase(word);
            case UPPER_UNDERSCORE:
                return Ascii.toUpperCase(word);
            case LOWER_HYPHEN:
                return Ascii.toLowerCase(word);
            case LOWER_CAMEL:
                return firstCharOnlyToUpper(word);
            case UPPER_CAMEL:
                return firstCharOnlyToUpper(word);
            default:
                throw new RuntimeException("unknown case: " + this);
        }
    }

    private static String firstCharOnlyToUpper(String word) {
        int length = word.length();
        if (length == 0) {
            return word;
        }
        StringBuilder sb = new StringBuilder(length);
        sb.append(Ascii.toUpperCase(word.charAt(0)));
        sb.append(Ascii.toLowerCase(word.substring(1)));
        return sb.toString();
    }
}
