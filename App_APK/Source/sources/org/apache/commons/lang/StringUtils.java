package org.apache.commons.lang;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.List;

public class StringUtils {
    public static final String EMPTY = "";
    public static final int INDEX_NOT_FOUND = -1;
    private static final int PAD_LIMIT = 8192;

    public static boolean isEmpty(String str) {
        return str == null || str.length() == 0;
    }

    public static boolean isNotEmpty(String str) {
        return !isEmpty(str);
    }

    public static boolean isBlank(String str) {
        if (str != null) {
            int length = str.length();
            int strLen = length;
            if (length != 0) {
                for (int i = 0; i < strLen; i++) {
                    if (!Character.isWhitespace(str.charAt(i))) {
                        return false;
                    }
                }
                return true;
            }
        }
        return true;
    }

    public static boolean isNotBlank(String str) {
        return !isBlank(str);
    }

    public static String clean(String str) {
        return str == null ? "" : str.trim();
    }

    public static String trim(String str) {
        if (str == null) {
            return null;
        }
        return str.trim();
    }

    public static String trimToNull(String str) {
        String ts = trim(str);
        if (isEmpty(ts)) {
            return null;
        }
        return ts;
    }

    public static String trimToEmpty(String str) {
        return str == null ? "" : str.trim();
    }

    public static String strip(String str) {
        return strip(str, (String) null);
    }

    public static String stripToNull(String str) {
        if (str == null) {
            return null;
        }
        String str2 = strip(str, (String) null);
        if (str2.length() == 0) {
            return null;
        }
        return str2;
    }

    public static String stripToEmpty(String str) {
        return str == null ? "" : strip(str, (String) null);
    }

    public static String strip(String str, String stripChars) {
        if (isEmpty(str)) {
            return str;
        }
        return stripEnd(stripStart(str, stripChars), stripChars);
    }

    public static String stripStart(String str, String stripChars) {
        if (str != null) {
            int length = str.length();
            int strLen = length;
            if (length != 0) {
                int start = 0;
                if (stripChars == null) {
                    while (start != strLen && Character.isWhitespace(str.charAt(start))) {
                        start++;
                    }
                } else if (stripChars.length() == 0) {
                    return str;
                } else {
                    while (start != strLen && stripChars.indexOf(str.charAt(start)) != -1) {
                        start++;
                    }
                }
                return str.substring(start);
            }
        }
        return str;
    }

    public static String stripEnd(String str, String stripChars) {
        if (str != null) {
            int length = str.length();
            int end = length;
            if (length != 0) {
                if (stripChars == null) {
                    while (end != 0 && Character.isWhitespace(str.charAt(end - 1))) {
                        end--;
                    }
                } else if (stripChars.length() == 0) {
                    return str;
                } else {
                    while (end != 0 && stripChars.indexOf(str.charAt(end - 1)) != -1) {
                        end--;
                    }
                }
                return str.substring(0, end);
            }
        }
        return str;
    }

    public static String[] stripAll(String[] strs) {
        return stripAll(strs, (String) null);
    }

    public static String[] stripAll(String[] strs, String stripChars) {
        if (strs != null) {
            int length = strs.length;
            int strsLen = length;
            if (length != 0) {
                String[] newArr = new String[strsLen];
                for (int i = 0; i < strsLen; i++) {
                    newArr[i] = strip(strs[i], stripChars);
                }
                return newArr;
            }
        }
        return strs;
    }

    public static boolean equals(String str1, String str2) {
        if (str1 == null) {
            return str2 == null;
        }
        return str1.equals(str2);
    }

    public static boolean equalsIgnoreCase(String str1, String str2) {
        if (str1 == null) {
            return str2 == null;
        }
        return str1.equalsIgnoreCase(str2);
    }

    public static int indexOf(String str, char searchChar) {
        if (isEmpty(str)) {
            return -1;
        }
        return str.indexOf(searchChar);
    }

    public static int indexOf(String str, char searchChar, int startPos) {
        if (isEmpty(str)) {
            return -1;
        }
        return str.indexOf(searchChar, startPos);
    }

    public static int indexOf(String str, String searchStr) {
        if (str == null || searchStr == null) {
            return -1;
        }
        return str.indexOf(searchStr);
    }

    public static int ordinalIndexOf(String str, String searchStr, int ordinal) {
        int index = -1;
        if (str == null || searchStr == null || ordinal <= 0) {
            return -1;
        }
        if (searchStr.length() == 0) {
            return 0;
        }
        int found = 0;
        do {
            index = str.indexOf(searchStr, index + 1);
            if (index < 0 || (found = found + 1) >= ordinal) {
                return index;
            }
            index = str.indexOf(searchStr, index + 1);
            return index;
        } while ((found = found + 1) >= ordinal);
        return index;
    }

    public static int indexOf(String str, String searchStr, int startPos) {
        if (str == null || searchStr == null) {
            return -1;
        }
        if (searchStr.length() != 0 || startPos < str.length()) {
            return str.indexOf(searchStr, startPos);
        }
        return str.length();
    }

    public static int lastIndexOf(String str, char searchChar) {
        if (isEmpty(str)) {
            return -1;
        }
        return str.lastIndexOf(searchChar);
    }

    public static int lastIndexOf(String str, char searchChar, int startPos) {
        if (isEmpty(str)) {
            return -1;
        }
        return str.lastIndexOf(searchChar, startPos);
    }

    public static int lastIndexOf(String str, String searchStr) {
        if (str == null || searchStr == null) {
            return -1;
        }
        return str.lastIndexOf(searchStr);
    }

    public static int lastIndexOf(String str, String searchStr, int startPos) {
        if (str == null || searchStr == null) {
            return -1;
        }
        return str.lastIndexOf(searchStr, startPos);
    }

    public static boolean contains(String str, char searchChar) {
        if (!isEmpty(str) && str.indexOf(searchChar) >= 0) {
            return true;
        }
        return false;
    }

    public static boolean contains(String str, String searchStr) {
        if (str == null || searchStr == null || str.indexOf(searchStr) < 0) {
            return false;
        }
        return true;
    }

    public static boolean containsIgnoreCase(String str, String searchStr) {
        if (str == null || searchStr == null) {
            return false;
        }
        return contains(str.toUpperCase(), searchStr.toUpperCase());
    }

    public static int indexOfAny(String str, char[] searchChars) {
        if (isEmpty(str) || ArrayUtils.isEmpty(searchChars)) {
            return -1;
        }
        for (int i = 0; i < str.length(); i++) {
            char ch = str.charAt(i);
            for (char c : searchChars) {
                if (c == ch) {
                    return i;
                }
            }
        }
        return -1;
    }

    public static int indexOfAny(String str, String searchChars) {
        if (isEmpty(str) || isEmpty(searchChars)) {
            return -1;
        }
        return indexOfAny(str, searchChars.toCharArray());
    }

    public static boolean containsAny(String str, char[] searchChars) {
        if (str == null || str.length() == 0 || searchChars == null || searchChars.length == 0) {
            return false;
        }
        for (int i = 0; i < str.length(); i++) {
            char ch = str.charAt(i);
            for (char c : searchChars) {
                if (c == ch) {
                    return true;
                }
            }
        }
        return false;
    }

    public static boolean containsAny(String str, String searchChars) {
        if (searchChars == null) {
            return false;
        }
        return containsAny(str, searchChars.toCharArray());
    }

    public static int indexOfAnyBut(String str, char[] searchChars) {
        if (isEmpty(str) || ArrayUtils.isEmpty(searchChars)) {
            return -1;
        }
        int i = 0;
        while (i < str.length()) {
            char ch = str.charAt(i);
            int j = 0;
            while (j < searchChars.length) {
                if (searchChars[j] == ch) {
                    i++;
                } else {
                    j++;
                }
            }
            return i;
        }
        return -1;
    }

    public static int indexOfAnyBut(String str, String searchChars) {
        if (isEmpty(str) || isEmpty(searchChars)) {
            return -1;
        }
        for (int i = 0; i < str.length(); i++) {
            if (searchChars.indexOf(str.charAt(i)) < 0) {
                return i;
            }
        }
        return -1;
    }

    public static boolean containsOnly(String str, char[] valid) {
        if (valid == null || str == null) {
            return false;
        }
        if (str.length() == 0) {
            return true;
        }
        if (valid.length != 0 && indexOfAnyBut(str, valid) == -1) {
            return true;
        }
        return false;
    }

    public static boolean containsOnly(String str, String validChars) {
        if (str == null || validChars == null) {
            return false;
        }
        return containsOnly(str, validChars.toCharArray());
    }

    public static boolean containsNone(String str, char[] invalidChars) {
        if (str == null || invalidChars == null) {
            return true;
        }
        int strSize = str.length();
        for (int i = 0; i < strSize; i++) {
            char ch = str.charAt(i);
            for (char c : invalidChars) {
                if (c == ch) {
                    return false;
                }
            }
        }
        return true;
    }

    public static boolean containsNone(String str, String invalidChars) {
        if (str == null || invalidChars == null) {
            return true;
        }
        return containsNone(str, invalidChars.toCharArray());
    }

    public static int indexOfAny(String str, String[] searchStrs) {
        int tmp;
        if (str == null || searchStrs == null) {
            return -1;
        }
        int ret = Integer.MAX_VALUE;
        for (String search : searchStrs) {
            if (!(search == null || (tmp = str.indexOf(search)) == -1 || tmp >= ret)) {
                ret = tmp;
            }
        }
        if (ret == Integer.MAX_VALUE) {
            return -1;
        }
        return ret;
    }

    public static int lastIndexOfAny(String str, String[] searchStrs) {
        int tmp;
        if (str == null || searchStrs == null) {
            return -1;
        }
        int ret = -1;
        for (String search : searchStrs) {
            if (search != null && (tmp = str.lastIndexOf(search)) > ret) {
                ret = tmp;
            }
        }
        return ret;
    }

    public static String substring(String str, int start) {
        if (str == null) {
            return null;
        }
        if (start < 0) {
            start += str.length();
        }
        if (start < 0) {
            start = 0;
        }
        if (start > str.length()) {
            return "";
        }
        return str.substring(start);
    }

    public static String substring(String str, int start, int end) {
        if (str == null) {
            return null;
        }
        if (end < 0) {
            end += str.length();
        }
        if (start < 0) {
            start += str.length();
        }
        if (end > str.length()) {
            end = str.length();
        }
        if (start > end) {
            return "";
        }
        if (start < 0) {
            start = 0;
        }
        if (end < 0) {
            end = 0;
        }
        return str.substring(start, end);
    }

    public static String left(String str, int len) {
        if (str == null) {
            return null;
        }
        if (len < 0) {
            return "";
        }
        if (str.length() <= len) {
            return str;
        }
        return str.substring(0, len);
    }

    public static String right(String str, int len) {
        if (str == null) {
            return null;
        }
        if (len < 0) {
            return "";
        }
        if (str.length() <= len) {
            return str;
        }
        return str.substring(str.length() - len);
    }

    public static String mid(String str, int pos, int len) {
        if (str == null) {
            return null;
        }
        if (len < 0 || pos > str.length()) {
            return "";
        }
        if (pos < 0) {
            pos = 0;
        }
        if (str.length() <= pos + len) {
            return str.substring(pos);
        }
        return str.substring(pos, pos + len);
    }

    public static String substringBefore(String str, String separator) {
        if (isEmpty(str) || separator == null) {
            return str;
        }
        if (separator.length() == 0) {
            return "";
        }
        int pos = str.indexOf(separator);
        if (pos == -1) {
            return str;
        }
        return str.substring(0, pos);
    }

    public static String substringAfter(String str, String separator) {
        int pos;
        if (isEmpty(str)) {
            return str;
        }
        if (separator == null || (pos = str.indexOf(separator)) == -1) {
            return "";
        }
        return str.substring(separator.length() + pos);
    }

    public static String substringBeforeLast(String str, String separator) {
        int pos;
        if (isEmpty(str) || isEmpty(separator) || (pos = str.lastIndexOf(separator)) == -1) {
            return str;
        }
        return str.substring(0, pos);
    }

    public static String substringAfterLast(String str, String separator) {
        int pos;
        if (isEmpty(str)) {
            return str;
        }
        if (isEmpty(separator) || (pos = str.lastIndexOf(separator)) == -1 || pos == str.length() - separator.length()) {
            return "";
        }
        return str.substring(separator.length() + pos);
    }

    public static String substringBetween(String str, String tag) {
        return substringBetween(str, tag, tag);
    }

    public static String substringBetween(String str, String open, String close) {
        int start;
        int end;
        if (str == null || open == null || close == null || (start = str.indexOf(open)) == -1 || (end = str.indexOf(close, open.length() + start)) == -1) {
            return null;
        }
        return str.substring(open.length() + start, end);
    }

    /* JADX WARNING: Code restructure failed: missing block: B:15:0x0032, code lost:
        r6 = r6 + r3;
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static java.lang.String[] substringsBetween(java.lang.String r9, java.lang.String r10, java.lang.String r11) {
        /*
            r0 = 0
            if (r9 == 0) goto L_0x0058
            boolean r1 = isEmpty(r10)
            if (r1 != 0) goto L_0x0058
            boolean r1 = isEmpty(r11)
            if (r1 == 0) goto L_0x0010
            goto L_0x0058
        L_0x0010:
            int r1 = r9.length()
            if (r1 != 0) goto L_0x0019
            java.lang.String[] r0 = org.apache.commons.lang.ArrayUtils.EMPTY_STRING_ARRAY
            return r0
        L_0x0019:
            int r2 = r11.length()
            int r3 = r10.length()
            java.util.ArrayList r4 = new java.util.ArrayList
            r4.<init>()
            r5 = 0
        L_0x0027:
            int r6 = r1 - r2
            if (r5 >= r6) goto L_0x0044
            int r6 = r9.indexOf(r10, r5)
            if (r6 >= 0) goto L_0x0032
            goto L_0x0044
        L_0x0032:
            int r6 = r6 + r3
            int r7 = r9.indexOf(r11, r6)
            if (r7 >= 0) goto L_0x003a
            goto L_0x0044
        L_0x003a:
            java.lang.String r8 = r9.substring(r6, r7)
            r4.add(r8)
            int r5 = r7 + r2
            goto L_0x0027
        L_0x0044:
            boolean r6 = r4.isEmpty()
            if (r6 == 0) goto L_0x004b
            return r0
        L_0x004b:
            int r0 = r4.size()
            java.lang.String[] r0 = new java.lang.String[r0]
            java.lang.Object[] r0 = r4.toArray(r0)
            java.lang.String[] r0 = (java.lang.String[]) r0
            return r0
        L_0x0058:
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.StringUtils.substringsBetween(java.lang.String, java.lang.String, java.lang.String):java.lang.String[]");
    }

    public static String getNestedString(String str, String tag) {
        return substringBetween(str, tag, tag);
    }

    public static String getNestedString(String str, String open, String close) {
        return substringBetween(str, open, close);
    }

    public static String[] split(String str) {
        return split(str, (String) null, -1);
    }

    public static String[] split(String str, char separatorChar) {
        return splitWorker(str, separatorChar, false);
    }

    public static String[] split(String str, String separatorChars) {
        return splitWorker(str, separatorChars, -1, false);
    }

    public static String[] split(String str, String separatorChars, int max) {
        return splitWorker(str, separatorChars, max, false);
    }

    public static String[] splitByWholeSeparator(String str, String separator) {
        return splitByWholeSeparatorWorker(str, separator, -1, false);
    }

    public static String[] splitByWholeSeparator(String str, String separator, int max) {
        return splitByWholeSeparatorWorker(str, separator, max, false);
    }

    public static String[] splitByWholeSeparatorPreserveAllTokens(String str, String separator) {
        return splitByWholeSeparatorWorker(str, separator, -1, true);
    }

    public static String[] splitByWholeSeparatorPreserveAllTokens(String str, String separator, int max) {
        return splitByWholeSeparatorWorker(str, separator, max, true);
    }

    private static String[] splitByWholeSeparatorWorker(String str, String separator, int max, boolean preserveAllTokens) {
        if (str == null) {
            return null;
        }
        int len = str.length();
        if (len == 0) {
            return ArrayUtils.EMPTY_STRING_ARRAY;
        }
        if (separator == null || "".equals(separator)) {
            return splitWorker(str, (String) null, max, preserveAllTokens);
        }
        int separatorLength = separator.length();
        ArrayList substrings = new ArrayList();
        int numberOfSubstrings = 0;
        int beg = 0;
        int end = 0;
        while (end < len) {
            end = str.indexOf(separator, beg);
            if (end <= -1) {
                substrings.add(str.substring(beg));
                end = len;
            } else if (end > beg) {
                numberOfSubstrings++;
                if (numberOfSubstrings == max) {
                    end = len;
                    substrings.add(str.substring(beg));
                } else {
                    substrings.add(str.substring(beg, end));
                    beg = end + separatorLength;
                }
            } else {
                if (preserveAllTokens) {
                    numberOfSubstrings++;
                    if (numberOfSubstrings == max) {
                        end = len;
                        substrings.add(str.substring(beg));
                    } else {
                        substrings.add("");
                    }
                }
                beg = end + separatorLength;
            }
        }
        return (String[]) substrings.toArray(new String[substrings.size()]);
    }

    public static String[] splitPreserveAllTokens(String str) {
        return splitWorker(str, (String) null, -1, true);
    }

    public static String[] splitPreserveAllTokens(String str, char separatorChar) {
        return splitWorker(str, separatorChar, true);
    }

    private static String[] splitWorker(String str, char separatorChar, boolean preserveAllTokens) {
        if (str == null) {
            return null;
        }
        int len = str.length();
        if (len == 0) {
            return ArrayUtils.EMPTY_STRING_ARRAY;
        }
        List list = new ArrayList();
        int i = 0;
        int start = 0;
        boolean match = false;
        boolean lastMatch = false;
        while (i < len) {
            if (str.charAt(i) == separatorChar) {
                if (match || preserveAllTokens) {
                    list.add(str.substring(start, i));
                    match = false;
                    lastMatch = true;
                }
                i++;
                start = i;
            } else {
                lastMatch = false;
                match = true;
                i++;
            }
        }
        if (match || (preserveAllTokens && lastMatch)) {
            list.add(str.substring(start, i));
        }
        return (String[]) list.toArray(new String[list.size()]);
    }

    public static String[] splitPreserveAllTokens(String str, String separatorChars) {
        return splitWorker(str, separatorChars, -1, true);
    }

    public static String[] splitPreserveAllTokens(String str, String separatorChars, int max) {
        return splitWorker(str, separatorChars, max, true);
    }

    private static String[] splitWorker(String str, String separatorChars, int max, boolean preserveAllTokens) {
        int i;
        int i2;
        if (str == null) {
            return null;
        }
        int len = str.length();
        if (len == 0) {
            return ArrayUtils.EMPTY_STRING_ARRAY;
        }
        List list = new ArrayList();
        int sizePlus1 = 1;
        int i3 = 0;
        int start = 0;
        boolean match = false;
        boolean lastMatch = false;
        if (separatorChars == null) {
            while (i3 < len) {
                if (Character.isWhitespace(str.charAt(i3))) {
                    if (match || preserveAllTokens) {
                        lastMatch = true;
                        int sizePlus12 = sizePlus1 + 1;
                        if (sizePlus1 == max) {
                            i3 = len;
                            lastMatch = false;
                        }
                        list.add(str.substring(start, i3));
                        match = false;
                        sizePlus1 = sizePlus12;
                    }
                    i3++;
                    start = i3;
                } else {
                    lastMatch = false;
                    match = true;
                    i3++;
                }
            }
        } else if (separatorChars.length() == 1) {
            char sep = separatorChars.charAt(0);
            while (i3 < len) {
                if (str.charAt(i3) == sep) {
                    if (match || preserveAllTokens) {
                        boolean lastMatch2 = true;
                        int sizePlus13 = sizePlus1 + 1;
                        if (sizePlus1 == max) {
                            i3 = len;
                            lastMatch2 = false;
                        }
                        list.add(str.substring(start, i3));
                        match = false;
                        sizePlus1 = sizePlus13;
                    }
                    i2 = i3 + 1;
                    start = i2;
                } else {
                    lastMatch = false;
                    match = true;
                    i2 = i3 + 1;
                }
            }
        } else {
            while (i3 < len) {
                if (separatorChars.indexOf(str.charAt(i3)) >= 0) {
                    if (match || preserveAllTokens) {
                        boolean lastMatch3 = true;
                        int sizePlus14 = sizePlus1 + 1;
                        if (sizePlus1 == max) {
                            i3 = len;
                            lastMatch3 = false;
                        }
                        list.add(str.substring(start, i3));
                        match = false;
                        sizePlus1 = sizePlus14;
                    }
                    i = i3 + 1;
                    start = i;
                } else {
                    lastMatch = false;
                    match = true;
                    i = i3 + 1;
                }
            }
        }
        if (match || (preserveAllTokens && lastMatch)) {
            list.add(str.substring(start, i3));
        }
        return (String[]) list.toArray(new String[list.size()]);
    }

    public static String[] splitByCharacterType(String str) {
        return splitByCharacterType(str, false);
    }

    public static String[] splitByCharacterTypeCamelCase(String str) {
        return splitByCharacterType(str, true);
    }

    private static String[] splitByCharacterType(String str, boolean camelCase) {
        if (str == null) {
            return null;
        }
        if (str.length() == 0) {
            return ArrayUtils.EMPTY_STRING_ARRAY;
        }
        char[] c = str.toCharArray();
        List list = new ArrayList();
        int tokenStart = 0;
        int currentType = Character.getType(c[0]);
        for (int pos = 0 + 1; pos < c.length; pos++) {
            int type = Character.getType(c[pos]);
            if (type != currentType) {
                if (camelCase && type == 2 && currentType == 1) {
                    int newTokenStart = pos - 1;
                    if (newTokenStart != tokenStart) {
                        list.add(new String(c, tokenStart, newTokenStart - tokenStart));
                        tokenStart = newTokenStart;
                    }
                } else {
                    list.add(new String(c, tokenStart, pos - tokenStart));
                    tokenStart = pos;
                }
                currentType = type;
            }
        }
        list.add(new String(c, tokenStart, c.length - tokenStart));
        return (String[]) list.toArray(new String[list.size()]);
    }

    public static String concatenate(Object[] array) {
        return join(array, (String) null);
    }

    public static String join(Object[] array) {
        return join(array, (String) null);
    }

    public static String join(Object[] array, char separator) {
        if (array == null) {
            return null;
        }
        return join(array, separator, 0, array.length);
    }

    public static String join(Object[] array, char separator, int startIndex, int endIndex) {
        if (array == null) {
            return null;
        }
        int bufSize = endIndex - startIndex;
        if (bufSize <= 0) {
            return "";
        }
        StringBuffer buf = new StringBuffer(bufSize * ((array[startIndex] == null ? 16 : array[startIndex].toString().length()) + 1));
        for (int i = startIndex; i < endIndex; i++) {
            if (i > startIndex) {
                buf.append(separator);
            }
            if (array[i] != null) {
                buf.append(array[i]);
            }
        }
        return buf.toString();
    }

    public static String join(Object[] array, String separator) {
        if (array == null) {
            return null;
        }
        return join(array, separator, 0, array.length);
    }

    public static String join(Object[] array, String separator, int startIndex, int endIndex) {
        if (array == null) {
            return null;
        }
        if (separator == null) {
            separator = "";
        }
        int bufSize = endIndex - startIndex;
        if (bufSize <= 0) {
            return "";
        }
        StringBuffer buf = new StringBuffer(bufSize * ((array[startIndex] == null ? 16 : array[startIndex].toString().length()) + separator.length()));
        for (int i = startIndex; i < endIndex; i++) {
            if (i > startIndex) {
                buf.append(separator);
            }
            if (array[i] != null) {
                buf.append(array[i]);
            }
        }
        return buf.toString();
    }

    public static String join(Iterator iterator, char separator) {
        if (iterator == null) {
            return null;
        }
        if (!iterator.hasNext()) {
            return "";
        }
        Object first = iterator.next();
        if (!iterator.hasNext()) {
            return ObjectUtils.toString(first);
        }
        StringBuffer buf = new StringBuffer(256);
        if (first != null) {
            buf.append(first);
        }
        while (iterator.hasNext()) {
            buf.append(separator);
            Object obj = iterator.next();
            if (obj != null) {
                buf.append(obj);
            }
        }
        return buf.toString();
    }

    public static String join(Iterator iterator, String separator) {
        if (iterator == null) {
            return null;
        }
        if (!iterator.hasNext()) {
            return "";
        }
        Object first = iterator.next();
        if (!iterator.hasNext()) {
            return ObjectUtils.toString(first);
        }
        StringBuffer buf = new StringBuffer(256);
        if (first != null) {
            buf.append(first);
        }
        while (iterator.hasNext()) {
            if (separator != null) {
                buf.append(separator);
            }
            Object obj = iterator.next();
            if (obj != null) {
                buf.append(obj);
            }
        }
        return buf.toString();
    }

    public static String join(Collection collection, char separator) {
        if (collection == null) {
            return null;
        }
        return join(collection.iterator(), separator);
    }

    public static String join(Collection collection, String separator) {
        if (collection == null) {
            return null;
        }
        return join(collection.iterator(), separator);
    }

    public static String deleteSpaces(String str) {
        if (str == null) {
            return null;
        }
        return CharSetUtils.delete(str, " \t\r\n\b");
    }

    public static String deleteWhitespace(String str) {
        if (isEmpty(str)) {
            return str;
        }
        int sz = str.length();
        char[] chs = new char[sz];
        int count = 0;
        for (int i = 0; i < sz; i++) {
            if (!Character.isWhitespace(str.charAt(i))) {
                chs[count] = str.charAt(i);
                count++;
            }
        }
        if (count == sz) {
            return str;
        }
        return new String(chs, 0, count);
    }

    public static String removeStart(String str, String remove) {
        if (isEmpty(str) || isEmpty(remove) || !str.startsWith(remove)) {
            return str;
        }
        return str.substring(remove.length());
    }

    public static String removeStartIgnoreCase(String str, String remove) {
        if (isEmpty(str) || isEmpty(remove) || !startsWithIgnoreCase(str, remove)) {
            return str;
        }
        return str.substring(remove.length());
    }

    public static String removeEnd(String str, String remove) {
        if (isEmpty(str) || isEmpty(remove) || !str.endsWith(remove)) {
            return str;
        }
        return str.substring(0, str.length() - remove.length());
    }

    public static String removeEndIgnoreCase(String str, String remove) {
        if (isEmpty(str) || isEmpty(remove) || !endsWithIgnoreCase(str, remove)) {
            return str;
        }
        return str.substring(0, str.length() - remove.length());
    }

    public static String remove(String str, String remove) {
        if (isEmpty(str) || isEmpty(remove)) {
            return str;
        }
        return replace(str, remove, "", -1);
    }

    public static String remove(String str, char remove) {
        if (isEmpty(str) || str.indexOf(remove) == -1) {
            return str;
        }
        char[] chars = str.toCharArray();
        int pos = 0;
        for (int i = 0; i < chars.length; i++) {
            if (chars[i] != remove) {
                chars[pos] = chars[i];
                pos++;
            }
        }
        return new String(chars, 0, pos);
    }

    public static String replaceOnce(String text, String searchString, String replacement) {
        return replace(text, searchString, replacement, 1);
    }

    public static String replace(String text, String searchString, String replacement) {
        return replace(text, searchString, replacement, -1);
    }

    public static String replace(String text, String searchString, String replacement, int max) {
        if (isEmpty(text) || isEmpty(searchString) || replacement == null || max == 0) {
            return text;
        }
        int start = 0;
        int end = text.indexOf(searchString, 0);
        if (end == -1) {
            return text;
        }
        int replLength = searchString.length();
        int increase = replacement.length() - replLength;
        int increase2 = increase < 0 ? 0 : increase;
        int i = 64;
        if (max < 0) {
            i = 16;
        } else if (max <= 64) {
            i = max;
        }
        StringBuffer buf = new StringBuffer(text.length() + (increase2 * i));
        while (end != -1) {
            buf.append(text.substring(start, end));
            buf.append(replacement);
            start = end + replLength;
            max--;
            if (max == 0) {
                break;
            }
            end = text.indexOf(searchString, start);
        }
        buf.append(text.substring(start));
        return buf.toString();
    }

    public static String replaceEach(String text, String[] searchList, String[] replacementList) {
        return replaceEach(text, searchList, replacementList, false, 0);
    }

    public static String replaceEachRepeatedly(String text, String[] searchList, String[] replacementList) {
        return replaceEach(text, searchList, replacementList, true, searchList == null ? 0 : searchList.length);
    }

    private static String replaceEach(String text, String[] searchList, String[] replacementList, boolean repeat, int timeToLive) {
        int tempIndex;
        String str = text;
        String[] strArr = searchList;
        String[] strArr2 = replacementList;
        boolean z = repeat;
        int i = timeToLive;
        if (str == null || text.length() == 0 || strArr == null || strArr.length == 0 || strArr2 == null || strArr2.length == 0) {
            return str;
        }
        if (i >= 0) {
            int searchLength = strArr.length;
            int replacementLength = strArr2.length;
            if (searchLength == replacementLength) {
                boolean[] noMoreMatchesForReplIndex = new boolean[searchLength];
                int replaceIndex = -1;
                int textIndex = -1;
                int i2 = 0;
                while (true) {
                    if (i2 >= searchLength) {
                        break;
                    }
                    if (!(noMoreMatchesForReplIndex[i2] || strArr[i2] == null || strArr[i2].length() == 0 || strArr2[i2] == null)) {
                        int tempIndex2 = str.indexOf(strArr[i2]);
                        if (tempIndex2 == -1) {
                            noMoreMatchesForReplIndex[i2] = true;
                        } else if (textIndex == -1 || tempIndex2 < textIndex) {
                            textIndex = tempIndex2;
                            replaceIndex = i2;
                        }
                    }
                    i2++;
                }
                if (textIndex == -1) {
                    return str;
                }
                int start = 0;
                int increase = 0;
                for (int i3 = 0; i3 < strArr.length; i3++) {
                    int greater = strArr2[i3].length() - strArr[i3].length();
                    if (greater > 0) {
                        increase += greater * 3;
                    }
                }
                StringBuffer buf = new StringBuffer(text.length() + Math.min(increase, text.length() / 5));
                for (tempIndex = -1; textIndex != tempIndex; tempIndex = -1) {
                    for (int i4 = start; i4 < textIndex; i4++) {
                        buf.append(str.charAt(i4));
                    }
                    buf.append(strArr2[replaceIndex]);
                    start = textIndex + strArr[replaceIndex].length();
                    int tempIndex3 = -1;
                    int replaceIndex2 = -1;
                    int textIndex2 = -1;
                    for (int i5 = 0; i5 < searchLength; i5++) {
                        if (!noMoreMatchesForReplIndex[i5] && strArr[i5] != null && strArr[i5].length() != 0 && strArr2[i5] != null) {
                            tempIndex3 = str.indexOf(strArr[i5], start);
                            if (tempIndex3 == -1) {
                                noMoreMatchesForReplIndex[i5] = true;
                            } else if (textIndex2 == -1 || tempIndex3 < textIndex2) {
                                textIndex2 = tempIndex3;
                                replaceIndex2 = i5;
                            }
                        }
                    }
                    textIndex = textIndex2;
                    replaceIndex = replaceIndex2;
                    int replaceIndex3 = tempIndex3;
                }
                int textLength = text.length();
                int i6 = start;
                while (i6 < textLength) {
                    buf.append(str.charAt(i6));
                    i6++;
                    noMoreMatchesForReplIndex = noMoreMatchesForReplIndex;
                }
                String result = buf.toString();
                if (!z) {
                    return result;
                }
                return replaceEach(result, strArr, strArr2, z, i - 1);
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Search and Replace array lengths don't match: ");
            stringBuffer.append(searchLength);
            stringBuffer.append(" vs ");
            stringBuffer.append(replacementLength);
            throw new IllegalArgumentException(stringBuffer.toString());
        }
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append("TimeToLive of ");
        stringBuffer2.append(i);
        stringBuffer2.append(" is less than 0: ");
        stringBuffer2.append(str);
        throw new IllegalStateException(stringBuffer2.toString());
    }

    public static String replaceChars(String str, char searchChar, char replaceChar) {
        if (str == null) {
            return null;
        }
        return str.replace(searchChar, replaceChar);
    }

    public static String replaceChars(String str, String searchChars, String replaceChars) {
        if (isEmpty(str) || isEmpty(searchChars)) {
            return str;
        }
        if (replaceChars == null) {
            replaceChars = "";
        }
        boolean modified = false;
        int replaceCharsLength = replaceChars.length();
        int strLength = str.length();
        StringBuffer buf = new StringBuffer(strLength);
        for (int i = 0; i < strLength; i++) {
            char ch = str.charAt(i);
            int index = searchChars.indexOf(ch);
            if (index >= 0) {
                modified = true;
                if (index < replaceCharsLength) {
                    buf.append(replaceChars.charAt(index));
                }
            } else {
                buf.append(ch);
            }
        }
        if (modified) {
            return buf.toString();
        }
        return str;
    }

    public static String overlayString(String text, String overlay, int start, int end) {
        StringBuffer stringBuffer = new StringBuffer((((overlay.length() + start) + text.length()) - end) + 1);
        stringBuffer.append(text.substring(0, start));
        stringBuffer.append(overlay);
        stringBuffer.append(text.substring(end));
        return stringBuffer.toString();
    }

    public static String overlay(String str, String overlay, int start, int end) {
        if (str == null) {
            return null;
        }
        if (overlay == null) {
            overlay = "";
        }
        int len = str.length();
        if (start < 0) {
            start = 0;
        }
        if (start > len) {
            start = len;
        }
        if (end < 0) {
            end = 0;
        }
        if (end > len) {
            end = len;
        }
        if (start > end) {
            int temp = start;
            start = end;
            end = temp;
        }
        StringBuffer stringBuffer = new StringBuffer(((len + start) - end) + overlay.length() + 1);
        stringBuffer.append(str.substring(0, start));
        stringBuffer.append(overlay);
        stringBuffer.append(str.substring(end));
        return stringBuffer.toString();
    }

    public static String chomp(String str) {
        if (isEmpty(str)) {
            return str;
        }
        if (str.length() == 1) {
            char ch = str.charAt(0);
            if (ch == 13 || ch == 10) {
                return "";
            }
            return str;
        }
        int lastIdx = str.length() - 1;
        char last = str.charAt(lastIdx);
        if (last == 10) {
            if (str.charAt(lastIdx - 1) == 13) {
                lastIdx--;
            }
        } else if (last != 13) {
            lastIdx++;
        }
        return str.substring(0, lastIdx);
    }

    public static String chomp(String str, String separator) {
        if (isEmpty(str) || separator == null || !str.endsWith(separator)) {
            return str;
        }
        return str.substring(0, str.length() - separator.length());
    }

    public static String chompLast(String str) {
        return chompLast(str, "\n");
    }

    public static String chompLast(String str, String sep) {
        if (str.length() != 0 && sep.equals(str.substring(str.length() - sep.length()))) {
            return str.substring(0, str.length() - sep.length());
        }
        return str;
    }

    public static String getChomp(String str, String sep) {
        int idx = str.lastIndexOf(sep);
        if (idx == str.length() - sep.length()) {
            return sep;
        }
        if (idx != -1) {
            return str.substring(idx);
        }
        return "";
    }

    public static String prechomp(String str, String sep) {
        int idx = str.indexOf(sep);
        if (idx == -1) {
            return str;
        }
        return str.substring(sep.length() + idx);
    }

    public static String getPrechomp(String str, String sep) {
        int idx = str.indexOf(sep);
        if (idx == -1) {
            return "";
        }
        return str.substring(0, sep.length() + idx);
    }

    public static String chop(String str) {
        if (str == null) {
            return null;
        }
        int strLen = str.length();
        if (strLen < 2) {
            return "";
        }
        int lastIdx = strLen - 1;
        String ret = str.substring(0, lastIdx);
        if (str.charAt(lastIdx) == 10 && ret.charAt(lastIdx - 1) == 13) {
            return ret.substring(0, lastIdx - 1);
        }
        return ret;
    }

    public static String chopNewline(String str) {
        int lastIdx = str.length() - 1;
        if (lastIdx <= 0) {
            return "";
        }
        if (str.charAt(lastIdx) != 10) {
            lastIdx++;
        } else if (str.charAt(lastIdx - 1) == 13) {
            lastIdx--;
        }
        return str.substring(0, lastIdx);
    }

    public static String escape(String str) {
        return StringEscapeUtils.escapeJava(str);
    }

    public static String repeat(String str, int repeat) {
        if (str == null) {
            return null;
        }
        if (repeat <= 0) {
            return "";
        }
        int inputLength = str.length();
        if (repeat == 1 || inputLength == 0) {
            return str;
        }
        if (inputLength == 1 && repeat <= 8192) {
            return padding(repeat, str.charAt(0));
        }
        int outputLength = inputLength * repeat;
        switch (inputLength) {
            case 1:
                char ch = str.charAt(0);
                char[] output1 = new char[outputLength];
                for (int i = repeat - 1; i >= 0; i--) {
                    output1[i] = ch;
                }
                return new String(output1);
            case 2:
                char ch0 = str.charAt(0);
                char ch1 = str.charAt(1);
                char[] output2 = new char[outputLength];
                for (int i2 = (repeat * 2) - 2; i2 >= 0; i2 = (i2 - 1) - 1) {
                    output2[i2] = ch0;
                    output2[i2 + 1] = ch1;
                }
                return new String(output2);
            default:
                StringBuffer buf = new StringBuffer(outputLength);
                for (int i3 = 0; i3 < repeat; i3++) {
                    buf.append(str);
                }
                return buf.toString();
        }
    }

    private static String padding(int repeat, char padChar) throws IndexOutOfBoundsException {
        if (repeat >= 0) {
            char[] buf = new char[repeat];
            for (int i = 0; i < buf.length; i++) {
                buf[i] = padChar;
            }
            return new String(buf);
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Cannot pad a negative amount: ");
        stringBuffer.append(repeat);
        throw new IndexOutOfBoundsException(stringBuffer.toString());
    }

    public static String rightPad(String str, int size) {
        return rightPad(str, size, ' ');
    }

    public static String rightPad(String str, int size, char padChar) {
        if (str == null) {
            return null;
        }
        int pads = size - str.length();
        if (pads <= 0) {
            return str;
        }
        if (pads > 8192) {
            return rightPad(str, size, String.valueOf(padChar));
        }
        return str.concat(padding(pads, padChar));
    }

    public static String rightPad(String str, int size, String padStr) {
        if (str == null) {
            return null;
        }
        if (isEmpty(padStr)) {
            padStr = " ";
        }
        int padLen = padStr.length();
        int pads = size - str.length();
        if (pads <= 0) {
            return str;
        }
        if (padLen == 1 && pads <= 8192) {
            return rightPad(str, size, padStr.charAt(0));
        }
        if (pads == padLen) {
            return str.concat(padStr);
        }
        if (pads < padLen) {
            return str.concat(padStr.substring(0, pads));
        }
        char[] padding = new char[pads];
        char[] padChars = padStr.toCharArray();
        for (int i = 0; i < pads; i++) {
            padding[i] = padChars[i % padLen];
        }
        return str.concat(new String(padding));
    }

    public static String leftPad(String str, int size) {
        return leftPad(str, size, ' ');
    }

    public static String leftPad(String str, int size, char padChar) {
        if (str == null) {
            return null;
        }
        int pads = size - str.length();
        if (pads <= 0) {
            return str;
        }
        if (pads > 8192) {
            return leftPad(str, size, String.valueOf(padChar));
        }
        return padding(pads, padChar).concat(str);
    }

    public static String leftPad(String str, int size, String padStr) {
        if (str == null) {
            return null;
        }
        if (isEmpty(padStr)) {
            padStr = " ";
        }
        int padLen = padStr.length();
        int pads = size - str.length();
        if (pads <= 0) {
            return str;
        }
        if (padLen == 1 && pads <= 8192) {
            return leftPad(str, size, padStr.charAt(0));
        }
        if (pads == padLen) {
            return padStr.concat(str);
        }
        if (pads < padLen) {
            return padStr.substring(0, pads).concat(str);
        }
        char[] padding = new char[pads];
        char[] padChars = padStr.toCharArray();
        for (int i = 0; i < pads; i++) {
            padding[i] = padChars[i % padLen];
        }
        return new String(padding).concat(str);
    }

    public static int length(String str) {
        if (str == null) {
            return 0;
        }
        return str.length();
    }

    public static String center(String str, int size) {
        return center(str, size, ' ');
    }

    /* JADX WARNING: Code restructure failed: missing block: B:2:0x0005, code lost:
        r0 = r3.length();
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static java.lang.String center(java.lang.String r3, int r4, char r5) {
        /*
            if (r3 == 0) goto L_0x001a
            if (r4 > 0) goto L_0x0005
            goto L_0x001a
        L_0x0005:
            int r0 = r3.length()
            int r1 = r4 - r0
            if (r1 > 0) goto L_0x000e
            return r3
        L_0x000e:
            int r2 = r1 / 2
            int r2 = r2 + r0
            java.lang.String r3 = leftPad((java.lang.String) r3, (int) r2, (char) r5)
            java.lang.String r3 = rightPad((java.lang.String) r3, (int) r4, (char) r5)
            return r3
        L_0x001a:
            return r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.StringUtils.center(java.lang.String, int, char):java.lang.String");
    }

    public static String center(String str, int size, String padStr) {
        if (str == null || size <= 0) {
            return str;
        }
        if (isEmpty(padStr)) {
            padStr = " ";
        }
        int strLen = str.length();
        int pads = size - strLen;
        if (pads <= 0) {
            return str;
        }
        return rightPad(leftPad(str, (pads / 2) + strLen, padStr), size, padStr);
    }

    public static String upperCase(String str) {
        if (str == null) {
            return null;
        }
        return str.toUpperCase();
    }

    public static String lowerCase(String str) {
        if (str == null) {
            return null;
        }
        return str.toLowerCase();
    }

    public static String capitalize(String str) {
        if (str != null) {
            int length = str.length();
            int strLen = length;
            if (length != 0) {
                StringBuffer stringBuffer = new StringBuffer(strLen);
                stringBuffer.append(Character.toTitleCase(str.charAt(0)));
                stringBuffer.append(str.substring(1));
                return stringBuffer.toString();
            }
        }
        return str;
    }

    public static String capitalise(String str) {
        return capitalize(str);
    }

    public static String uncapitalize(String str) {
        if (str != null) {
            int length = str.length();
            int strLen = length;
            if (length != 0) {
                StringBuffer stringBuffer = new StringBuffer(strLen);
                stringBuffer.append(Character.toLowerCase(str.charAt(0)));
                stringBuffer.append(str.substring(1));
                return stringBuffer.toString();
            }
        }
        return str;
    }

    public static String uncapitalise(String str) {
        return uncapitalize(str);
    }

    public static String swapCase(String str) {
        if (str != null) {
            int length = str.length();
            int strLen = length;
            if (length != 0) {
                StringBuffer buffer = new StringBuffer(strLen);
                for (int i = 0; i < strLen; i++) {
                    char ch = str.charAt(i);
                    if (Character.isUpperCase(ch)) {
                        ch = Character.toLowerCase(ch);
                    } else if (Character.isTitleCase(ch)) {
                        ch = Character.toLowerCase(ch);
                    } else if (Character.isLowerCase(ch)) {
                        ch = Character.toUpperCase(ch);
                    }
                    buffer.append(ch);
                }
                return buffer.toString();
            }
        }
        return str;
    }

    public static String capitaliseAllWords(String str) {
        return WordUtils.capitalize(str);
    }

    public static int countMatches(String str, String sub) {
        int idx = 0;
        if (isEmpty(str) || isEmpty(sub)) {
            return 0;
        }
        int count = 0;
        while (true) {
            int indexOf = str.indexOf(sub, idx);
            int idx2 = indexOf;
            if (indexOf == -1) {
                return count;
            }
            count++;
            idx = idx2 + sub.length();
        }
    }

    public static boolean isAlpha(String str) {
        if (str == null) {
            return false;
        }
        int sz = str.length();
        for (int i = 0; i < sz; i++) {
            if (!Character.isLetter(str.charAt(i))) {
                return false;
            }
        }
        return true;
    }

    public static boolean isAlphaSpace(String str) {
        if (str == null) {
            return false;
        }
        int sz = str.length();
        for (int i = 0; i < sz; i++) {
            if (!Character.isLetter(str.charAt(i)) && str.charAt(i) != ' ') {
                return false;
            }
        }
        return true;
    }

    public static boolean isAlphanumeric(String str) {
        if (str == null) {
            return false;
        }
        int sz = str.length();
        for (int i = 0; i < sz; i++) {
            if (!Character.isLetterOrDigit(str.charAt(i))) {
                return false;
            }
        }
        return true;
    }

    public static boolean isAlphanumericSpace(String str) {
        if (str == null) {
            return false;
        }
        int sz = str.length();
        for (int i = 0; i < sz; i++) {
            if (!Character.isLetterOrDigit(str.charAt(i)) && str.charAt(i) != ' ') {
                return false;
            }
        }
        return true;
    }

    public static boolean isAsciiPrintable(String str) {
        if (str == null) {
            return false;
        }
        int sz = str.length();
        for (int i = 0; i < sz; i++) {
            if (!CharUtils.isAsciiPrintable(str.charAt(i))) {
                return false;
            }
        }
        return true;
    }

    public static boolean isNumeric(String str) {
        if (str == null) {
            return false;
        }
        int sz = str.length();
        for (int i = 0; i < sz; i++) {
            if (!Character.isDigit(str.charAt(i))) {
                return false;
            }
        }
        return true;
    }

    public static boolean isNumericSpace(String str) {
        if (str == null) {
            return false;
        }
        int sz = str.length();
        for (int i = 0; i < sz; i++) {
            if (!Character.isDigit(str.charAt(i)) && str.charAt(i) != ' ') {
                return false;
            }
        }
        return true;
    }

    public static boolean isWhitespace(String str) {
        if (str == null) {
            return false;
        }
        int sz = str.length();
        for (int i = 0; i < sz; i++) {
            if (!Character.isWhitespace(str.charAt(i))) {
                return false;
            }
        }
        return true;
    }

    public static String defaultString(String str) {
        return str == null ? "" : str;
    }

    public static String defaultString(String str, String defaultStr) {
        return str == null ? defaultStr : str;
    }

    public static String defaultIfEmpty(String str, String defaultStr) {
        return isEmpty(str) ? defaultStr : str;
    }

    public static String reverse(String str) {
        if (str == null) {
            return null;
        }
        return new StringBuffer(str).reverse().toString();
    }

    public static String reverseDelimited(String str, char separatorChar) {
        if (str == null) {
            return null;
        }
        String[] strs = split(str, separatorChar);
        ArrayUtils.reverse((Object[]) strs);
        return join((Object[]) strs, separatorChar);
    }

    public static String reverseDelimitedString(String str, String separatorChars) {
        if (str == null) {
            return null;
        }
        String[] strs = split(str, separatorChars);
        ArrayUtils.reverse((Object[]) strs);
        if (separatorChars == null) {
            return join((Object[]) strs, ' ');
        }
        return join((Object[]) strs, separatorChars);
    }

    public static String abbreviate(String str, int maxWidth) {
        return abbreviate(str, 0, maxWidth);
    }

    public static String abbreviate(String str, int offset, int maxWidth) {
        if (str == null) {
            return null;
        }
        if (maxWidth < 4) {
            throw new IllegalArgumentException("Minimum abbreviation width is 4");
        } else if (str.length() <= maxWidth) {
            return str;
        } else {
            if (offset > str.length()) {
                offset = str.length();
            }
            if (str.length() - offset < maxWidth - 3) {
                offset = str.length() - (maxWidth - 3);
            }
            if (offset <= 4) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(str.substring(0, maxWidth - 3));
                stringBuffer.append("...");
                return stringBuffer.toString();
            } else if (maxWidth < 7) {
                throw new IllegalArgumentException("Minimum abbreviation width with offset is 7");
            } else if ((maxWidth - 3) + offset < str.length()) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("...");
                stringBuffer2.append(abbreviate(str.substring(offset), maxWidth - 3));
                return stringBuffer2.toString();
            } else {
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("...");
                stringBuffer3.append(str.substring(str.length() - (maxWidth - 3)));
                return stringBuffer3.toString();
            }
        }
    }

    public static String difference(String str1, String str2) {
        if (str1 == null) {
            return str2;
        }
        if (str2 == null) {
            return str1;
        }
        int at = indexOfDifference(str1, str2);
        if (at == -1) {
            return "";
        }
        return str2.substring(at);
    }

    public static int indexOfDifference(String str1, String str2) {
        if (str1 == str2) {
            return -1;
        }
        int i = 0;
        if (str1 == null || str2 == null) {
            return 0;
        }
        while (i < str1.length() && i < str2.length() && str1.charAt(i) == str2.charAt(i)) {
            i++;
        }
        if (i < str2.length() || i < str1.length()) {
            return i;
        }
        return -1;
    }

    public static int indexOfDifference(String[] strs) {
        if (strs == null || strs.length <= 1) {
            return -1;
        }
        boolean allStringsNull = true;
        int arrayLen = strs.length;
        int longestStrLen = 0;
        int shortestStrLen = Integer.MAX_VALUE;
        boolean anyStringNull = false;
        for (int i = 0; i < arrayLen; i++) {
            if (strs[i] == null) {
                anyStringNull = true;
                shortestStrLen = 0;
            } else {
                allStringsNull = false;
                shortestStrLen = Math.min(strs[i].length(), shortestStrLen);
                longestStrLen = Math.max(strs[i].length(), longestStrLen);
            }
        }
        if (allStringsNull || (longestStrLen == 0 && !anyStringNull)) {
            return -1;
        }
        if (shortestStrLen == 0) {
            return 0;
        }
        int firstDiff = -1;
        for (int stringPos = 0; stringPos < shortestStrLen; stringPos++) {
            char comparisonChar = strs[0].charAt(stringPos);
            int arrayPos = 1;
            while (true) {
                if (arrayPos >= arrayLen) {
                    break;
                } else if (strs[arrayPos].charAt(stringPos) != comparisonChar) {
                    firstDiff = stringPos;
                    break;
                } else {
                    arrayPos++;
                }
            }
            if (firstDiff != -1) {
                break;
            }
        }
        if (firstDiff != -1 || shortestStrLen == longestStrLen) {
            return firstDiff;
        }
        return shortestStrLen;
    }

    public static String getCommonPrefix(String[] strs) {
        if (strs == null || strs.length == 0) {
            return "";
        }
        int smallestIndexOfDiff = indexOfDifference(strs);
        if (smallestIndexOfDiff == -1) {
            if (strs[0] == null) {
                return "";
            }
            return strs[0];
        } else if (smallestIndexOfDiff == 0) {
            return "";
        } else {
            return strs[0].substring(0, smallestIndexOfDiff);
        }
    }

    public static int getLevenshteinDistance(String s, String t) {
        if (s == null || t == null) {
            throw new IllegalArgumentException("Strings must not be null");
        }
        int n = s.length();
        int m = t.length();
        if (n == 0) {
            return m;
        }
        if (m == 0) {
            return n;
        }
        if (n > m) {
            String tmp = s;
            s = t;
            t = tmp;
            n = m;
            m = t.length();
        }
        int[] p = new int[(n + 1)];
        int[] d = new int[(n + 1)];
        for (int i = 0; i <= n; i++) {
            p[i] = i;
        }
        int[] p2 = p;
        for (int j = 1; j <= m; j++) {
            char t_j = t.charAt(j - 1);
            d[0] = j;
            for (int i2 = 1; i2 <= n; i2++) {
                d[i2] = Math.min(Math.min(d[i2 - 1] + 1, p2[i2] + 1), p2[i2 - 1] + (s.charAt(i2 + -1) == t_j ? 0 : 1));
            }
            int[] _d = p2;
            p2 = d;
            d = _d;
        }
        return p2[n];
    }

    public static boolean startsWith(String str, String prefix) {
        return startsWith(str, prefix, false);
    }

    public static boolean startsWithIgnoreCase(String str, String prefix) {
        return startsWith(str, prefix, true);
    }

    private static boolean startsWith(String str, String prefix, boolean ignoreCase) {
        if (str == null || prefix == null) {
            if (str == null && prefix == null) {
                return true;
            }
            return false;
        } else if (prefix.length() > str.length()) {
            return false;
        } else {
            return str.regionMatches(ignoreCase, 0, prefix, 0, prefix.length());
        }
    }

    public static boolean endsWith(String str, String suffix) {
        return endsWith(str, suffix, false);
    }

    public static boolean endsWithIgnoreCase(String str, String suffix) {
        return endsWith(str, suffix, true);
    }

    private static boolean endsWith(String str, String suffix, boolean ignoreCase) {
        if (str == null || suffix == null) {
            if (str == null && suffix == null) {
                return true;
            }
            return false;
        } else if (suffix.length() > str.length()) {
            return false;
        } else {
            return str.regionMatches(ignoreCase, str.length() - suffix.length(), suffix, 0, suffix.length());
        }
    }
}
