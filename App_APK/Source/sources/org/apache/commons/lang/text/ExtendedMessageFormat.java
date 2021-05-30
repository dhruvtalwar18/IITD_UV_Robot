package org.apache.commons.lang.text;

import java.text.Format;
import java.text.MessageFormat;
import java.text.ParsePosition;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Locale;
import java.util.Map;

public class ExtendedMessageFormat extends MessageFormat {
    private static final String DUMMY_PATTERN = "";
    private static final char END_FE = '}';
    private static final String ESCAPED_QUOTE = "''";
    private static final char QUOTE = '\'';
    private static final char START_FE = '{';
    private static final char START_FMT = ',';
    private static final long serialVersionUID = -2362048321261811743L;
    private Map registry;
    private String toPattern;

    public ExtendedMessageFormat(String pattern) {
        this(pattern, Locale.getDefault());
    }

    public ExtendedMessageFormat(String pattern, Locale locale) {
        this(pattern, locale, (Map) null);
    }

    public ExtendedMessageFormat(String pattern, Map registry2) {
        this(pattern, Locale.getDefault(), registry2);
    }

    public ExtendedMessageFormat(String pattern, Locale locale, Map registry2) {
        super("");
        setLocale(locale);
        this.registry = registry2;
        applyPattern(pattern);
    }

    public String toPattern() {
        return this.toPattern;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:13:0x006c, code lost:
        r11 = parseFormatDescription(r15, next(r3));
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public final void applyPattern(java.lang.String r15) {
        /*
            r14 = this;
            java.util.Map r0 = r14.registry
            if (r0 != 0) goto L_0x000e
            super.applyPattern(r15)
            java.lang.String r0 = super.toPattern()
            r14.toPattern = r0
            return
        L_0x000e:
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            java.util.ArrayList r1 = new java.util.ArrayList
            r1.<init>()
            java.lang.StringBuffer r2 = new java.lang.StringBuffer
            int r3 = r15.length()
            r2.<init>(r3)
            java.text.ParsePosition r3 = new java.text.ParsePosition
            r4 = 0
            r3.<init>(r4)
            char[] r5 = r15.toCharArray()
            r6 = 0
        L_0x002c:
            int r7 = r3.getIndex()
            int r8 = r15.length()
            if (r7 >= r8) goto L_0x00d6
            int r7 = r3.getIndex()
            char r7 = r5[r7]
            r8 = 39
            r9 = 1
            if (r7 == r8) goto L_0x00d1
            r8 = 123(0x7b, float:1.72E-43)
            if (r7 == r8) goto L_0x0046
            goto L_0x00ac
        L_0x0046:
            int r6 = r6 + 1
            r14.seekNonWs(r15, r3)
            int r7 = r3.getIndex()
            java.text.ParsePosition r10 = r14.next(r3)
            int r10 = r14.readArgumentIndex(r15, r10)
            r2.append(r8)
            r2.append(r10)
            r14.seekNonWs(r15, r3)
            r8 = 0
            r11 = 0
            int r12 = r3.getIndex()
            char r12 = r5[r12]
            r13 = 44
            if (r12 != r13) goto L_0x0080
            java.text.ParsePosition r12 = r14.next(r3)
            java.lang.String r11 = r14.parseFormatDescription(r15, r12)
            java.text.Format r8 = r14.getFormat(r11)
            if (r8 != 0) goto L_0x0080
            r2.append(r13)
            r2.append(r11)
        L_0x0080:
            r0.add(r8)
            if (r8 != 0) goto L_0x0087
            r12 = 0
            goto L_0x0088
        L_0x0087:
            r12 = r11
        L_0x0088:
            r1.add(r12)
            int r12 = r0.size()
            if (r12 != r6) goto L_0x0093
            r12 = 1
            goto L_0x0094
        L_0x0093:
            r12 = 0
        L_0x0094:
            org.apache.commons.lang.Validate.isTrue(r12)
            int r12 = r1.size()
            if (r12 != r6) goto L_0x009e
            goto L_0x009f
        L_0x009e:
            r9 = 0
        L_0x009f:
            org.apache.commons.lang.Validate.isTrue(r9)
            int r9 = r3.getIndex()
            char r9 = r5[r9]
            r12 = 125(0x7d, float:1.75E-43)
            if (r9 != r12) goto L_0x00ba
        L_0x00ac:
            int r7 = r3.getIndex()
            char r7 = r5[r7]
            r2.append(r7)
            r14.next(r3)
            goto L_0x002c
        L_0x00ba:
            java.lang.IllegalArgumentException r4 = new java.lang.IllegalArgumentException
            java.lang.StringBuffer r9 = new java.lang.StringBuffer
            r9.<init>()
            java.lang.String r12 = "Unreadable format element at position "
            r9.append(r12)
            r9.append(r7)
            java.lang.String r9 = r9.toString()
            r4.<init>(r9)
            throw r4
        L_0x00d1:
            r14.appendQuotedString(r15, r3, r2, r9)
            goto L_0x002c
        L_0x00d6:
            java.lang.String r4 = r2.toString()
            super.applyPattern(r4)
            java.lang.String r4 = super.toPattern()
            java.lang.String r4 = r14.insertFormats(r4, r1)
            r14.toPattern = r4
            boolean r4 = r14.containsElements(r0)
            if (r4 == 0) goto L_0x010c
            java.text.Format[] r4 = r14.getFormats()
            r7 = 0
            java.util.Iterator r8 = r0.iterator()
        L_0x00f6:
            boolean r9 = r8.hasNext()
            if (r9 == 0) goto L_0x0109
            java.lang.Object r9 = r8.next()
            java.text.Format r9 = (java.text.Format) r9
            if (r9 == 0) goto L_0x0106
            r4[r7] = r9
        L_0x0106:
            int r7 = r7 + 1
            goto L_0x00f6
        L_0x0109:
            super.setFormats(r4)
        L_0x010c:
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.text.ExtendedMessageFormat.applyPattern(java.lang.String):void");
    }

    public void setFormat(int formatElementIndex, Format newFormat) {
        throw new UnsupportedOperationException();
    }

    public void setFormatByArgumentIndex(int argumentIndex, Format newFormat) {
        throw new UnsupportedOperationException();
    }

    public void setFormats(Format[] newFormats) {
        throw new UnsupportedOperationException();
    }

    public void setFormatsByArgumentIndex(Format[] newFormats) {
        throw new UnsupportedOperationException();
    }

    private Format getFormat(String desc) {
        if (this.registry == null) {
            return null;
        }
        String name = desc;
        String args = null;
        int i = desc.indexOf(44);
        if (i > 0) {
            name = desc.substring(0, i).trim();
            args = desc.substring(i + 1).trim();
        }
        FormatFactory factory = (FormatFactory) this.registry.get(name);
        if (factory != null) {
            return factory.getFormat(name, args, getLocale());
        }
        return null;
    }

    private int readArgumentIndex(String pattern, ParsePosition pos) {
        int start = pos.getIndex();
        seekNonWs(pattern, pos);
        StringBuffer result = new StringBuffer();
        boolean error = false;
        while (!error && pos.getIndex() < pattern.length()) {
            char c = pattern.charAt(pos.getIndex());
            if (Character.isWhitespace(c)) {
                seekNonWs(pattern, pos);
                c = pattern.charAt(pos.getIndex());
                if (!(c == ',' || c == '}')) {
                    error = true;
                    next(pos);
                }
            }
            if ((c == ',' || c == '}') && result.length() > 0) {
                try {
                    return Integer.parseInt(result.toString());
                } catch (NumberFormatException e) {
                }
            }
            error = !Character.isDigit(c);
            result.append(c);
            next(pos);
        }
        if (error) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid format argument index at position ");
            stringBuffer.append(start);
            stringBuffer.append(": ");
            stringBuffer.append(pattern.substring(start, pos.getIndex()));
            throw new IllegalArgumentException(stringBuffer.toString());
        }
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append("Unterminated format element at position ");
        stringBuffer2.append(start);
        throw new IllegalArgumentException(stringBuffer2.toString());
    }

    private String parseFormatDescription(String pattern, ParsePosition pos) {
        int start = pos.getIndex();
        seekNonWs(pattern, pos);
        int text = pos.getIndex();
        int depth = 1;
        while (pos.getIndex() < pattern.length()) {
            char charAt = pattern.charAt(pos.getIndex());
            if (charAt == '\'') {
                getQuotedString(pattern, pos, false);
            } else if (charAt == '{') {
                depth++;
            } else if (charAt == '}' && depth - 1 == 0) {
                return pattern.substring(text, pos.getIndex());
            }
            next(pos);
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Unterminated format element at position ");
        stringBuffer.append(start);
        throw new IllegalArgumentException(stringBuffer.toString());
    }

    private String insertFormats(String pattern, ArrayList customPatterns) {
        if (!containsElements(customPatterns)) {
            return pattern;
        }
        StringBuffer sb = new StringBuffer(pattern.length() * 2);
        ParsePosition pos = new ParsePosition(0);
        int fe = -1;
        int depth = 0;
        while (pos.getIndex() < pattern.length()) {
            char c = pattern.charAt(pos.getIndex());
            if (c == '\'') {
                appendQuotedString(pattern, pos, sb, false);
            } else if (c != '{') {
                if (c == '}') {
                    depth--;
                }
                sb.append(c);
                next(pos);
            } else {
                depth++;
                if (depth == 1) {
                    fe++;
                    sb.append(START_FE);
                    sb.append(readArgumentIndex(pattern, next(pos)));
                    String customPattern = (String) customPatterns.get(fe);
                    if (customPattern != null) {
                        sb.append(START_FMT);
                        sb.append(customPattern);
                    }
                }
            }
        }
        return sb.toString();
    }

    private void seekNonWs(String pattern, ParsePosition pos) {
        char[] buffer = pattern.toCharArray();
        do {
            int len = StrMatcher.splitMatcher().isMatch(buffer, pos.getIndex());
            pos.setIndex(pos.getIndex() + len);
            if (len <= 0 || pos.getIndex() >= pattern.length()) {
            }
            int len2 = StrMatcher.splitMatcher().isMatch(buffer, pos.getIndex());
            pos.setIndex(pos.getIndex() + len2);
            return;
        } while (pos.getIndex() >= pattern.length());
    }

    private ParsePosition next(ParsePosition pos) {
        pos.setIndex(pos.getIndex() + 1);
        return pos;
    }

    private StringBuffer appendQuotedString(String pattern, ParsePosition pos, StringBuffer appendTo, boolean escapingOn) {
        int start = pos.getIndex();
        char[] c = pattern.toCharArray();
        if (!escapingOn || c[start] != '\'') {
            int lastHold = start;
            for (int i = pos.getIndex(); i < pattern.length(); i++) {
                if (escapingOn && pattern.substring(i).startsWith(ESCAPED_QUOTE)) {
                    appendTo.append(c, lastHold, pos.getIndex() - lastHold);
                    appendTo.append(QUOTE);
                    pos.setIndex(ESCAPED_QUOTE.length() + i);
                    lastHold = pos.getIndex();
                } else if (c[pos.getIndex()] != '\'') {
                    next(pos);
                } else {
                    next(pos);
                    if (appendTo == null) {
                        return null;
                    }
                    appendTo.append(c, lastHold, pos.getIndex() - lastHold);
                    return appendTo;
                }
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Unterminated quoted string at position ");
            stringBuffer.append(start);
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (appendTo == null) {
            return null;
        } else {
            appendTo.append(QUOTE);
            return appendTo;
        }
    }

    private void getQuotedString(String pattern, ParsePosition pos, boolean escapingOn) {
        appendQuotedString(pattern, pos, (StringBuffer) null, escapingOn);
    }

    private boolean containsElements(Collection coll) {
        if (coll == null || coll.size() == 0) {
            return false;
        }
        for (Object obj : coll) {
            if (obj != null) {
                return true;
            }
        }
        return false;
    }
}
