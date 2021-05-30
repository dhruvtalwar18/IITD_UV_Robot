package org.apache.commons.lang.time;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.text.DateFormat;
import java.text.FieldPosition;
import java.text.Format;
import java.text.ParsePosition;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TimeZone;
import org.apache.commons.lang.Validate;

public class FastDateFormat extends Format {
    public static final int FULL = 0;
    public static final int LONG = 1;
    public static final int MEDIUM = 2;
    public static final int SHORT = 3;
    private static final Map cDateInstanceCache = new HashMap(7);
    private static final Map cDateTimeInstanceCache = new HashMap(7);
    private static String cDefaultPattern = null;
    private static final Map cInstanceCache = new HashMap(7);
    private static final Map cTimeInstanceCache = new HashMap(7);
    private static final Map cTimeZoneDisplayCache = new HashMap(7);
    private static final long serialVersionUID = 1;
    private final Locale mLocale;
    private final boolean mLocaleForced;
    private transient int mMaxLengthEstimate;
    private final String mPattern;
    private transient Rule[] mRules;
    private final TimeZone mTimeZone;
    private final boolean mTimeZoneForced;

    private interface NumberRule extends Rule {
        void appendTo(StringBuffer stringBuffer, int i);
    }

    private interface Rule {
        void appendTo(StringBuffer stringBuffer, Calendar calendar);

        int estimateLength();
    }

    public static FastDateFormat getInstance() {
        return getInstance(getDefaultPattern(), (TimeZone) null, (Locale) null);
    }

    public static FastDateFormat getInstance(String pattern) {
        return getInstance(pattern, (TimeZone) null, (Locale) null);
    }

    public static FastDateFormat getInstance(String pattern, TimeZone timeZone) {
        return getInstance(pattern, timeZone, (Locale) null);
    }

    public static FastDateFormat getInstance(String pattern, Locale locale) {
        return getInstance(pattern, (TimeZone) null, locale);
    }

    public static synchronized FastDateFormat getInstance(String pattern, TimeZone timeZone, Locale locale) {
        FastDateFormat format;
        synchronized (FastDateFormat.class) {
            FastDateFormat emptyFormat = new FastDateFormat(pattern, timeZone, locale);
            format = (FastDateFormat) cInstanceCache.get(emptyFormat);
            if (format == null) {
                format = emptyFormat;
                format.init();
                cInstanceCache.put(format, format);
            }
        }
        return format;
    }

    public static FastDateFormat getDateInstance(int style) {
        return getDateInstance(style, (TimeZone) null, (Locale) null);
    }

    public static FastDateFormat getDateInstance(int style, Locale locale) {
        return getDateInstance(style, (TimeZone) null, locale);
    }

    public static FastDateFormat getDateInstance(int style, TimeZone timeZone) {
        return getDateInstance(style, timeZone, (Locale) null);
    }

    public static synchronized FastDateFormat getDateInstance(int style, TimeZone timeZone, Locale locale) {
        FastDateFormat format;
        synchronized (FastDateFormat.class) {
            Object key = new Integer(style);
            if (timeZone != null) {
                key = new Pair(key, timeZone);
            }
            if (locale == null) {
                locale = Locale.getDefault();
            }
            Object key2 = new Pair(key, locale);
            format = (FastDateFormat) cDateInstanceCache.get(key2);
            if (format == null) {
                try {
                    format = getInstance(((SimpleDateFormat) DateFormat.getDateInstance(style, locale)).toPattern(), timeZone, locale);
                    cDateInstanceCache.put(key2, format);
                } catch (ClassCastException e) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("No date pattern for locale: ");
                    stringBuffer.append(locale);
                    throw new IllegalArgumentException(stringBuffer.toString());
                }
            }
        }
        return format;
    }

    public static FastDateFormat getTimeInstance(int style) {
        return getTimeInstance(style, (TimeZone) null, (Locale) null);
    }

    public static FastDateFormat getTimeInstance(int style, Locale locale) {
        return getTimeInstance(style, (TimeZone) null, locale);
    }

    public static FastDateFormat getTimeInstance(int style, TimeZone timeZone) {
        return getTimeInstance(style, timeZone, (Locale) null);
    }

    public static synchronized FastDateFormat getTimeInstance(int style, TimeZone timeZone, Locale locale) {
        FastDateFormat format;
        synchronized (FastDateFormat.class) {
            Object key = new Integer(style);
            if (timeZone != null) {
                key = new Pair(key, timeZone);
            }
            if (locale != null) {
                key = new Pair(key, locale);
            }
            format = (FastDateFormat) cTimeInstanceCache.get(key);
            if (format == null) {
                if (locale == null) {
                    locale = Locale.getDefault();
                }
                try {
                    format = getInstance(((SimpleDateFormat) DateFormat.getTimeInstance(style, locale)).toPattern(), timeZone, locale);
                    cTimeInstanceCache.put(key, format);
                } catch (ClassCastException e) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("No date pattern for locale: ");
                    stringBuffer.append(locale);
                    throw new IllegalArgumentException(stringBuffer.toString());
                }
            }
        }
        return format;
    }

    public static FastDateFormat getDateTimeInstance(int dateStyle, int timeStyle) {
        return getDateTimeInstance(dateStyle, timeStyle, (TimeZone) null, (Locale) null);
    }

    public static FastDateFormat getDateTimeInstance(int dateStyle, int timeStyle, Locale locale) {
        return getDateTimeInstance(dateStyle, timeStyle, (TimeZone) null, locale);
    }

    public static FastDateFormat getDateTimeInstance(int dateStyle, int timeStyle, TimeZone timeZone) {
        return getDateTimeInstance(dateStyle, timeStyle, timeZone, (Locale) null);
    }

    public static synchronized FastDateFormat getDateTimeInstance(int dateStyle, int timeStyle, TimeZone timeZone, Locale locale) {
        FastDateFormat format;
        synchronized (FastDateFormat.class) {
            Object key = new Pair(new Integer(dateStyle), new Integer(timeStyle));
            if (timeZone != null) {
                key = new Pair(key, timeZone);
            }
            if (locale == null) {
                locale = Locale.getDefault();
            }
            Object key2 = new Pair(key, locale);
            format = (FastDateFormat) cDateTimeInstanceCache.get(key2);
            if (format == null) {
                try {
                    format = getInstance(((SimpleDateFormat) DateFormat.getDateTimeInstance(dateStyle, timeStyle, locale)).toPattern(), timeZone, locale);
                    cDateTimeInstanceCache.put(key2, format);
                } catch (ClassCastException e) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("No date time pattern for locale: ");
                    stringBuffer.append(locale);
                    throw new IllegalArgumentException(stringBuffer.toString());
                }
            }
        }
        return format;
    }

    static synchronized String getTimeZoneDisplay(TimeZone tz, boolean daylight, int style, Locale locale) {
        String value;
        synchronized (FastDateFormat.class) {
            Object key = new TimeZoneDisplayKey(tz, daylight, style, locale);
            value = (String) cTimeZoneDisplayCache.get(key);
            if (value == null) {
                value = tz.getDisplayName(daylight, style, locale);
                cTimeZoneDisplayCache.put(key, value);
            }
        }
        return value;
    }

    private static synchronized String getDefaultPattern() {
        String str;
        synchronized (FastDateFormat.class) {
            if (cDefaultPattern == null) {
                cDefaultPattern = new SimpleDateFormat().toPattern();
            }
            str = cDefaultPattern;
        }
        return str;
    }

    protected FastDateFormat(String pattern, TimeZone timeZone, Locale locale) {
        if (pattern != null) {
            this.mPattern = pattern;
            boolean z = false;
            this.mTimeZoneForced = timeZone != null;
            this.mTimeZone = timeZone == null ? TimeZone.getDefault() : timeZone;
            this.mLocaleForced = locale != null ? true : z;
            this.mLocale = locale == null ? Locale.getDefault() : locale;
            return;
        }
        throw new IllegalArgumentException("The pattern must not be null");
    }

    /* access modifiers changed from: protected */
    public void init() {
        List rulesList = parsePattern();
        this.mRules = (Rule[]) rulesList.toArray(new Rule[rulesList.size()]);
        int len = 0;
        int i = this.mRules.length;
        while (true) {
            i--;
            if (i >= 0) {
                len += this.mRules[i].estimateLength();
            } else {
                this.mMaxLengthEstimate = len;
                return;
            }
        }
    }

    /* access modifiers changed from: protected */
    /* JADX WARNING: Can't fix incorrect switch cases order */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.util.List parsePattern() {
        /*
            r20 = this;
            r0 = r20
            java.text.DateFormatSymbols r1 = new java.text.DateFormatSymbols
            java.util.Locale r2 = r0.mLocale
            r1.<init>(r2)
            java.util.ArrayList r2 = new java.util.ArrayList
            r2.<init>()
            java.lang.String[] r3 = r1.getEras()
            java.lang.String[] r4 = r1.getMonths()
            java.lang.String[] r5 = r1.getShortMonths()
            java.lang.String[] r6 = r1.getWeekdays()
            java.lang.String[] r7 = r1.getShortWeekdays()
            java.lang.String[] r8 = r1.getAmPmStrings()
            java.lang.String r9 = r0.mPattern
            int r9 = r9.length()
            r10 = 1
            int[] r11 = new int[r10]
            r12 = 0
            r13 = 0
        L_0x0031:
            if (r13 >= r9) goto L_0x01f4
            r11[r12] = r13
            java.lang.String r14 = r0.mPattern
            java.lang.String r14 = r0.parseToken(r14, r11)
            r13 = r11[r12]
            int r15 = r14.length()
            if (r15 != 0) goto L_0x004c
            r17 = r1
            r18 = r6
            r19 = r7
            goto L_0x01fa
        L_0x004c:
            char r16 = r14.charAt(r12)
            r12 = 4
            switch(r16) {
                case 39: goto L_0x01c2;
                case 68: goto L_0x01b2;
                case 69: goto L_0x019e;
                case 70: goto L_0x0191;
                case 71: goto L_0x0184;
                case 72: goto L_0x0177;
                case 75: goto L_0x016a;
                case 77: goto L_0x0146;
                case 83: goto L_0x0138;
                case 87: goto L_0x012c;
                case 90: goto L_0x011b;
                case 97: goto L_0x010c;
                case 100: goto L_0x00ff;
                case 104: goto L_0x00ec;
                case 107: goto L_0x00d9;
                case 109: goto L_0x00cb;
                case 115: goto L_0x00bd;
                case 119: goto L_0x00b0;
                case 121: goto L_0x009c;
                case 122: goto L_0x0071;
                default: goto L_0x0054;
            }
        L_0x0054:
            r17 = r1
            r18 = r6
            r19 = r7
            java.lang.IllegalArgumentException r1 = new java.lang.IllegalArgumentException
            java.lang.StringBuffer r6 = new java.lang.StringBuffer
            r6.<init>()
            java.lang.String r7 = "Illegal pattern component: "
            r6.append(r7)
            r6.append(r14)
            java.lang.String r6 = r6.toString()
            r1.<init>(r6)
            throw r1
        L_0x0071:
            if (r15 < r12) goto L_0x0088
            org.apache.commons.lang.time.FastDateFormat$TimeZoneNameRule r12 = new org.apache.commons.lang.time.FastDateFormat$TimeZoneNameRule
            java.util.TimeZone r10 = r0.mTimeZone
            r17 = r1
            boolean r1 = r0.mTimeZoneForced
            r18 = r6
            java.util.Locale r6 = r0.mLocale
            r19 = r7
            r7 = 1
            r12.<init>(r10, r1, r6, r7)
            r1 = r12
            goto L_0x01be
        L_0x0088:
            r17 = r1
            r18 = r6
            r19 = r7
            org.apache.commons.lang.time.FastDateFormat$TimeZoneNameRule r1 = new org.apache.commons.lang.time.FastDateFormat$TimeZoneNameRule
            java.util.TimeZone r6 = r0.mTimeZone
            boolean r7 = r0.mTimeZoneForced
            java.util.Locale r10 = r0.mLocale
            r12 = 0
            r1.<init>(r6, r7, r10, r12)
            goto L_0x01be
        L_0x009c:
            r17 = r1
            r18 = r6
            r19 = r7
            if (r15 < r12) goto L_0x00ac
            r1 = 1
            org.apache.commons.lang.time.FastDateFormat$NumberRule r6 = r0.selectNumberRule(r1, r15)
            r7 = r6
            goto L_0x01c0
        L_0x00ac:
            org.apache.commons.lang.time.FastDateFormat$TwoDigitYearField r1 = org.apache.commons.lang.time.FastDateFormat.TwoDigitYearField.INSTANCE
            goto L_0x01be
        L_0x00b0:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 3
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r1, r15)
            goto L_0x01be
        L_0x00bd:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 13
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r1, r15)
            goto L_0x01be
        L_0x00cb:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 12
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r1, r15)
            goto L_0x01be
        L_0x00d9:
            r17 = r1
            r18 = r6
            r19 = r7
            org.apache.commons.lang.time.FastDateFormat$TwentyFourHourField r1 = new org.apache.commons.lang.time.FastDateFormat$TwentyFourHourField
            r6 = 11
            org.apache.commons.lang.time.FastDateFormat$NumberRule r6 = r0.selectNumberRule(r6, r15)
            r1.<init>(r6)
            goto L_0x01be
        L_0x00ec:
            r17 = r1
            r18 = r6
            r19 = r7
            org.apache.commons.lang.time.FastDateFormat$TwelveHourField r1 = new org.apache.commons.lang.time.FastDateFormat$TwelveHourField
            r6 = 10
            org.apache.commons.lang.time.FastDateFormat$NumberRule r6 = r0.selectNumberRule(r6, r15)
            r1.<init>(r6)
            goto L_0x01be
        L_0x00ff:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 5
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r1, r15)
            goto L_0x01be
        L_0x010c:
            r17 = r1
            r18 = r6
            r19 = r7
            org.apache.commons.lang.time.FastDateFormat$TextField r1 = new org.apache.commons.lang.time.FastDateFormat$TextField
            r6 = 9
            r1.<init>(r6, r8)
            goto L_0x01be
        L_0x011b:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 1
            if (r15 != r1) goto L_0x0128
            org.apache.commons.lang.time.FastDateFormat$TimeZoneNumberRule r1 = org.apache.commons.lang.time.FastDateFormat.TimeZoneNumberRule.INSTANCE_NO_COLON
            goto L_0x01be
        L_0x0128:
            org.apache.commons.lang.time.FastDateFormat$TimeZoneNumberRule r1 = org.apache.commons.lang.time.FastDateFormat.TimeZoneNumberRule.INSTANCE_COLON
            goto L_0x01be
        L_0x012c:
            r17 = r1
            r18 = r6
            r19 = r7
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r12, r15)
            goto L_0x01be
        L_0x0138:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 14
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r1, r15)
            goto L_0x01be
        L_0x0146:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 2
            if (r15 < r12) goto L_0x0157
            org.apache.commons.lang.time.FastDateFormat$TextField r6 = new org.apache.commons.lang.time.FastDateFormat$TextField
            r6.<init>(r1, r4)
            r1 = r6
            goto L_0x01be
        L_0x0157:
            r6 = 3
            if (r15 != r6) goto L_0x0162
            org.apache.commons.lang.time.FastDateFormat$TextField r6 = new org.apache.commons.lang.time.FastDateFormat$TextField
            r6.<init>(r1, r5)
            r1 = r6
            goto L_0x01be
        L_0x0162:
            if (r15 != r1) goto L_0x0167
            org.apache.commons.lang.time.FastDateFormat$TwoDigitMonthField r1 = org.apache.commons.lang.time.FastDateFormat.TwoDigitMonthField.INSTANCE
            goto L_0x01be
        L_0x0167:
            org.apache.commons.lang.time.FastDateFormat$UnpaddedMonthField r1 = org.apache.commons.lang.time.FastDateFormat.UnpaddedMonthField.INSTANCE
            goto L_0x01be
        L_0x016a:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 10
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r1, r15)
            goto L_0x01be
        L_0x0177:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 11
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r1, r15)
            goto L_0x01be
        L_0x0184:
            r17 = r1
            r18 = r6
            r19 = r7
            org.apache.commons.lang.time.FastDateFormat$TextField r1 = new org.apache.commons.lang.time.FastDateFormat$TextField
            r6 = 0
            r1.<init>(r6, r3)
            goto L_0x01be
        L_0x0191:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 8
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r1, r15)
            goto L_0x01be
        L_0x019e:
            r17 = r1
            r18 = r6
            r19 = r7
            org.apache.commons.lang.time.FastDateFormat$TextField r1 = new org.apache.commons.lang.time.FastDateFormat$TextField
            r6 = 7
            if (r15 >= r12) goto L_0x01ac
            r7 = r19
            goto L_0x01ae
        L_0x01ac:
            r7 = r18
        L_0x01ae:
            r1.<init>(r6, r7)
            goto L_0x01be
        L_0x01b2:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 6
            org.apache.commons.lang.time.FastDateFormat$NumberRule r1 = r0.selectNumberRule(r1, r15)
        L_0x01be:
            r7 = r1
            r1 = 1
        L_0x01c0:
            r10 = 0
            goto L_0x01e5
        L_0x01c2:
            r17 = r1
            r18 = r6
            r19 = r7
            r1 = 1
            java.lang.String r6 = r14.substring(r1)
            int r7 = r6.length()
            if (r7 != r1) goto L_0x01de
            org.apache.commons.lang.time.FastDateFormat$CharacterLiteral r7 = new org.apache.commons.lang.time.FastDateFormat$CharacterLiteral
            r10 = 0
            char r12 = r6.charAt(r10)
            r7.<init>(r12)
            goto L_0x01e5
        L_0x01de:
            r10 = 0
            org.apache.commons.lang.time.FastDateFormat$StringLiteral r7 = new org.apache.commons.lang.time.FastDateFormat$StringLiteral
            r7.<init>(r6)
        L_0x01e5:
            r2.add(r7)
            int r13 = r13 + 1
            r1 = r17
            r6 = r18
            r7 = r19
            r10 = 1
            r12 = 0
            goto L_0x0031
        L_0x01f4:
            r17 = r1
            r18 = r6
            r19 = r7
        L_0x01fa:
            return r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.time.FastDateFormat.parsePattern():java.util.List");
    }

    /* access modifiers changed from: protected */
    public String parseToken(String pattern, int[] indexRef) {
        int i;
        StringBuffer buf = new StringBuffer();
        int i2 = indexRef[0];
        int length = pattern.length();
        char c = pattern.charAt(i2);
        if ((c < 'A' || c > 'Z') && (c < 'a' || c > 'z')) {
            buf.append('\'');
            char c2 = c;
            i = i2;
            boolean inLiteral = false;
            while (true) {
                if (i >= length) {
                    break;
                }
                char c3 = pattern.charAt(i);
                if (c3 == '\'') {
                    if (i + 1 >= length || pattern.charAt(i + 1) != '\'') {
                        inLiteral = !inLiteral;
                    } else {
                        i++;
                        buf.append(c3);
                    }
                } else if (inLiteral || ((c3 < 'A' || c3 > 'Z') && (c3 < 'a' || c3 > 'z'))) {
                    buf.append(c3);
                }
                i++;
            }
            i--;
        } else {
            buf.append(c);
            while (i2 + 1 < length && pattern.charAt(i2 + 1) == c) {
                buf.append(c);
                i2++;
            }
            char c4 = c;
            i = i2;
        }
        indexRef[0] = i;
        return buf.toString();
    }

    /* access modifiers changed from: protected */
    public NumberRule selectNumberRule(int field, int padding) {
        switch (padding) {
            case 1:
                return new UnpaddedNumberField(field);
            case 2:
                return new TwoDigitNumberField(field);
            default:
                return new PaddedNumberField(field, padding);
        }
    }

    public StringBuffer format(Object obj, StringBuffer toAppendTo, FieldPosition pos) {
        if (obj instanceof Date) {
            return format((Date) obj, toAppendTo);
        }
        if (obj instanceof Calendar) {
            return format((Calendar) obj, toAppendTo);
        }
        if (obj instanceof Long) {
            return format(((Long) obj).longValue(), toAppendTo);
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Unknown class: ");
        stringBuffer.append(obj == null ? "<null>" : obj.getClass().getName());
        throw new IllegalArgumentException(stringBuffer.toString());
    }

    public String format(long millis) {
        return format(new Date(millis));
    }

    public String format(Date date) {
        Calendar c = new GregorianCalendar(this.mTimeZone);
        c.setTime(date);
        return applyRules(c, new StringBuffer(this.mMaxLengthEstimate)).toString();
    }

    public String format(Calendar calendar) {
        return format(calendar, new StringBuffer(this.mMaxLengthEstimate)).toString();
    }

    public StringBuffer format(long millis, StringBuffer buf) {
        return format(new Date(millis), buf);
    }

    public StringBuffer format(Date date, StringBuffer buf) {
        Calendar c = new GregorianCalendar(this.mTimeZone);
        c.setTime(date);
        return applyRules(c, buf);
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r0v2, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r2v2, resolved type: java.util.Calendar} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.lang.StringBuffer format(java.util.Calendar r2, java.lang.StringBuffer r3) {
        /*
            r1 = this;
            boolean r0 = r1.mTimeZoneForced
            if (r0 == 0) goto L_0x0010
            java.lang.Object r0 = r2.clone()
            r2 = r0
            java.util.Calendar r2 = (java.util.Calendar) r2
            java.util.TimeZone r0 = r1.mTimeZone
            r2.setTimeZone(r0)
        L_0x0010:
            java.lang.StringBuffer r0 = r1.applyRules(r2, r3)
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.time.FastDateFormat.format(java.util.Calendar, java.lang.StringBuffer):java.lang.StringBuffer");
    }

    /* access modifiers changed from: protected */
    public StringBuffer applyRules(Calendar calendar, StringBuffer buf) {
        Rule[] rules = this.mRules;
        int len = this.mRules.length;
        for (int i = 0; i < len; i++) {
            rules[i].appendTo(buf, calendar);
        }
        return buf;
    }

    public Object parseObject(String source, ParsePosition pos) {
        pos.setIndex(0);
        pos.setErrorIndex(0);
        return null;
    }

    public String getPattern() {
        return this.mPattern;
    }

    public TimeZone getTimeZone() {
        return this.mTimeZone;
    }

    public boolean getTimeZoneOverridesCalendar() {
        return this.mTimeZoneForced;
    }

    public Locale getLocale() {
        return this.mLocale;
    }

    public int getMaxLengthEstimate() {
        return this.mMaxLengthEstimate;
    }

    public boolean equals(Object obj) {
        if (!(obj instanceof FastDateFormat)) {
            return false;
        }
        FastDateFormat other = (FastDateFormat) obj;
        if ((this.mPattern == other.mPattern || this.mPattern.equals(other.mPattern)) && ((this.mTimeZone == other.mTimeZone || this.mTimeZone.equals(other.mTimeZone)) && ((this.mLocale == other.mLocale || this.mLocale.equals(other.mLocale)) && this.mTimeZoneForced == other.mTimeZoneForced && this.mLocaleForced == other.mLocaleForced))) {
            return true;
        }
        return false;
    }

    public int hashCode() {
        return 0 + this.mPattern.hashCode() + this.mTimeZone.hashCode() + (this.mTimeZoneForced ? 1 : 0) + this.mLocale.hashCode() + (this.mLocaleForced ? 1 : 0);
    }

    public String toString() {
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("FastDateFormat[");
        stringBuffer.append(this.mPattern);
        stringBuffer.append("]");
        return stringBuffer.toString();
    }

    private void readObject(ObjectInputStream in) throws IOException, ClassNotFoundException {
        in.defaultReadObject();
        init();
    }

    private static class CharacterLiteral implements Rule {
        private final char mValue;

        CharacterLiteral(char value) {
            this.mValue = value;
        }

        public int estimateLength() {
            return 1;
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            buffer.append(this.mValue);
        }
    }

    private static class StringLiteral implements Rule {
        private final String mValue;

        StringLiteral(String value) {
            this.mValue = value;
        }

        public int estimateLength() {
            return this.mValue.length();
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            buffer.append(this.mValue);
        }
    }

    private static class TextField implements Rule {
        private final int mField;
        private final String[] mValues;

        TextField(int field, String[] values) {
            this.mField = field;
            this.mValues = values;
        }

        public int estimateLength() {
            int max = 0;
            int i = this.mValues.length;
            while (true) {
                i--;
                if (i < 0) {
                    return max;
                }
                int len = this.mValues[i].length();
                if (len > max) {
                    max = len;
                }
            }
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            buffer.append(this.mValues[calendar.get(this.mField)]);
        }
    }

    private static class UnpaddedNumberField implements NumberRule {
        static final UnpaddedNumberField INSTANCE_YEAR = new UnpaddedNumberField(1);
        private final int mField;

        UnpaddedNumberField(int field) {
            this.mField = field;
        }

        public int estimateLength() {
            return 4;
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            appendTo(buffer, calendar.get(this.mField));
        }

        public final void appendTo(StringBuffer buffer, int value) {
            if (value < 10) {
                buffer.append((char) (value + 48));
            } else if (value < 100) {
                buffer.append((char) ((value / 10) + 48));
                buffer.append((char) ((value % 10) + 48));
            } else {
                buffer.append(Integer.toString(value));
            }
        }
    }

    private static class UnpaddedMonthField implements NumberRule {
        static final UnpaddedMonthField INSTANCE = new UnpaddedMonthField();

        UnpaddedMonthField() {
        }

        public int estimateLength() {
            return 2;
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            appendTo(buffer, calendar.get(2) + 1);
        }

        public final void appendTo(StringBuffer buffer, int value) {
            if (value < 10) {
                buffer.append((char) (value + 48));
                return;
            }
            buffer.append((char) ((value / 10) + 48));
            buffer.append((char) ((value % 10) + 48));
        }
    }

    private static class PaddedNumberField implements NumberRule {
        private final int mField;
        private final int mSize;

        PaddedNumberField(int field, int size) {
            if (size >= 3) {
                this.mField = field;
                this.mSize = size;
                return;
            }
            throw new IllegalArgumentException();
        }

        public int estimateLength() {
            return 4;
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            appendTo(buffer, calendar.get(this.mField));
        }

        public final void appendTo(StringBuffer buffer, int value) {
            int digits;
            if (value < 100) {
                int i = this.mSize;
                while (true) {
                    i--;
                    if (i >= 2) {
                        buffer.append('0');
                    } else {
                        buffer.append((char) ((value / 10) + 48));
                        buffer.append((char) ((value % 10) + 48));
                        return;
                    }
                }
            } else {
                if (value < 1000) {
                    digits = 3;
                } else {
                    Validate.isTrue(value > -1, "Negative values should not be possible", (long) value);
                    digits = Integer.toString(value).length();
                }
                int i2 = this.mSize;
                while (true) {
                    i2--;
                    if (i2 >= digits) {
                        buffer.append('0');
                    } else {
                        buffer.append(Integer.toString(value));
                        return;
                    }
                }
            }
        }
    }

    private static class TwoDigitNumberField implements NumberRule {
        private final int mField;

        TwoDigitNumberField(int field) {
            this.mField = field;
        }

        public int estimateLength() {
            return 2;
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            appendTo(buffer, calendar.get(this.mField));
        }

        public final void appendTo(StringBuffer buffer, int value) {
            if (value < 100) {
                buffer.append((char) ((value / 10) + 48));
                buffer.append((char) ((value % 10) + 48));
                return;
            }
            buffer.append(Integer.toString(value));
        }
    }

    private static class TwoDigitYearField implements NumberRule {
        static final TwoDigitYearField INSTANCE = new TwoDigitYearField();

        TwoDigitYearField() {
        }

        public int estimateLength() {
            return 2;
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            appendTo(buffer, calendar.get(1) % 100);
        }

        public final void appendTo(StringBuffer buffer, int value) {
            buffer.append((char) ((value / 10) + 48));
            buffer.append((char) ((value % 10) + 48));
        }
    }

    private static class TwoDigitMonthField implements NumberRule {
        static final TwoDigitMonthField INSTANCE = new TwoDigitMonthField();

        TwoDigitMonthField() {
        }

        public int estimateLength() {
            return 2;
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            appendTo(buffer, calendar.get(2) + 1);
        }

        public final void appendTo(StringBuffer buffer, int value) {
            buffer.append((char) ((value / 10) + 48));
            buffer.append((char) ((value % 10) + 48));
        }
    }

    private static class TwelveHourField implements NumberRule {
        private final NumberRule mRule;

        TwelveHourField(NumberRule rule) {
            this.mRule = rule;
        }

        public int estimateLength() {
            return this.mRule.estimateLength();
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            int value = calendar.get(10);
            if (value == 0) {
                value = calendar.getLeastMaximum(10) + 1;
            }
            this.mRule.appendTo(buffer, value);
        }

        public void appendTo(StringBuffer buffer, int value) {
            this.mRule.appendTo(buffer, value);
        }
    }

    private static class TwentyFourHourField implements NumberRule {
        private final NumberRule mRule;

        TwentyFourHourField(NumberRule rule) {
            this.mRule = rule;
        }

        public int estimateLength() {
            return this.mRule.estimateLength();
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            int value = calendar.get(11);
            if (value == 0) {
                value = calendar.getMaximum(11) + 1;
            }
            this.mRule.appendTo(buffer, value);
        }

        public void appendTo(StringBuffer buffer, int value) {
            this.mRule.appendTo(buffer, value);
        }
    }

    private static class TimeZoneNameRule implements Rule {
        private final String mDaylight;
        private final Locale mLocale;
        private final String mStandard;
        private final int mStyle;
        private final TimeZone mTimeZone;
        private final boolean mTimeZoneForced;

        TimeZoneNameRule(TimeZone timeZone, boolean timeZoneForced, Locale locale, int style) {
            this.mTimeZone = timeZone;
            this.mTimeZoneForced = timeZoneForced;
            this.mLocale = locale;
            this.mStyle = style;
            if (timeZoneForced) {
                this.mStandard = FastDateFormat.getTimeZoneDisplay(timeZone, false, style, locale);
                this.mDaylight = FastDateFormat.getTimeZoneDisplay(timeZone, true, style, locale);
                return;
            }
            this.mStandard = null;
            this.mDaylight = null;
        }

        public int estimateLength() {
            if (this.mTimeZoneForced) {
                return Math.max(this.mStandard.length(), this.mDaylight.length());
            }
            if (this.mStyle == 0) {
                return 4;
            }
            return 40;
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            if (!this.mTimeZoneForced) {
                TimeZone timeZone = calendar.getTimeZone();
                if (!timeZone.useDaylightTime() || calendar.get(16) == 0) {
                    buffer.append(FastDateFormat.getTimeZoneDisplay(timeZone, false, this.mStyle, this.mLocale));
                } else {
                    buffer.append(FastDateFormat.getTimeZoneDisplay(timeZone, true, this.mStyle, this.mLocale));
                }
            } else if (!this.mTimeZone.useDaylightTime() || calendar.get(16) == 0) {
                buffer.append(this.mStandard);
            } else {
                buffer.append(this.mDaylight);
            }
        }
    }

    private static class TimeZoneNumberRule implements Rule {
        static final TimeZoneNumberRule INSTANCE_COLON = new TimeZoneNumberRule(true);
        static final TimeZoneNumberRule INSTANCE_NO_COLON = new TimeZoneNumberRule(false);
        final boolean mColon;

        TimeZoneNumberRule(boolean colon) {
            this.mColon = colon;
        }

        public int estimateLength() {
            return 5;
        }

        public void appendTo(StringBuffer buffer, Calendar calendar) {
            int offset = calendar.get(15) + calendar.get(16);
            if (offset < 0) {
                buffer.append('-');
                offset = -offset;
            } else {
                buffer.append('+');
            }
            int hours = offset / DateUtils.MILLIS_IN_HOUR;
            buffer.append((char) ((hours / 10) + 48));
            buffer.append((char) ((hours % 10) + 48));
            if (this.mColon) {
                buffer.append(':');
            }
            int minutes = (offset / DateUtils.MILLIS_IN_MINUTE) - (hours * 60);
            buffer.append((char) ((minutes / 10) + 48));
            buffer.append((char) ((minutes % 10) + 48));
        }
    }

    private static class TimeZoneDisplayKey {
        private final Locale mLocale;
        private final int mStyle;
        private final TimeZone mTimeZone;

        TimeZoneDisplayKey(TimeZone timeZone, boolean daylight, int style, Locale locale) {
            this.mTimeZone = timeZone;
            this.mStyle = daylight ? style | Integer.MIN_VALUE : style;
            this.mLocale = locale;
        }

        public int hashCode() {
            return (this.mStyle * 31) + this.mLocale.hashCode();
        }

        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof TimeZoneDisplayKey)) {
                return false;
            }
            TimeZoneDisplayKey other = (TimeZoneDisplayKey) obj;
            if (!this.mTimeZone.equals(other.mTimeZone) || this.mStyle != other.mStyle || !this.mLocale.equals(other.mLocale)) {
                return false;
            }
            return true;
        }
    }

    private static class Pair {
        private final Object mObj1;
        private final Object mObj2;

        public Pair(Object obj1, Object obj2) {
            this.mObj1 = obj1;
            this.mObj2 = obj2;
        }

        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof Pair)) {
                return false;
            }
            Pair key = (Pair) obj;
            if (this.mObj1 != null ? this.mObj1.equals(key.mObj1) : key.mObj1 == null) {
                if (this.mObj2 == null) {
                    if (key.mObj2 == null) {
                        return true;
                    }
                } else if (this.mObj2.equals(key.mObj2)) {
                    return true;
                }
            }
            return false;
        }

        public int hashCode() {
            int i = 0;
            int hashCode = this.mObj1 == null ? 0 : this.mObj1.hashCode();
            if (this.mObj2 != null) {
                i = this.mObj2.hashCode();
            }
            return hashCode + i;
        }

        public String toString() {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("[");
            stringBuffer.append(this.mObj1);
            stringBuffer.append(':');
            stringBuffer.append(this.mObj2);
            stringBuffer.append(']');
            return stringBuffer.toString();
        }
    }
}
