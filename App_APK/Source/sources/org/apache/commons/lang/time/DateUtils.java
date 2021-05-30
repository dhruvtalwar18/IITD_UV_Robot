package org.apache.commons.lang.time;

import java.text.ParseException;
import java.text.ParsePosition;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.TimeZone;

public class DateUtils {
    public static final int MILLIS_IN_DAY = 86400000;
    public static final int MILLIS_IN_HOUR = 3600000;
    public static final int MILLIS_IN_MINUTE = 60000;
    public static final int MILLIS_IN_SECOND = 1000;
    public static final long MILLIS_PER_DAY = 86400000;
    public static final long MILLIS_PER_HOUR = 3600000;
    public static final long MILLIS_PER_MINUTE = 60000;
    public static final long MILLIS_PER_SECOND = 1000;
    public static final int RANGE_MONTH_MONDAY = 6;
    public static final int RANGE_MONTH_SUNDAY = 5;
    public static final int RANGE_WEEK_CENTER = 4;
    public static final int RANGE_WEEK_MONDAY = 2;
    public static final int RANGE_WEEK_RELATIVE = 3;
    public static final int RANGE_WEEK_SUNDAY = 1;
    public static final int SEMI_MONTH = 1001;
    public static final TimeZone UTC_TIME_ZONE = TimeZone.getTimeZone("GMT");
    private static final int[][] fields = {new int[]{14}, new int[]{13}, new int[]{12}, new int[]{11, 10}, new int[]{5, 5, 9}, new int[]{2, 1001}, new int[]{1}, new int[]{0}};

    public static boolean isSameDay(Date date1, Date date2) {
        if (date1 == null || date2 == null) {
            throw new IllegalArgumentException("The date must not be null");
        }
        Calendar cal1 = Calendar.getInstance();
        cal1.setTime(date1);
        Calendar cal2 = Calendar.getInstance();
        cal2.setTime(date2);
        return isSameDay(cal1, cal2);
    }

    public static boolean isSameDay(Calendar cal1, Calendar cal2) {
        if (cal1 != null && cal2 != null) {
            return cal1.get(0) == cal2.get(0) && cal1.get(1) == cal2.get(1) && cal1.get(6) == cal2.get(6);
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static boolean isSameInstant(Date date1, Date date2) {
        if (date1 != null && date2 != null) {
            return date1.getTime() == date2.getTime();
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static boolean isSameInstant(Calendar cal1, Calendar cal2) {
        if (cal1 != null && cal2 != null) {
            return cal1.getTime().getTime() == cal2.getTime().getTime();
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static boolean isSameLocalTime(Calendar cal1, Calendar cal2) {
        if (cal1 != null && cal2 != null) {
            return cal1.get(14) == cal2.get(14) && cal1.get(13) == cal2.get(13) && cal1.get(12) == cal2.get(12) && cal1.get(10) == cal2.get(10) && cal1.get(6) == cal2.get(6) && cal1.get(1) == cal2.get(1) && cal1.get(0) == cal2.get(0) && cal1.getClass() == cal2.getClass();
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static Date parseDate(String str, String[] parsePatterns) throws ParseException {
        if (str == null || parsePatterns == null) {
            throw new IllegalArgumentException("Date and Patterns must not be null");
        }
        ParsePosition pos = new ParsePosition(0);
        SimpleDateFormat parser = null;
        for (int i = 0; i < parsePatterns.length; i++) {
            if (i == 0) {
                parser = new SimpleDateFormat(parsePatterns[0]);
            } else {
                parser.applyPattern(parsePatterns[i]);
            }
            pos.setIndex(0);
            Date date = parser.parse(str, pos);
            if (date != null && pos.getIndex() == str.length()) {
                return date;
            }
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Unable to parse the date: ");
        stringBuffer.append(str);
        throw new ParseException(stringBuffer.toString(), -1);
    }

    public static Date addYears(Date date, int amount) {
        return add(date, 1, amount);
    }

    public static Date addMonths(Date date, int amount) {
        return add(date, 2, amount);
    }

    public static Date addWeeks(Date date, int amount) {
        return add(date, 3, amount);
    }

    public static Date addDays(Date date, int amount) {
        return add(date, 5, amount);
    }

    public static Date addHours(Date date, int amount) {
        return add(date, 11, amount);
    }

    public static Date addMinutes(Date date, int amount) {
        return add(date, 12, amount);
    }

    public static Date addSeconds(Date date, int amount) {
        return add(date, 13, amount);
    }

    public static Date addMilliseconds(Date date, int amount) {
        return add(date, 14, amount);
    }

    public static Date add(Date date, int calendarField, int amount) {
        if (date != null) {
            Calendar c = Calendar.getInstance();
            c.setTime(date);
            c.add(calendarField, amount);
            return c.getTime();
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static Date setYears(Date date, int amount) {
        return set(date, 1, amount);
    }

    public static Date setMonths(Date date, int amount) {
        return set(date, 2, amount);
    }

    public static Date setDays(Date date, int amount) {
        return set(date, 5, amount);
    }

    public static Date setHours(Date date, int amount) {
        return set(date, 11, amount);
    }

    public static Date setMinutes(Date date, int amount) {
        return set(date, 12, amount);
    }

    public static Date setSeconds(Date date, int amount) {
        return set(date, 13, amount);
    }

    public static Date setMilliseconds(Date date, int amount) {
        return set(date, 14, amount);
    }

    private static Date set(Date date, int calendarField, int amount) {
        if (date != null) {
            Calendar c = Calendar.getInstance();
            c.setLenient(false);
            c.setTime(date);
            c.set(calendarField, amount);
            return c.getTime();
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static Date round(Date date, int field) {
        if (date != null) {
            Calendar gval = Calendar.getInstance();
            gval.setTime(date);
            modify(gval, field, true);
            return gval.getTime();
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static Calendar round(Calendar date, int field) {
        if (date != null) {
            Calendar rounded = (Calendar) date.clone();
            modify(rounded, field, true);
            return rounded;
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static Date round(Object date, int field) {
        if (date == null) {
            throw new IllegalArgumentException("The date must not be null");
        } else if (date instanceof Date) {
            return round((Date) date, field);
        } else {
            if (date instanceof Calendar) {
                return round((Calendar) date, field).getTime();
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Could not round ");
            stringBuffer.append(date);
            throw new ClassCastException(stringBuffer.toString());
        }
    }

    public static Date truncate(Date date, int field) {
        if (date != null) {
            Calendar gval = Calendar.getInstance();
            gval.setTime(date);
            modify(gval, field, false);
            return gval.getTime();
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static Calendar truncate(Calendar date, int field) {
        if (date != null) {
            Calendar truncated = (Calendar) date.clone();
            modify(truncated, field, false);
            return truncated;
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    public static Date truncate(Object date, int field) {
        if (date == null) {
            throw new IllegalArgumentException("The date must not be null");
        } else if (date instanceof Date) {
            return truncate((Date) date, field);
        } else {
            if (date instanceof Calendar) {
                return truncate((Calendar) date, field).getTime();
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Could not truncate ");
            stringBuffer.append(date);
            throw new ClassCastException(stringBuffer.toString());
        }
    }

    /* JADX WARNING: Removed duplicated region for block: B:71:0x00fa  */
    /* JADX WARNING: Removed duplicated region for block: B:77:0x012a  */
    /* JADX WARNING: Removed duplicated region for block: B:78:0x0140  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private static void modify(java.util.Calendar r18, int r19, boolean r20) {
        /*
            r0 = r18
            r1 = r19
            r3 = 1
            int r4 = r0.get(r3)
            r5 = 280000000(0x10b07600, float:6.960157E-29)
            if (r4 > r5) goto L_0x0164
            r4 = 14
            if (r1 != r4) goto L_0x0013
            return
        L_0x0013:
            java.util.Date r5 = r18.getTime()
            long r6 = r5.getTime()
            r8 = 0
            int r4 = r0.get(r4)
            if (r20 == 0) goto L_0x0026
            r9 = 500(0x1f4, float:7.0E-43)
            if (r4 >= r9) goto L_0x0028
        L_0x0026:
            long r9 = (long) r4
            long r6 = r6 - r9
        L_0x0028:
            r9 = 13
            if (r1 != r9) goto L_0x002d
            r8 = 1
        L_0x002d:
            int r9 = r0.get(r9)
            r10 = 30
            if (r8 != 0) goto L_0x003f
            if (r20 == 0) goto L_0x0039
            if (r9 >= r10) goto L_0x003f
        L_0x0039:
            long r11 = (long) r9
            r13 = 1000(0x3e8, double:4.94E-321)
            long r11 = r11 * r13
            long r6 = r6 - r11
        L_0x003f:
            r11 = 12
            if (r1 != r11) goto L_0x0044
            r8 = 1
        L_0x0044:
            int r12 = r0.get(r11)
            if (r8 != 0) goto L_0x0055
            if (r20 == 0) goto L_0x004e
            if (r12 >= r10) goto L_0x0055
        L_0x004e:
            long r13 = (long) r12
            r15 = 60000(0xea60, double:2.9644E-319)
            long r13 = r13 * r15
            long r6 = r6 - r13
        L_0x0055:
            long r13 = r5.getTime()
            int r10 = (r13 > r6 ? 1 : (r13 == r6 ? 0 : -1))
            if (r10 == 0) goto L_0x0063
            r5.setTime(r6)
            r0.setTime(r5)
        L_0x0063:
            r10 = 0
            r14 = r10
            r10 = 0
        L_0x0066:
            int[][] r15 = fields
            int r15 = r15.length
            if (r10 >= r15) goto L_0x0148
            r15 = 0
        L_0x006c:
            int[][] r16 = fields
            r11 = r16[r10]
            int r11 = r11.length
            r13 = 1001(0x3e9, float:1.403E-42)
            r3 = 5
            if (r15 >= r11) goto L_0x00ad
            int[][] r11 = fields
            r11 = r11[r10]
            r11 = r11[r15]
            if (r11 != r1) goto L_0x00a7
            if (r20 == 0) goto L_0x00a6
            if (r14 == 0) goto L_0x00a6
            if (r1 != r13) goto L_0x009b
            int r11 = r0.get(r3)
            r13 = 1
            if (r11 != r13) goto L_0x0091
            r11 = 15
            r0.add(r3, r11)
            goto L_0x00a6
        L_0x0091:
            r11 = -15
            r0.add(r3, r11)
            r3 = 2
            r0.add(r3, r13)
            goto L_0x00a6
        L_0x009b:
            r13 = 1
            int[][] r3 = fields
            r3 = r3[r10]
            r11 = 0
            r3 = r3[r11]
            r0.add(r3, r13)
        L_0x00a6:
            return
        L_0x00a7:
            int r15 = r15 + 1
            r3 = 1
            r11 = 12
            goto L_0x006c
        L_0x00ad:
            r11 = 0
            r15 = 0
            r3 = 9
            if (r1 == r3) goto L_0x00d8
            if (r1 == r13) goto L_0x00b6
            goto L_0x00d5
        L_0x00b6:
            int[][] r3 = fields
            r3 = r3[r10]
            r13 = 0
            r3 = r3[r13]
            r13 = 5
            if (r3 != r13) goto L_0x00d5
            int r3 = r0.get(r13)
            r13 = 1
            int r3 = r3 - r13
            r11 = 15
            if (r3 < r11) goto L_0x00cc
            int r3 = r3 + -15
        L_0x00cc:
            r11 = r3
            r3 = 7
            if (r11 <= r3) goto L_0x00d2
            r3 = 1
            goto L_0x00d3
        L_0x00d2:
            r3 = 0
        L_0x00d3:
            r14 = r3
            r15 = 1
        L_0x00d5:
            r13 = 12
            goto L_0x00f8
        L_0x00d8:
            r13 = 1
            int[][] r3 = fields
            r3 = r3[r10]
            r16 = 0
            r3 = r3[r16]
            r13 = 11
            if (r3 != r13) goto L_0x00d5
            int r3 = r0.get(r13)
            r13 = 12
            if (r3 < r13) goto L_0x00ef
            int r3 = r3 + -12
        L_0x00ef:
            r11 = r3
            r3 = 6
            if (r11 <= r3) goto L_0x00f5
            r3 = 1
            goto L_0x00f6
        L_0x00f5:
            r3 = 0
        L_0x00f6:
            r14 = r3
            r15 = 1
        L_0x00f8:
            if (r15 != 0) goto L_0x0128
            int[][] r3 = fields
            r3 = r3[r10]
            r16 = 0
            r3 = r3[r16]
            int r3 = r0.getActualMinimum(r3)
            int[][] r17 = fields
            r17 = r17[r10]
            r13 = r17[r16]
            int r13 = r0.getActualMaximum(r13)
            int[][] r17 = fields
            r17 = r17[r10]
            r2 = r17[r16]
            int r2 = r0.get(r2)
            int r11 = r2 - r3
            int r2 = r13 - r3
            r16 = 2
            int r2 = r2 / 2
            if (r11 <= r2) goto L_0x0126
            r2 = 1
            goto L_0x0127
        L_0x0126:
            r2 = 0
        L_0x0127:
            r14 = r2
        L_0x0128:
            if (r11 == 0) goto L_0x0140
            int[][] r2 = fields
            r2 = r2[r10]
            r3 = 0
            r2 = r2[r3]
            int[][] r13 = fields
            r13 = r13[r10]
            r13 = r13[r3]
            int r13 = r0.get(r13)
            int r13 = r13 - r11
            r0.set(r2, r13)
            goto L_0x0141
        L_0x0140:
            r3 = 0
        L_0x0141:
            int r10 = r10 + 1
            r3 = 1
            r11 = 12
            goto L_0x0066
        L_0x0148:
            java.lang.IllegalArgumentException r2 = new java.lang.IllegalArgumentException
            java.lang.StringBuffer r3 = new java.lang.StringBuffer
            r3.<init>()
            java.lang.String r10 = "The field "
            r3.append(r10)
            r3.append(r1)
            java.lang.String r10 = " is not supported"
            r3.append(r10)
            java.lang.String r3 = r3.toString()
            r2.<init>(r3)
            throw r2
        L_0x0164:
            java.lang.ArithmeticException r2 = new java.lang.ArithmeticException
            java.lang.String r3 = "Calendar value too large for accurate calculations"
            r2.<init>(r3)
            throw r2
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.time.DateUtils.modify(java.util.Calendar, int, boolean):void");
    }

    public static Iterator iterator(Date focus, int rangeStyle) {
        if (focus != null) {
            Calendar gval = Calendar.getInstance();
            gval.setTime(focus);
            return iterator(gval, rangeStyle);
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r9v0, resolved type: java.lang.Object} */
    /* JADX DEBUG: Multi-variable search result rejected for TypeSearchVarInfo{r1v4, resolved type: java.util.Calendar} */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static java.util.Iterator iterator(java.util.Calendar r10, int r11) {
        /*
            if (r10 == 0) goto L_0x0092
            r0 = 0
            r1 = 0
            r2 = 1
            r3 = 7
            r4 = -1
            r5 = 1
            r6 = 5
            r7 = 7
            switch(r11) {
                case 1: goto L_0x0041;
                case 2: goto L_0x0041;
                case 3: goto L_0x0041;
                case 4: goto L_0x0041;
                case 5: goto L_0x0029;
                case 6: goto L_0x0029;
                default: goto L_0x000d;
            }
        L_0x000d:
            java.lang.IllegalArgumentException r4 = new java.lang.IllegalArgumentException
            java.lang.StringBuffer r5 = new java.lang.StringBuffer
            r5.<init>()
            java.lang.String r6 = "The range style "
            r5.append(r6)
            r5.append(r11)
            java.lang.String r6 = " is not valid."
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r4.<init>(r5)
            throw r4
        L_0x0029:
            r8 = 2
            java.util.Calendar r0 = truncate((java.util.Calendar) r10, (int) r8)
            java.lang.Object r9 = r0.clone()
            r1 = r9
            java.util.Calendar r1 = (java.util.Calendar) r1
            r1.add(r8, r5)
            r1.add(r6, r4)
            r8 = 6
            if (r11 != r8) goto L_0x0068
            r2 = 2
            r3 = 1
            goto L_0x0068
        L_0x0041:
            java.util.Calendar r0 = truncate((java.util.Calendar) r10, (int) r6)
            java.util.Calendar r1 = truncate((java.util.Calendar) r10, (int) r6)
            switch(r11) {
                case 1: goto L_0x0066;
                case 2: goto L_0x0063;
                case 3: goto L_0x005c;
                case 4: goto L_0x004d;
                default: goto L_0x004c;
            }
        L_0x004c:
            goto L_0x0067
        L_0x004d:
            int r8 = r10.get(r7)
            int r8 = r8 + -3
            int r2 = r10.get(r7)
            int r2 = r2 + 3
            r3 = r2
            r2 = r8
            goto L_0x0067
        L_0x005c:
            int r2 = r10.get(r7)
            int r3 = r2 + -1
            goto L_0x0067
        L_0x0063:
            r2 = 2
            r3 = 1
            goto L_0x0067
        L_0x0066:
        L_0x0067:
        L_0x0068:
            if (r2 >= r5) goto L_0x006c
            int r2 = r2 + 7
        L_0x006c:
            if (r2 <= r7) goto L_0x0070
            int r2 = r2 + -7
        L_0x0070:
            if (r3 >= r5) goto L_0x0074
            int r3 = r3 + 7
        L_0x0074:
            if (r3 <= r7) goto L_0x0078
            int r3 = r3 + -7
        L_0x0078:
            int r8 = r0.get(r7)
            if (r8 == r2) goto L_0x0082
            r0.add(r6, r4)
            goto L_0x0078
        L_0x0082:
            int r4 = r1.get(r7)
            if (r4 == r3) goto L_0x008c
            r1.add(r6, r5)
            goto L_0x0082
        L_0x008c:
            org.apache.commons.lang.time.DateUtils$DateIterator r4 = new org.apache.commons.lang.time.DateUtils$DateIterator
            r4.<init>(r0, r1)
            return r4
        L_0x0092:
            java.lang.IllegalArgumentException r0 = new java.lang.IllegalArgumentException
            java.lang.String r1 = "The date must not be null"
            r0.<init>(r1)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.time.DateUtils.iterator(java.util.Calendar, int):java.util.Iterator");
    }

    public static Iterator iterator(Object focus, int rangeStyle) {
        if (focus == null) {
            throw new IllegalArgumentException("The date must not be null");
        } else if (focus instanceof Date) {
            return iterator((Date) focus, rangeStyle);
        } else {
            if (focus instanceof Calendar) {
                return iterator((Calendar) focus, rangeStyle);
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Could not iterate based on ");
            stringBuffer.append(focus);
            throw new ClassCastException(stringBuffer.toString());
        }
    }

    public static long getFragmentInMilliseconds(Date date, int fragment) {
        return getFragment(date, fragment, 14);
    }

    public static long getFragmentInSeconds(Date date, int fragment) {
        return getFragment(date, fragment, 13);
    }

    public static long getFragmentInMinutes(Date date, int fragment) {
        return getFragment(date, fragment, 12);
    }

    public static long getFragmentInHours(Date date, int fragment) {
        return getFragment(date, fragment, 11);
    }

    public static long getFragmentInDays(Date date, int fragment) {
        return getFragment(date, fragment, 6);
    }

    public static long getFragmentInMilliseconds(Calendar calendar, int fragment) {
        return getFragment(calendar, fragment, 14);
    }

    public static long getFragmentInSeconds(Calendar calendar, int fragment) {
        return getFragment(calendar, fragment, 13);
    }

    public static long getFragmentInMinutes(Calendar calendar, int fragment) {
        return getFragment(calendar, fragment, 12);
    }

    public static long getFragmentInHours(Calendar calendar, int fragment) {
        return getFragment(calendar, fragment, 11);
    }

    public static long getFragmentInDays(Calendar calendar, int fragment) {
        return getFragment(calendar, fragment, 6);
    }

    private static long getFragment(Date date, int fragment, int unit) {
        if (date != null) {
            Calendar calendar = Calendar.getInstance();
            calendar.setTime(date);
            return getFragment(calendar, fragment, unit);
        }
        throw new IllegalArgumentException("The date must not be null");
    }

    /* JADX WARNING: Code restructure failed: missing block: B:10:0x0061, code lost:
        r2 = r2 + ((((long) r8.get(13)) * 1000) / r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:14:?, code lost:
        return r2 + (((long) (r8.get(14) * 1)) / r0);
     */
    /* JADX WARNING: Code restructure failed: missing block: B:9:0x0053, code lost:
        r2 = r2 + ((((long) r8.get(12)) * MILLIS_PER_MINUTE) / r0);
     */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    private static long getFragment(java.util.Calendar r8, int r9, int r10) {
        /*
            if (r8 == 0) goto L_0x007b
            long r0 = getMillisPerUnit(r10)
            r2 = 0
            r4 = 86400000(0x5265c00, double:4.2687272E-316)
            switch(r9) {
                case 1: goto L_0x001a;
                case 2: goto L_0x000f;
                default: goto L_0x000e;
            }
        L_0x000e:
            goto L_0x0025
        L_0x000f:
            r6 = 5
            int r6 = r8.get(r6)
            long r6 = (long) r6
            long r6 = r6 * r4
            long r6 = r6 / r0
            long r2 = r2 + r6
            goto L_0x0025
        L_0x001a:
            r6 = 6
            int r6 = r8.get(r6)
            long r6 = (long) r6
            long r6 = r6 * r4
            long r6 = r6 / r0
            long r2 = r2 + r6
        L_0x0025:
            switch(r9) {
                case 1: goto L_0x0045;
                case 2: goto L_0x0045;
                case 3: goto L_0x0028;
                case 4: goto L_0x0028;
                case 5: goto L_0x0045;
                case 6: goto L_0x0045;
                case 7: goto L_0x0028;
                case 8: goto L_0x0028;
                case 9: goto L_0x0028;
                case 10: goto L_0x0028;
                case 11: goto L_0x0053;
                case 12: goto L_0x0061;
                case 13: goto L_0x006e;
                case 14: goto L_0x0044;
                default: goto L_0x0028;
            }
        L_0x0028:
            java.lang.IllegalArgumentException r4 = new java.lang.IllegalArgumentException
            java.lang.StringBuffer r5 = new java.lang.StringBuffer
            r5.<init>()
            java.lang.String r6 = "The fragment "
            r5.append(r6)
            r5.append(r9)
            java.lang.String r6 = " is not supported"
            r5.append(r6)
            java.lang.String r5 = r5.toString()
            r4.<init>(r5)
            throw r4
        L_0x0044:
            goto L_0x007a
        L_0x0045:
            r4 = 11
            int r4 = r8.get(r4)
            long r4 = (long) r4
            r6 = 3600000(0x36ee80, double:1.7786363E-317)
            long r4 = r4 * r6
            long r4 = r4 / r0
            long r2 = r2 + r4
        L_0x0053:
            r4 = 12
            int r4 = r8.get(r4)
            long r4 = (long) r4
            r6 = 60000(0xea60, double:2.9644E-319)
            long r4 = r4 * r6
            long r4 = r4 / r0
            long r2 = r2 + r4
        L_0x0061:
            r4 = 13
            int r4 = r8.get(r4)
            long r4 = (long) r4
            r6 = 1000(0x3e8, double:4.94E-321)
            long r4 = r4 * r6
            long r4 = r4 / r0
            long r2 = r2 + r4
        L_0x006e:
            r4 = 14
            int r4 = r8.get(r4)
            int r4 = r4 * 1
            long r4 = (long) r4
            long r4 = r4 / r0
            long r2 = r2 + r4
        L_0x007a:
            return r2
        L_0x007b:
            java.lang.IllegalArgumentException r0 = new java.lang.IllegalArgumentException
            java.lang.String r1 = "The date must not be null"
            r0.<init>(r1)
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.lang.time.DateUtils.getFragment(java.util.Calendar, int, int):long");
    }

    private static long getMillisPerUnit(int unit) {
        switch (unit) {
            case 5:
            case 6:
                return MILLIS_PER_DAY;
            default:
                switch (unit) {
                    case 11:
                        return MILLIS_PER_HOUR;
                    case 12:
                        return MILLIS_PER_MINUTE;
                    case 13:
                        return 1000;
                    case 14:
                        return 1;
                    default:
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("The unit ");
                        stringBuffer.append(unit);
                        stringBuffer.append(" cannot be represented is milleseconds");
                        throw new IllegalArgumentException(stringBuffer.toString());
                }
        }
    }

    static class DateIterator implements Iterator {
        private final Calendar endFinal;
        private final Calendar spot;

        DateIterator(Calendar startFinal, Calendar endFinal2) {
            this.endFinal = endFinal2;
            this.spot = startFinal;
            this.spot.add(5, -1);
        }

        public boolean hasNext() {
            return this.spot.before(this.endFinal);
        }

        public Object next() {
            if (!this.spot.equals(this.endFinal)) {
                this.spot.add(5, 1);
                return this.spot.clone();
            }
            throw new NoSuchElementException();
        }

        public void remove() {
            throw new UnsupportedOperationException();
        }
    }
}
