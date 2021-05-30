package org.apache.commons.lang.time;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.TimeZone;
import org.apache.commons.lang.StringUtils;

public class DurationFormatUtils {
    static final Object H = "H";
    public static final String ISO_EXTENDED_FORMAT_PATTERN = "'P'yyyy'Y'M'M'd'DT'H'H'm'M's.S'S'";
    static final Object M = "M";
    static final Object S = "S";
    static final Object d = "d";
    static final Object m = "m";
    static final Object s = "s";
    static final Object y = "y";

    public static String formatDurationHMS(long durationMillis) {
        return formatDuration(durationMillis, "H:mm:ss.SSS");
    }

    public static String formatDurationISO(long durationMillis) {
        return formatDuration(durationMillis, ISO_EXTENDED_FORMAT_PATTERN, false);
    }

    public static String formatDuration(long durationMillis, String format) {
        return formatDuration(durationMillis, format, true);
    }

    public static String formatDuration(long durationMillis, String format, boolean padWithZeros) {
        long durationMillis2;
        int hours;
        int seconds;
        long durationMillis3;
        int milliseconds;
        Token[] tokens = lexx(format);
        int days = 0;
        int minutes = 0;
        if (Token.containsTokenWithValue(tokens, d)) {
            days = (int) (durationMillis / DateUtils.MILLIS_PER_DAY);
            durationMillis2 = durationMillis - (((long) days) * DateUtils.MILLIS_PER_DAY);
        } else {
            durationMillis2 = durationMillis;
        }
        int days2 = days;
        if (Token.containsTokenWithValue(tokens, H)) {
            int hours2 = (int) (durationMillis2 / DateUtils.MILLIS_PER_HOUR);
            durationMillis2 -= ((long) hours2) * DateUtils.MILLIS_PER_HOUR;
            hours = hours2;
        } else {
            hours = 0;
        }
        if (Token.containsTokenWithValue(tokens, m)) {
            minutes = (int) (durationMillis2 / DateUtils.MILLIS_PER_MINUTE);
            durationMillis2 -= ((long) minutes) * DateUtils.MILLIS_PER_MINUTE;
        }
        int minutes2 = minutes;
        if (Token.containsTokenWithValue(tokens, s)) {
            int seconds2 = (int) (durationMillis2 / 1000);
            durationMillis3 = durationMillis2 - (((long) seconds2) * 1000);
            seconds = seconds2;
        } else {
            durationMillis3 = durationMillis2;
            seconds = 0;
        }
        if (Token.containsTokenWithValue(tokens, S)) {
            milliseconds = (int) durationMillis3;
        } else {
            milliseconds = 0;
        }
        return format(tokens, 0, 0, days2, hours, minutes2, seconds, milliseconds, padWithZeros);
    }

    public static String formatDurationWords(long durationMillis, boolean suppressLeadingZeroElements, boolean suppressTrailingZeroElements) {
        String duration = formatDuration(durationMillis, "d' days 'H' hours 'm' minutes 's' seconds'");
        if (suppressLeadingZeroElements) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(" ");
            stringBuffer.append(duration);
            duration = stringBuffer.toString();
            String tmp = StringUtils.replaceOnce(duration, " 0 days", "");
            if (tmp.length() != duration.length()) {
                duration = tmp;
                String tmp2 = StringUtils.replaceOnce(duration, " 0 hours", "");
                if (tmp2.length() != duration.length()) {
                    String tmp3 = StringUtils.replaceOnce(tmp2, " 0 minutes", "");
                    duration = tmp3;
                    if (tmp3.length() != duration.length()) {
                        duration = StringUtils.replaceOnce(tmp3, " 0 seconds", "");
                    }
                }
            }
            if (duration.length() != 0) {
                duration = duration.substring(1);
            }
        }
        if (suppressTrailingZeroElements) {
            String tmp4 = StringUtils.replaceOnce(duration, " 0 seconds", "");
            if (tmp4.length() != duration.length()) {
                duration = tmp4;
                String tmp5 = StringUtils.replaceOnce(duration, " 0 minutes", "");
                if (tmp5.length() != duration.length()) {
                    duration = tmp5;
                    String tmp6 = StringUtils.replaceOnce(duration, " 0 hours", "");
                    if (tmp6.length() != duration.length()) {
                        duration = StringUtils.replaceOnce(tmp6, " 0 days", "");
                    }
                }
            }
        }
        StringBuffer stringBuffer2 = new StringBuffer();
        stringBuffer2.append(" ");
        stringBuffer2.append(duration);
        return StringUtils.replaceOnce(StringUtils.replaceOnce(StringUtils.replaceOnce(StringUtils.replaceOnce(stringBuffer2.toString(), " 1 seconds", " 1 second"), " 1 minutes", " 1 minute"), " 1 hours", " 1 hour"), " 1 days", " 1 day").trim();
    }

    public static String formatPeriodISO(long startMillis, long endMillis) {
        return formatPeriod(startMillis, endMillis, ISO_EXTENDED_FORMAT_PATTERN, false, TimeZone.getDefault());
    }

    public static String formatPeriod(long startMillis, long endMillis, String format) {
        return formatPeriod(startMillis, endMillis, format, true, TimeZone.getDefault());
    }

    public static String formatPeriod(long startMillis, long endMillis, String format, boolean padWithZeros, TimeZone timezone) {
        int months;
        int days;
        int hours;
        int minutes;
        int milliseconds;
        int seconds;
        int target;
        Token[] tokens = lexx(format);
        Calendar start = Calendar.getInstance(timezone);
        start.setTime(new Date(startMillis));
        Calendar end = Calendar.getInstance(timezone);
        end.setTime(new Date(endMillis));
        int milliseconds2 = end.get(14) - start.get(14);
        int seconds2 = end.get(13) - start.get(13);
        int minutes2 = end.get(12) - start.get(12);
        int hours2 = end.get(11) - start.get(11);
        int days2 = end.get(5) - start.get(5);
        int months2 = end.get(2) - start.get(2);
        int years = end.get(1) - start.get(1);
        while (milliseconds2 < 0) {
            milliseconds2 += 1000;
            seconds2--;
        }
        while (seconds2 < 0) {
            seconds2 += 60;
            minutes2--;
        }
        while (minutes2 < 0) {
            minutes2 += 60;
            hours2--;
        }
        while (hours2 < 0) {
            hours2 += 24;
            days2--;
        }
        if (Token.containsTokenWithValue(tokens, M)) {
            while (days2 < 0) {
                days2 += start.getActualMaximum(5);
                months2--;
                start.add(2, 1);
            }
            while (months2 < 0) {
                months2 += 12;
                years--;
            }
            if (!Token.containsTokenWithValue(tokens, y) && years != 0) {
                while (years != 0) {
                    months2 += years * 12;
                    years = 0;
                }
            }
            months = months2;
        } else {
            if (!Token.containsTokenWithValue(tokens, y)) {
                int target2 = end.get(1);
                if (months2 < 0) {
                    target2--;
                }
                while (start.get(1) != target2) {
                    int days3 = days2 + (start.getActualMaximum(6) - start.get(6));
                    if (start instanceof GregorianCalendar) {
                        target = target2;
                        if (start.get(2) == 1 && start.get(5) == 29) {
                            days3++;
                        }
                    } else {
                        target = target2;
                    }
                    start.add(1, 1);
                    days2 = days3 + start.get(6);
                    target2 = target;
                }
                years = 0;
            }
            while (start.get(2) != end.get(2)) {
                days2 += start.getActualMaximum(5);
                start.add(2, 1);
            }
            int months3 = 0;
            while (days2 < 0) {
                days2 += start.getActualMaximum(5);
                months3--;
                start.add(2, 1);
            }
            months = months3;
        }
        if (!Token.containsTokenWithValue(tokens, d)) {
            hours2 += days2 * 24;
            days = 0;
        } else {
            days = days2;
        }
        if (!Token.containsTokenWithValue(tokens, H)) {
            minutes2 += hours2 * 60;
            hours = 0;
        } else {
            hours = hours2;
        }
        if (!Token.containsTokenWithValue(tokens, m)) {
            seconds2 += minutes2 * 60;
            minutes = 0;
        } else {
            minutes = minutes2;
        }
        if (!Token.containsTokenWithValue(tokens, s)) {
            seconds = 0;
            milliseconds = milliseconds2 + (seconds2 * 1000);
        } else {
            milliseconds = milliseconds2;
            seconds = seconds2;
        }
        return format(tokens, years, months, days, hours, minutes, seconds, milliseconds, padWithZeros);
    }

    static String format(Token[] tokens, int years, int months, int days, int hours, int minutes, int seconds, int milliseconds, boolean padWithZeros) {
        StringBuffer buffer = new StringBuffer();
        boolean lastOutputSeconds = false;
        int milliseconds2 = milliseconds;
        for (Token token : tokens) {
            Object value = token.getValue();
            int count = token.getCount();
            if (value instanceof StringBuffer) {
                buffer.append(value.toString());
            } else if (value == y) {
                buffer.append(padWithZeros ? StringUtils.leftPad(Integer.toString(years), count, '0') : Integer.toString(years));
                lastOutputSeconds = false;
            } else if (value == M) {
                buffer.append(padWithZeros ? StringUtils.leftPad(Integer.toString(months), count, '0') : Integer.toString(months));
                lastOutputSeconds = false;
            } else if (value == d) {
                buffer.append(padWithZeros ? StringUtils.leftPad(Integer.toString(days), count, '0') : Integer.toString(days));
                lastOutputSeconds = false;
            } else if (value == H) {
                buffer.append(padWithZeros ? StringUtils.leftPad(Integer.toString(hours), count, '0') : Integer.toString(hours));
                lastOutputSeconds = false;
            } else if (value == m) {
                buffer.append(padWithZeros ? StringUtils.leftPad(Integer.toString(minutes), count, '0') : Integer.toString(minutes));
                lastOutputSeconds = false;
            } else if (value == s) {
                buffer.append(padWithZeros ? StringUtils.leftPad(Integer.toString(seconds), count, '0') : Integer.toString(seconds));
                lastOutputSeconds = true;
            } else if (value == S) {
                if (lastOutputSeconds) {
                    milliseconds2 += 1000;
                    buffer.append((padWithZeros ? StringUtils.leftPad(Integer.toString(milliseconds2), count, '0') : Integer.toString(milliseconds2)).substring(1));
                } else {
                    buffer.append(padWithZeros ? StringUtils.leftPad(Integer.toString(milliseconds2), count, '0') : Integer.toString(milliseconds2));
                }
                lastOutputSeconds = false;
            }
        }
        return buffer.toString();
    }

    static Token[] lexx(String format) {
        char[] array = format.toCharArray();
        ArrayList list = new ArrayList(array.length);
        boolean inLiteral = false;
        StringBuffer buffer = null;
        Token previous = null;
        for (char ch : array) {
            if (!inLiteral || ch == '\'') {
                Object value = null;
                if (ch != '\'') {
                    if (ch == 'H') {
                        value = H;
                    } else if (ch == 'M') {
                        value = M;
                    } else if (ch == 'S') {
                        value = S;
                    } else if (ch == 'd') {
                        value = d;
                    } else if (ch == 'm') {
                        value = m;
                    } else if (ch == 's') {
                        value = s;
                    } else if (ch != 'y') {
                        if (buffer == null) {
                            buffer = new StringBuffer();
                            list.add(new Token(buffer));
                        }
                        buffer.append(ch);
                    } else {
                        value = y;
                    }
                } else if (inLiteral) {
                    buffer = null;
                    inLiteral = false;
                } else {
                    buffer = new StringBuffer();
                    list.add(new Token(buffer));
                    inLiteral = true;
                }
                if (value != null) {
                    if (previous == null || previous.getValue() != value) {
                        Token token = new Token(value);
                        list.add(token);
                        previous = token;
                    } else {
                        previous.increment();
                    }
                    buffer = null;
                }
            } else {
                buffer.append(ch);
            }
        }
        return (Token[]) list.toArray(new Token[list.size()]);
    }

    static class Token {
        private int count;
        private Object value;

        static boolean containsTokenWithValue(Token[] tokens, Object value2) {
            for (Token value3 : tokens) {
                if (value3.getValue() == value2) {
                    return true;
                }
            }
            return false;
        }

        Token(Object value2) {
            this.value = value2;
            this.count = 1;
        }

        Token(Object value2, int count2) {
            this.value = value2;
            this.count = count2;
        }

        /* access modifiers changed from: package-private */
        public void increment() {
            this.count++;
        }

        /* access modifiers changed from: package-private */
        public int getCount() {
            return this.count;
        }

        /* access modifiers changed from: package-private */
        public Object getValue() {
            return this.value;
        }

        public boolean equals(Object obj2) {
            if (!(obj2 instanceof Token)) {
                return false;
            }
            Token tok2 = (Token) obj2;
            if (this.value.getClass() != tok2.value.getClass() || this.count != tok2.count) {
                return false;
            }
            if (this.value instanceof StringBuffer) {
                return this.value.toString().equals(tok2.value.toString());
            }
            if (this.value instanceof Number) {
                return this.value.equals(tok2.value);
            }
            if (this.value == tok2.value) {
                return true;
            }
            return false;
        }

        public int hashCode() {
            return this.value.hashCode();
        }

        public String toString() {
            return StringUtils.repeat(this.value.toString(), this.count);
        }
    }
}
