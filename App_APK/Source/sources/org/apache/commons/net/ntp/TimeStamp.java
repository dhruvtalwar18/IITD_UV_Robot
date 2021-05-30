package org.apache.commons.net.ntp;

import java.io.Serializable;
import java.lang.ref.SoftReference;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;

public class TimeStamp implements Serializable, Comparable {
    public static final String NTP_DATE_FORMAT = "EEE, MMM dd yyyy HH:mm:ss.SSS";
    protected static final long msb0baseTime = 2085978496000L;
    protected static final long msb1baseTime = -2208988800000L;
    private static final long serialVersionUID = 8139806907588338737L;
    private static SoftReference<DateFormat> simpleFormatter = null;
    private static SoftReference<DateFormat> utcFormatter = null;
    private long ntpTime;

    public TimeStamp(long ntpTime2) {
        this.ntpTime = ntpTime2;
    }

    public TimeStamp(String s) throws NumberFormatException {
        this.ntpTime = decodeNtpHexString(s);
    }

    public TimeStamp(Date d) {
        this.ntpTime = d == null ? 0 : toNtpTime(d.getTime());
    }

    public long ntpValue() {
        return this.ntpTime;
    }

    public long getSeconds() {
        return (this.ntpTime >>> 32) & 4294967295L;
    }

    public long getFraction() {
        return this.ntpTime & 4294967295L;
    }

    public long getTime() {
        return getTime(this.ntpTime);
    }

    public Date getDate() {
        return new Date(getTime(this.ntpTime));
    }

    public static long getTime(long ntpTimeValue) {
        long seconds = (ntpTimeValue >>> 32) & 4294967295L;
        double d = (double) (4294967295L & ntpTimeValue);
        Double.isNaN(d);
        long fraction = Math.round((d * 1000.0d) / 4.294967296E9d);
        if ((2147483648L & seconds) == 0) {
            return (1000 * seconds) + msb0baseTime + fraction;
        }
        return (1000 * seconds) + msb1baseTime + fraction;
    }

    public static TimeStamp getNtpTime(long date) {
        return new TimeStamp(toNtpTime(date));
    }

    public static TimeStamp getCurrentTime() {
        return getNtpTime(System.currentTimeMillis());
    }

    protected static long decodeNtpHexString(String s) throws NumberFormatException {
        if (s != null) {
            int ind = s.indexOf(46);
            if (ind != -1) {
                return (Long.parseLong(s.substring(0, ind), 16) << 32) | Long.parseLong(s.substring(ind + 1), 16);
            }
            if (s.length() == 0) {
                return 0;
            }
            return Long.parseLong(s, 16) << 32;
        }
        throw new NumberFormatException("null");
    }

    public static TimeStamp parseNtpString(String s) throws NumberFormatException {
        return new TimeStamp(decodeNtpHexString(s));
    }

    protected static long toNtpTime(long t) {
        long baseTime;
        boolean useBase1 = t < msb0baseTime;
        if (useBase1) {
            baseTime = t - msb1baseTime;
        } else {
            baseTime = t - msb0baseTime;
        }
        long seconds = baseTime / 1000;
        long fraction = ((baseTime % 1000) * 4294967296L) / 1000;
        if (useBase1) {
            seconds |= 2147483648L;
        }
        return (seconds << 32) | fraction;
    }

    public int hashCode() {
        return (int) (this.ntpTime ^ (this.ntpTime >>> 32));
    }

    public boolean equals(Object obj) {
        if (!(obj instanceof TimeStamp) || this.ntpTime != ((TimeStamp) obj).ntpValue()) {
            return false;
        }
        return true;
    }

    public String toString() {
        return toString(this.ntpTime);
    }

    private static void appendHexString(StringBuffer buf, long l) {
        String s = Long.toHexString(l);
        for (int i = s.length(); i < 8; i++) {
            buf.append('0');
        }
        buf.append(s);
    }

    public static String toString(long ntpTime2) {
        StringBuffer buf = new StringBuffer();
        appendHexString(buf, (ntpTime2 >>> 32) & 4294967295L);
        buf.append('.');
        appendHexString(buf, ntpTime2 & 4294967295L);
        return buf.toString();
    }

    public String toDateString() {
        String format;
        DateFormat formatter = null;
        if (simpleFormatter != null) {
            formatter = simpleFormatter.get();
        }
        if (formatter == null) {
            formatter = new SimpleDateFormat(NTP_DATE_FORMAT, Locale.US);
            formatter.setTimeZone(TimeZone.getDefault());
            simpleFormatter = new SoftReference<>(formatter);
        }
        Date ntpDate = getDate();
        synchronized (formatter) {
            format = formatter.format(ntpDate);
        }
        return format;
    }

    public String toUTCString() {
        String format;
        DateFormat formatter = null;
        if (utcFormatter != null) {
            formatter = utcFormatter.get();
        }
        if (formatter == null) {
            formatter = new SimpleDateFormat("EEE, MMM dd yyyy HH:mm:ss.SSS 'UTC'", Locale.US);
            formatter.setTimeZone(TimeZone.getTimeZone("UTC"));
            utcFormatter = new SoftReference<>(formatter);
        }
        Date ntpDate = getDate();
        synchronized (formatter) {
            format = formatter.format(ntpDate);
        }
        return format;
    }

    public int compareTo(TimeStamp anotherTimeStamp) {
        long thisVal = this.ntpTime;
        long anotherVal = anotherTimeStamp.ntpTime;
        if (thisVal < anotherVal) {
            return -1;
        }
        return thisVal == anotherVal ? 0 : 1;
    }

    public int compareTo(Object o) {
        return compareTo((TimeStamp) o);
    }
}
