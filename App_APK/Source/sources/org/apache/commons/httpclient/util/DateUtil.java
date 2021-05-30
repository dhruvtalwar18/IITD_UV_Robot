package org.apache.commons.httpclient.util;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Collection;
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;

public class DateUtil {
    private static final Collection DEFAULT_PATTERNS = Arrays.asList(new String[]{"EEE MMM d HH:mm:ss yyyy", "EEEE, dd-MMM-yy HH:mm:ss zzz", "EEE, dd MMM yyyy HH:mm:ss zzz"});
    private static final Date DEFAULT_TWO_DIGIT_YEAR_START;
    private static final TimeZone GMT = TimeZone.getTimeZone("GMT");
    public static final String PATTERN_ASCTIME = "EEE MMM d HH:mm:ss yyyy";
    public static final String PATTERN_RFC1036 = "EEEE, dd-MMM-yy HH:mm:ss zzz";
    public static final String PATTERN_RFC1123 = "EEE, dd MMM yyyy HH:mm:ss zzz";

    static {
        Calendar calendar = Calendar.getInstance();
        calendar.set(2000, 0, 1, 0, 0);
        DEFAULT_TWO_DIGIT_YEAR_START = calendar.getTime();
    }

    public static Date parseDate(String dateValue) throws DateParseException {
        return parseDate(dateValue, (Collection) null, (Date) null);
    }

    public static Date parseDate(String dateValue, Collection dateFormats) throws DateParseException {
        return parseDate(dateValue, dateFormats, (Date) null);
    }

    public static Date parseDate(String dateValue, Collection<String> dateFormats, Date startDate) throws DateParseException {
        if (dateValue != null) {
            if (dateFormats == null) {
                dateFormats = DEFAULT_PATTERNS;
            }
            if (startDate == null) {
                startDate = DEFAULT_TWO_DIGIT_YEAR_START;
            }
            if (dateValue.length() > 1 && dateValue.startsWith("'") && dateValue.endsWith("'")) {
                dateValue = dateValue.substring(1, dateValue.length() - 1);
            }
            SimpleDateFormat dateParser = null;
            for (String format : dateFormats) {
                if (dateParser == null) {
                    dateParser = new SimpleDateFormat(format, Locale.US);
                    dateParser.setTimeZone(TimeZone.getTimeZone("GMT"));
                    dateParser.set2DigitYearStart(startDate);
                } else {
                    dateParser.applyPattern(format);
                }
                try {
                    return dateParser.parse(dateValue);
                } catch (ParseException e) {
                }
            }
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Unable to parse the date ");
            stringBuffer.append(dateValue);
            throw new DateParseException(stringBuffer.toString());
        }
        throw new IllegalArgumentException("dateValue is null");
    }

    public static String formatDate(Date date) {
        return formatDate(date, "EEE, dd MMM yyyy HH:mm:ss zzz");
    }

    public static String formatDate(Date date, String pattern) {
        if (date == null) {
            throw new IllegalArgumentException("date is null");
        } else if (pattern != null) {
            SimpleDateFormat formatter = new SimpleDateFormat(pattern, Locale.US);
            formatter.setTimeZone(GMT);
            return formatter.format(date);
        } else {
            throw new IllegalArgumentException("pattern is null");
        }
    }

    private DateUtil() {
    }
}
