package org.apache.commons.httpclient.util;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Collection;
import java.util.Date;
import java.util.Locale;
import java.util.TimeZone;

public class DateParser {
    private static final Collection DEFAULT_PATTERNS = Arrays.asList(new String[]{"EEE MMM d HH:mm:ss yyyy", "EEEE, dd-MMM-yy HH:mm:ss zzz", "EEE, dd MMM yyyy HH:mm:ss zzz"});
    public static final String PATTERN_ASCTIME = "EEE MMM d HH:mm:ss yyyy";
    public static final String PATTERN_RFC1036 = "EEEE, dd-MMM-yy HH:mm:ss zzz";
    public static final String PATTERN_RFC1123 = "EEE, dd MMM yyyy HH:mm:ss zzz";

    public static Date parseDate(String dateValue) throws DateParseException {
        return parseDate(dateValue, (Collection) null);
    }

    public static Date parseDate(String dateValue, Collection<String> dateFormats) throws DateParseException {
        if (dateValue != null) {
            if (dateFormats == null) {
                dateFormats = DEFAULT_PATTERNS;
            }
            if (dateValue.length() > 1 && dateValue.startsWith("'") && dateValue.endsWith("'")) {
                dateValue = dateValue.substring(1, dateValue.length() - 1);
            }
            SimpleDateFormat dateParser = null;
            for (String format : dateFormats) {
                if (dateParser == null) {
                    dateParser = new SimpleDateFormat(format, Locale.US);
                    dateParser.setTimeZone(TimeZone.getTimeZone("GMT"));
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

    private DateParser() {
    }
}
