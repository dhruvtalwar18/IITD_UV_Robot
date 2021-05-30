package org.apache.ws.commons.util;

import java.text.FieldPosition;
import java.text.Format;
import java.text.ParsePosition;
import java.util.Calendar;
import java.util.TimeZone;
import org.apache.commons.lang.time.DateUtils;

public class XsDateTimeFormat extends Format {
    private static final long serialVersionUID = 3258131340871479609L;
    final boolean parseDate;
    final boolean parseTime;

    XsDateTimeFormat(boolean pParseDate, boolean pParseTime) {
        this.parseDate = pParseDate;
        this.parseTime = pParseTime;
    }

    public XsDateTimeFormat() {
        this(true, true);
    }

    private int parseInt(String pString, int pOffset, StringBuffer pDigits) {
        int length = pString.length();
        pDigits.setLength(0);
        while (pOffset < length) {
            char c = pString.charAt(pOffset);
            if (!Character.isDigit(c)) {
                break;
            }
            pDigits.append(c);
            pOffset++;
        }
        return pOffset;
    }

    public Object parseObject(String pString, ParsePosition pParsePosition) {
        int mday;
        int month;
        int year;
        int offset;
        int hour;
        int millis;
        int minute;
        int second;
        int i;
        String str = pString;
        ParsePosition parsePosition = pParsePosition;
        if (str == null) {
            throw new NullPointerException("The String argument must not be null.");
        } else if (parsePosition != null) {
            int offset2 = pParsePosition.getIndex();
            int length = pString.length();
            boolean isMinus = false;
            StringBuffer digits = new StringBuffer();
            if (this.parseDate) {
                if (offset2 < length) {
                    char c = str.charAt(offset2);
                    if (c == '+') {
                        offset2++;
                    } else if (c == '-') {
                        offset2++;
                        isMinus = true;
                    }
                }
                int offset3 = parseInt(str, offset2, digits);
                if (digits.length() < 4) {
                    parsePosition.setErrorIndex(offset3);
                    return null;
                }
                year = Integer.parseInt(digits.toString());
                if (offset3 >= length || str.charAt(offset3) != '-') {
                    parsePosition.setErrorIndex(offset3);
                    return null;
                }
                int offset4 = parseInt(str, offset3 + 1, digits);
                if (digits.length() != 2) {
                    parsePosition.setErrorIndex(offset4);
                    return null;
                }
                month = Integer.parseInt(digits.toString());
                if (offset4 >= length || str.charAt(offset4) != '-') {
                    parsePosition.setErrorIndex(offset4);
                    return null;
                }
                offset2 = parseInt(str, offset4 + 1, digits);
                if (digits.length() != 2) {
                    parsePosition.setErrorIndex(offset2);
                    return null;
                }
                mday = Integer.parseInt(digits.toString());
                if (this.parseTime) {
                    if (offset2 >= length || str.charAt(offset2) != 'T') {
                        parsePosition.setErrorIndex(offset2);
                        return null;
                    }
                    offset2++;
                }
            } else {
                mday = 0;
                month = 0;
                year = 0;
            }
            if (this.parseTime) {
                int offset5 = parseInt(str, offset, digits);
                if (digits.length() != 2) {
                    parsePosition.setErrorIndex(offset5);
                    return null;
                }
                int second2 = Integer.parseInt(digits.toString());
                if (offset5 >= length || str.charAt(offset5) != ':') {
                    parsePosition.setErrorIndex(offset5);
                    return null;
                }
                int offset6 = parseInt(str, offset5 + 1, digits);
                if (digits.length() != 2) {
                    parsePosition.setErrorIndex(offset6);
                    return null;
                }
                int parseInt = Integer.parseInt(digits.toString());
                if (offset6 >= length || str.charAt(offset6) != ':') {
                    parsePosition.setErrorIndex(offset6);
                    return null;
                }
                offset = parseInt(str, offset6 + 1, digits);
                if (digits.length() != 2) {
                    parsePosition.setErrorIndex(offset);
                    return null;
                }
                int minute2 = Integer.parseInt(digits.toString());
                if (offset >= length || str.charAt(offset) != '.') {
                    millis = 0;
                } else {
                    offset = parseInt(str, offset + 1, digits);
                    if (digits.length() > 0) {
                        millis = Integer.parseInt(digits.toString());
                        if (millis > 999) {
                            parsePosition.setErrorIndex(offset);
                            return null;
                        }
                        for (int i2 = digits.length(); i2 < 3; i2++) {
                            millis *= 10;
                        }
                    } else {
                        millis = 0;
                    }
                }
                hour = second2;
                second = minute2;
                minute = parseInt;
                i = 0;
            } else {
                i = 0;
                millis = 0;
                second = 0;
                minute = 0;
                hour = 0;
            }
            digits.setLength(i);
            digits.append("GMT");
            if (offset < length) {
                char c2 = str.charAt(offset);
                if (c2 == 'Z') {
                    offset++;
                } else if (c2 == '+' || c2 == '-') {
                    digits.append(c2);
                    int offset7 = offset + 1;
                    while (i < 5) {
                        if (offset >= length) {
                            parsePosition.setErrorIndex(offset);
                            return null;
                        }
                        char c3 = str.charAt(offset);
                        if (i == 2 || !Character.isDigit(c3)) {
                            if (i != 2 || c3 != ':') {
                                parsePosition.setErrorIndex(offset);
                                return null;
                            }
                        }
                        digits.append(c3);
                        offset7 = offset + 1;
                        i++;
                    }
                }
            }
            Calendar cal = Calendar.getInstance(TimeZone.getTimeZone(digits.toString()));
            cal.set(isMinus ? -year : year, this.parseDate ? month - 1 : month, mday, hour, minute, second);
            cal.set(14, millis);
            parsePosition.setIndex(offset);
            return cal;
        } else {
            throw new NullPointerException("The ParsePosition argument must not be null.");
        }
    }

    private void append(StringBuffer pBuffer, int pNum, int pMinLen) {
        String s = Integer.toString(pNum);
        for (int i = s.length(); i < pMinLen; i++) {
            pBuffer.append('0');
        }
        pBuffer.append(s);
    }

    public StringBuffer format(Object pCalendar, StringBuffer pBuffer, FieldPosition pPos) {
        if (pCalendar == null) {
            throw new NullPointerException("The Calendar argument must not be null.");
        } else if (pBuffer == null) {
            throw new NullPointerException("The StringBuffer argument must not be null.");
        } else if (pPos != null) {
            Calendar cal = (Calendar) pCalendar;
            if (this.parseDate) {
                int year = cal.get(1);
                if (year < 0) {
                    pBuffer.append('-');
                    year = -year;
                }
                append(pBuffer, year, 4);
                pBuffer.append('-');
                append(pBuffer, cal.get(2) + 1, 2);
                pBuffer.append('-');
                append(pBuffer, cal.get(5), 2);
                if (this.parseTime) {
                    pBuffer.append('T');
                }
            }
            if (this.parseTime) {
                append(pBuffer, cal.get(11), 2);
                pBuffer.append(':');
                append(pBuffer, cal.get(12), 2);
                pBuffer.append(':');
                append(pBuffer, cal.get(13), 2);
                int millis = cal.get(14);
                if (millis > 0) {
                    pBuffer.append('.');
                    append(pBuffer, millis, 3);
                }
            }
            TimeZone tz = cal.getTimeZone();
            int offset = cal.get(15);
            if (tz.inDaylightTime(cal.getTime())) {
                offset += cal.get(16);
            }
            if (offset == 0) {
                pBuffer.append('Z');
            } else {
                if (offset < 0) {
                    pBuffer.append('-');
                    offset = -offset;
                } else {
                    pBuffer.append('+');
                }
                int minutes = offset / DateUtils.MILLIS_IN_MINUTE;
                int hours = minutes / 60;
                append(pBuffer, hours, 2);
                pBuffer.append(':');
                append(pBuffer, minutes - (hours * 60), 2);
            }
            return pBuffer;
        } else {
            throw new NullPointerException("The FieldPosition argument must not be null.");
        }
    }
}
