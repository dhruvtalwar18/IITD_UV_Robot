package org.apache.xmlrpc.util;

import java.text.FieldPosition;
import java.text.Format;
import java.text.ParsePosition;
import java.util.Calendar;
import java.util.TimeZone;

public abstract class XmlRpcDateTimeFormat extends Format {
    private static final long serialVersionUID = -8008230377361175138L;

    /* access modifiers changed from: protected */
    public abstract TimeZone getTimeZone();

    private int parseInt(String pString, int pOffset, StringBuffer pDigits, int pMaxDigits) {
        int length = pString.length();
        pDigits.setLength(0);
        while (true) {
            int pMaxDigits2 = pMaxDigits - 1;
            if (pMaxDigits <= 0 || pOffset >= length) {
                break;
            }
            char c = pString.charAt(pOffset);
            if (!Character.isDigit(c)) {
                break;
            }
            pDigits.append(c);
            pOffset++;
            pMaxDigits = pMaxDigits2;
        }
        return pOffset;
    }

    public Object parseObject(String pString, ParsePosition pParsePosition) {
        String str = pString;
        ParsePosition parsePosition = pParsePosition;
        if (str == null) {
            throw new NullPointerException("The String argument must not be null.");
        } else if (parsePosition != null) {
            int offset = pParsePosition.getIndex();
            int length = pString.length();
            StringBuffer digits = new StringBuffer();
            int offset2 = parseInt(str, offset, digits, 4);
            if (digits.length() < 4) {
                parsePosition.setErrorIndex(offset2);
                return null;
            }
            int year = Integer.parseInt(digits.toString());
            int offset3 = parseInt(str, offset2, digits, 2);
            if (digits.length() != 2) {
                parsePosition.setErrorIndex(offset3);
                return null;
            }
            int month = Integer.parseInt(digits.toString());
            int offset4 = parseInt(str, offset3, digits, 2);
            if (digits.length() != 2) {
                parsePosition.setErrorIndex(offset4);
                return null;
            }
            int mday = Integer.parseInt(digits.toString());
            if (offset4 >= length || str.charAt(offset4) != 'T') {
                parsePosition.setErrorIndex(offset4);
                return null;
            }
            int offset5 = parseInt(str, offset4 + 1, digits, 2);
            if (digits.length() != 2) {
                parsePosition.setErrorIndex(offset5);
                return null;
            }
            int hour = Integer.parseInt(digits.toString());
            if (offset5 >= length || str.charAt(offset5) != ':') {
                parsePosition.setErrorIndex(offset5);
                return null;
            }
            int offset6 = parseInt(str, offset5 + 1, digits, 2);
            if (digits.length() != 2) {
                parsePosition.setErrorIndex(offset6);
                return null;
            }
            int minute = Integer.parseInt(digits.toString());
            if (offset6 >= length || str.charAt(offset6) != ':') {
                parsePosition.setErrorIndex(offset6);
                return null;
            }
            int offset7 = parseInt(str, offset6 + 1, digits, 2);
            if (digits.length() != 2) {
                parsePosition.setErrorIndex(offset7);
                return null;
            }
            int second = Integer.parseInt(digits.toString());
            Calendar cal = Calendar.getInstance(getTimeZone());
            cal.set(year, month - 1, mday, hour, minute, second);
            cal.set(14, 0);
            parsePosition.setIndex(offset7);
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
            append(pBuffer, cal.get(1), 4);
            append(pBuffer, cal.get(2) + 1, 2);
            append(pBuffer, cal.get(5), 2);
            pBuffer.append('T');
            append(pBuffer, cal.get(11), 2);
            pBuffer.append(':');
            append(pBuffer, cal.get(12), 2);
            pBuffer.append(':');
            append(pBuffer, cal.get(13), 2);
            return pBuffer;
        } else {
            throw new NullPointerException("The FieldPosition argument must not be null.");
        }
    }
}
