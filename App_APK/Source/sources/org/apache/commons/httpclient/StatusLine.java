package org.apache.commons.httpclient;

public class StatusLine {
    private final String httpVersion;
    private final String reasonPhrase;
    private final int statusCode;
    private final String statusLine;

    public StatusLine(String statusLine2) throws HttpException {
        int length = statusLine2.length();
        int at = 0;
        int start = 0;
        while (Character.isWhitespace(statusLine2.charAt(at))) {
            try {
                at++;
                start++;
            } catch (NumberFormatException e) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Unable to parse status code from status line: '");
                stringBuffer.append(statusLine2);
                stringBuffer.append("'");
                throw new ProtocolException(stringBuffer.toString());
            } catch (StringIndexOutOfBoundsException e2) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Status-Line '");
                stringBuffer2.append(statusLine2);
                stringBuffer2.append("' is not valid");
                throw new HttpException(stringBuffer2.toString());
            }
        }
        int at2 = at + 4;
        try {
            if ("HTTP".equals(statusLine2.substring(at, at2)) != 0) {
                int at3 = statusLine2.indexOf(" ", at2);
                if (at3 > 0) {
                    this.httpVersion = statusLine2.substring(start, at3).toUpperCase();
                    while (statusLine2.charAt(at3) == ' ') {
                        at3++;
                    }
                    int to = statusLine2.indexOf(" ", at3);
                    to = to < 0 ? length : to;
                    this.statusCode = Integer.parseInt(statusLine2.substring(at3, to));
                    int at4 = to + 1;
                    if (at4 < length) {
                        this.reasonPhrase = statusLine2.substring(at4).trim();
                    } else {
                        this.reasonPhrase = "";
                    }
                    this.statusLine = statusLine2;
                    return;
                }
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Unable to parse HTTP-Version from the status line: '");
                stringBuffer3.append(statusLine2);
                stringBuffer3.append("'");
                throw new ProtocolException(stringBuffer3.toString());
            }
            StringBuffer stringBuffer4 = new StringBuffer();
            stringBuffer4.append("Status-Line '");
            stringBuffer4.append(statusLine2);
            stringBuffer4.append("' does not start with HTTP");
            throw new HttpException(stringBuffer4.toString());
        } catch (StringIndexOutOfBoundsException e3) {
            int i = at2;
            StringBuffer stringBuffer22 = new StringBuffer();
            stringBuffer22.append("Status-Line '");
            stringBuffer22.append(statusLine2);
            stringBuffer22.append("' is not valid");
            throw new HttpException(stringBuffer22.toString());
        }
    }

    public final int getStatusCode() {
        return this.statusCode;
    }

    public final String getHttpVersion() {
        return this.httpVersion;
    }

    public final String getReasonPhrase() {
        return this.reasonPhrase;
    }

    public final String toString() {
        return this.statusLine;
    }

    public static boolean startsWithHTTP(String s) {
        int at = 0;
        while (Character.isWhitespace(s.charAt(at))) {
            try {
                at++;
            } catch (StringIndexOutOfBoundsException e) {
                return false;
            }
        }
        return "HTTP".equals(s.substring(at, at + 4));
    }
}
