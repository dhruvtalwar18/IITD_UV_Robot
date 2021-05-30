package org.apache.commons.lang;

import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import org.apache.commons.lang.exception.NestableRuntimeException;

public class StringEscapeUtils {
    private static final char CSV_DELIMITER = ',';
    private static final char CSV_QUOTE = '\"';
    private static final String CSV_QUOTE_STR = String.valueOf(CSV_QUOTE);
    private static final char[] CSV_SEARCH_CHARS = {CSV_DELIMITER, CSV_QUOTE, CharUtils.CR, 10};

    public static String escapeJava(String str) {
        return escapeJavaStyleString(str, false);
    }

    public static void escapeJava(Writer out, String str) throws IOException {
        escapeJavaStyleString(out, str, false);
    }

    public static String escapeJavaScript(String str) {
        return escapeJavaStyleString(str, true);
    }

    public static void escapeJavaScript(Writer out, String str) throws IOException {
        escapeJavaStyleString(out, str, true);
    }

    private static String escapeJavaStyleString(String str, boolean escapeSingleQuotes) {
        if (str == null) {
            return null;
        }
        try {
            StringWriter writer = new StringWriter(str.length() * 2);
            escapeJavaStyleString(writer, str, escapeSingleQuotes);
            return writer.toString();
        } catch (IOException ioe) {
            ioe.printStackTrace();
            return null;
        }
    }

    private static void escapeJavaStyleString(Writer out, String str, boolean escapeSingleQuote) throws IOException {
        if (out == null) {
            throw new IllegalArgumentException("The Writer must not be null");
        } else if (str != null) {
            int sz = str.length();
            for (int i = 0; i < sz; i++) {
                char ch = str.charAt(i);
                if (ch > 4095) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("\\u");
                    stringBuffer.append(hex(ch));
                    out.write(stringBuffer.toString());
                } else if (ch > 255) {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("\\u0");
                    stringBuffer2.append(hex(ch));
                    out.write(stringBuffer2.toString());
                } else if (ch > 127) {
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append("\\u00");
                    stringBuffer3.append(hex(ch));
                    out.write(stringBuffer3.toString());
                } else if (ch < ' ') {
                    switch (ch) {
                        case 8:
                            out.write(92);
                            out.write(98);
                            break;
                        case 9:
                            out.write(92);
                            out.write(116);
                            break;
                        case 10:
                            out.write(92);
                            out.write(110);
                            break;
                        case 12:
                            out.write(92);
                            out.write(102);
                            break;
                        case 13:
                            out.write(92);
                            out.write(114);
                            break;
                        default:
                            if (ch <= 15) {
                                StringBuffer stringBuffer4 = new StringBuffer();
                                stringBuffer4.append("\\u000");
                                stringBuffer4.append(hex(ch));
                                out.write(stringBuffer4.toString());
                                break;
                            } else {
                                StringBuffer stringBuffer5 = new StringBuffer();
                                stringBuffer5.append("\\u00");
                                stringBuffer5.append(hex(ch));
                                out.write(stringBuffer5.toString());
                                break;
                            }
                    }
                } else if (ch == '\"') {
                    out.write(92);
                    out.write(34);
                } else if (ch == '\'') {
                    if (escapeSingleQuote) {
                        out.write(92);
                    }
                    out.write(39);
                } else if (ch == '/') {
                    out.write(92);
                    out.write(47);
                } else if (ch != '\\') {
                    out.write(ch);
                } else {
                    out.write(92);
                    out.write(92);
                }
            }
        }
    }

    private static String hex(char ch) {
        return Integer.toHexString(ch).toUpperCase();
    }

    public static String unescapeJava(String str) {
        if (str == null) {
            return null;
        }
        try {
            StringWriter writer = new StringWriter(str.length());
            unescapeJava(writer, str);
            return writer.toString();
        } catch (IOException ioe) {
            ioe.printStackTrace();
            return null;
        }
    }

    public static void unescapeJava(Writer out, String str) throws IOException {
        if (out == null) {
            throw new IllegalArgumentException("The Writer must not be null");
        } else if (str != null) {
            int sz = str.length();
            StringBuffer unicode = new StringBuffer(4);
            boolean inUnicode = false;
            boolean hadSlash = false;
            for (int i = 0; i < sz; i++) {
                char ch = str.charAt(i);
                if (inUnicode) {
                    unicode.append(ch);
                    if (unicode.length() == 4) {
                        try {
                            out.write((char) Integer.parseInt(unicode.toString(), 16));
                            unicode.setLength(0);
                            inUnicode = false;
                            hadSlash = false;
                        } catch (NumberFormatException nfe) {
                            StringBuffer stringBuffer = new StringBuffer();
                            stringBuffer.append("Unable to parse unicode value: ");
                            stringBuffer.append(unicode);
                            throw new NestableRuntimeException(stringBuffer.toString(), nfe);
                        }
                    }
                } else if (hadSlash) {
                    hadSlash = false;
                    if (ch == '\"') {
                        out.write(34);
                    } else if (ch == '\'') {
                        out.write(39);
                    } else if (ch == '\\') {
                        out.write(92);
                    } else if (ch == 'b') {
                        out.write(8);
                    } else if (ch == 'f') {
                        out.write(12);
                    } else if (ch == 'n') {
                        out.write(10);
                    } else if (ch != 'r') {
                        switch (ch) {
                            case 't':
                                out.write(9);
                                break;
                            case 'u':
                                inUnicode = true;
                                break;
                            default:
                                out.write(ch);
                                break;
                        }
                    } else {
                        out.write(13);
                    }
                } else if (ch == '\\') {
                    hadSlash = true;
                } else {
                    out.write(ch);
                }
            }
            if (hadSlash) {
                out.write(92);
            }
        }
    }

    public static String unescapeJavaScript(String str) {
        return unescapeJava(str);
    }

    public static void unescapeJavaScript(Writer out, String str) throws IOException {
        unescapeJava(out, str);
    }

    public static String escapeHtml(String str) {
        if (str == null) {
            return null;
        }
        try {
            double length = (double) str.length();
            Double.isNaN(length);
            StringWriter writer = new StringWriter((int) (length * 1.5d));
            escapeHtml(writer, str);
            return writer.toString();
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static void escapeHtml(Writer writer, String string) throws IOException {
        if (writer == null) {
            throw new IllegalArgumentException("The Writer must not be null.");
        } else if (string != null) {
            Entities.HTML40.escape(writer, string);
        }
    }

    public static String unescapeHtml(String str) {
        if (str == null) {
            return null;
        }
        try {
            double length = (double) str.length();
            Double.isNaN(length);
            StringWriter writer = new StringWriter((int) (length * 1.5d));
            unescapeHtml(writer, str);
            return writer.toString();
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    public static void unescapeHtml(Writer writer, String string) throws IOException {
        if (writer == null) {
            throw new IllegalArgumentException("The Writer must not be null.");
        } else if (string != null) {
            Entities.HTML40.unescape(writer, string);
        }
    }

    public static void escapeXml(Writer writer, String str) throws IOException {
        if (writer == null) {
            throw new IllegalArgumentException("The Writer must not be null.");
        } else if (str != null) {
            Entities.XML.escape(writer, str);
        }
    }

    public static String escapeXml(String str) {
        if (str == null) {
            return null;
        }
        return Entities.XML.escape(str);
    }

    public static void unescapeXml(Writer writer, String str) throws IOException {
        if (writer == null) {
            throw new IllegalArgumentException("The Writer must not be null.");
        } else if (str != null) {
            Entities.XML.unescape(writer, str);
        }
    }

    public static String unescapeXml(String str) {
        if (str == null) {
            return null;
        }
        return Entities.XML.unescape(str);
    }

    public static String escapeSql(String str) {
        if (str == null) {
            return null;
        }
        return StringUtils.replace(str, "'", "''");
    }

    public static String escapeCsv(String str) {
        if (StringUtils.containsNone(str, CSV_SEARCH_CHARS)) {
            return str;
        }
        try {
            StringWriter writer = new StringWriter();
            escapeCsv(writer, str);
            return writer.toString();
        } catch (IOException ioe) {
            ioe.printStackTrace();
            return null;
        }
    }

    public static void escapeCsv(Writer out, String str) throws IOException {
        if (!StringUtils.containsNone(str, CSV_SEARCH_CHARS)) {
            out.write(34);
            for (int i = 0; i < str.length(); i++) {
                char c = str.charAt(i);
                if (c == '\"') {
                    out.write(34);
                }
                out.write(c);
            }
            out.write(34);
        } else if (str != null) {
            out.write(str);
        }
    }

    public static String unescapeCsv(String str) {
        if (str == null) {
            return null;
        }
        try {
            StringWriter writer = new StringWriter();
            unescapeCsv(writer, str);
            return writer.toString();
        } catch (IOException ioe) {
            ioe.printStackTrace();
            return null;
        }
    }

    public static void unescapeCsv(Writer out, String str) throws IOException {
        if (str != null) {
            if (str.length() < 2) {
                out.write(str);
            } else if (str.charAt(0) == '\"' && str.charAt(str.length() - 1) == '\"') {
                String quoteless = str.substring(1, str.length() - 1);
                if (StringUtils.containsAny(quoteless, CSV_SEARCH_CHARS)) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append(CSV_QUOTE_STR);
                    stringBuffer.append(CSV_QUOTE_STR);
                    str = StringUtils.replace(quoteless, stringBuffer.toString(), CSV_QUOTE_STR);
                }
                out.write(str);
            } else {
                out.write(str);
            }
        }
    }
}
