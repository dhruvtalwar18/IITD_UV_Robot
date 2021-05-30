package org.apache.commons.httpclient;

import java.io.UnsupportedEncodingException;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class HttpConstants {
    public static final String DEFAULT_CONTENT_CHARSET = "ISO-8859-1";
    public static final String HTTP_ELEMENT_CHARSET = "US-ASCII";
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$HttpConstants;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$HttpConstants == null) {
            cls = class$("org.apache.commons.httpclient.HttpConstants");
            class$org$apache$commons$httpclient$HttpConstants = cls;
        } else {
            cls = class$org$apache$commons$httpclient$HttpConstants;
        }
        LOG = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public static byte[] getBytes(String data) {
        if (data != null) {
            try {
                return data.getBytes("US-ASCII");
            } catch (UnsupportedEncodingException e) {
                if (LOG.isWarnEnabled()) {
                    LOG.warn("Unsupported encoding: US-ASCII. System default encoding used");
                }
                return data.getBytes();
            }
        } else {
            throw new IllegalArgumentException("Parameter may not be null");
        }
    }

    public static String getString(byte[] data, int offset, int length) {
        if (data != null) {
            try {
                return new String(data, offset, length, "US-ASCII");
            } catch (UnsupportedEncodingException e) {
                if (LOG.isWarnEnabled()) {
                    LOG.warn("Unsupported encoding: US-ASCII. System default encoding used");
                }
                return new String(data, offset, length);
            }
        } else {
            throw new IllegalArgumentException("Parameter may not be null");
        }
    }

    public static String getString(byte[] data) {
        return getString(data, 0, data.length);
    }

    public static byte[] getContentBytes(String data, String charset) {
        if (data != null) {
            if (charset == null || charset.equals("")) {
                charset = "ISO-8859-1";
            }
            try {
                return data.getBytes(charset);
            } catch (UnsupportedEncodingException e) {
                if (LOG.isWarnEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Unsupported encoding: ");
                    stringBuffer.append(charset);
                    stringBuffer.append(". HTTP default encoding used");
                    log.warn(stringBuffer.toString());
                }
                try {
                    return data.getBytes("ISO-8859-1");
                } catch (UnsupportedEncodingException e2) {
                    if (LOG.isWarnEnabled()) {
                        LOG.warn("Unsupported encoding: ISO-8859-1. System encoding used");
                    }
                    return data.getBytes();
                }
            }
        } else {
            throw new IllegalArgumentException("Parameter may not be null");
        }
    }

    public static String getContentString(byte[] data, int offset, int length, String charset) {
        if (data != null) {
            if (charset == null || charset.equals("")) {
                charset = "ISO-8859-1";
            }
            try {
                return new String(data, offset, length, charset);
            } catch (UnsupportedEncodingException e) {
                if (LOG.isWarnEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Unsupported encoding: ");
                    stringBuffer.append(charset);
                    stringBuffer.append(". Default HTTP encoding used");
                    log.warn(stringBuffer.toString());
                }
                try {
                    return new String(data, offset, length, "ISO-8859-1");
                } catch (UnsupportedEncodingException e2) {
                    if (LOG.isWarnEnabled()) {
                        LOG.warn("Unsupported encoding: ISO-8859-1. System encoding used");
                    }
                    return new String(data, offset, length);
                }
            }
        } else {
            throw new IllegalArgumentException("Parameter may not be null");
        }
    }

    public static String getContentString(byte[] data, String charset) {
        return getContentString(data, 0, data.length, charset);
    }

    public static byte[] getContentBytes(String data) {
        return getContentBytes(data, (String) null);
    }

    public static String getContentString(byte[] data, int offset, int length) {
        return getContentString(data, offset, length, (String) null);
    }

    public static String getContentString(byte[] data) {
        return getContentString(data, (String) null);
    }

    public static byte[] getAsciiBytes(String data) {
        if (data != null) {
            try {
                return data.getBytes("US-ASCII");
            } catch (UnsupportedEncodingException e) {
                throw new RuntimeException("HttpClient requires ASCII support");
            }
        } else {
            throw new IllegalArgumentException("Parameter may not be null");
        }
    }

    public static String getAsciiString(byte[] data, int offset, int length) {
        if (data != null) {
            try {
                return new String(data, offset, length, "US-ASCII");
            } catch (UnsupportedEncodingException e) {
                throw new RuntimeException("HttpClient requires ASCII support");
            }
        } else {
            throw new IllegalArgumentException("Parameter may not be null");
        }
    }

    public static String getAsciiString(byte[] data) {
        return getAsciiString(data, 0, data.length);
    }
}
