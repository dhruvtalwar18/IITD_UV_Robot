package org.apache.commons.httpclient.util;

import java.io.UnsupportedEncodingException;
import org.apache.commons.codec.net.URLCodec;
import org.apache.commons.httpclient.HttpClientError;
import org.apache.commons.httpclient.NameValuePair;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class EncodingUtil {
    private static final String DEFAULT_CHARSET = "ISO-8859-1";
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$util$EncodingUtil;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$util$EncodingUtil == null) {
            cls = class$("org.apache.commons.httpclient.util.EncodingUtil");
            class$org$apache$commons$httpclient$util$EncodingUtil = cls;
        } else {
            cls = class$org$apache$commons$httpclient$util$EncodingUtil;
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

    public static String formUrlEncode(NameValuePair[] pairs, String charset) {
        try {
            return doFormUrlEncode(pairs, charset);
        } catch (UnsupportedEncodingException e) {
            Log log = LOG;
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Encoding not supported: ");
            stringBuffer.append(charset);
            log.error(stringBuffer.toString());
            try {
                return doFormUrlEncode(pairs, "ISO-8859-1");
            } catch (UnsupportedEncodingException e2) {
                throw new HttpClientError("Encoding not supported: ISO-8859-1");
            }
        }
    }

    private static String doFormUrlEncode(NameValuePair[] pairs, String charset) throws UnsupportedEncodingException {
        StringBuffer buf = new StringBuffer();
        for (int i = 0; i < pairs.length; i++) {
            URLCodec codec = new URLCodec();
            NameValuePair pair = pairs[i];
            if (pair.getName() != null) {
                if (i > 0) {
                    buf.append("&");
                }
                buf.append(codec.encode(pair.getName(), charset));
                buf.append("=");
                if (pair.getValue() != null) {
                    buf.append(codec.encode(pair.getValue(), charset));
                }
            }
        }
        return buf.toString();
    }

    public static String getString(byte[] data, int offset, int length, String charset) {
        if (data == null) {
            throw new IllegalArgumentException("Parameter may not be null");
        } else if (charset == null || charset.length() == 0) {
            throw new IllegalArgumentException("charset may not be null or empty");
        } else {
            try {
                return new String(data, offset, length, charset);
            } catch (UnsupportedEncodingException e) {
                if (LOG.isWarnEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Unsupported encoding: ");
                    stringBuffer.append(charset);
                    stringBuffer.append(". System encoding used");
                    log.warn(stringBuffer.toString());
                }
                return new String(data, offset, length);
            }
        }
    }

    public static String getString(byte[] data, String charset) {
        return getString(data, 0, data.length, charset);
    }

    public static byte[] getBytes(String data, String charset) {
        if (data == null) {
            throw new IllegalArgumentException("data may not be null");
        } else if (charset == null || charset.length() == 0) {
            throw new IllegalArgumentException("charset may not be null or empty");
        } else {
            try {
                return data.getBytes(charset);
            } catch (UnsupportedEncodingException e) {
                if (LOG.isWarnEnabled()) {
                    Log log = LOG;
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Unsupported encoding: ");
                    stringBuffer.append(charset);
                    stringBuffer.append(". System encoding used.");
                    log.warn(stringBuffer.toString());
                }
                return data.getBytes();
            }
        }
    }

    public static byte[] getAsciiBytes(String data) {
        if (data != null) {
            try {
                return data.getBytes("US-ASCII");
            } catch (UnsupportedEncodingException e) {
                throw new HttpClientError("HttpClient requires ASCII support");
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
                throw new HttpClientError("HttpClient requires ASCII support");
            }
        } else {
            throw new IllegalArgumentException("Parameter may not be null");
        }
    }

    public static String getAsciiString(byte[] data) {
        return getAsciiString(data, 0, data.length);
    }

    private EncodingUtil() {
    }
}
