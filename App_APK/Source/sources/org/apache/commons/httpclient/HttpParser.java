package org.apache.commons.httpclient;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import org.apache.commons.httpclient.util.EncodingUtil;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class HttpParser {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$HttpParser;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$HttpParser == null) {
            cls = class$("org.apache.commons.httpclient.HttpParser");
            class$org$apache$commons$httpclient$HttpParser = cls;
        } else {
            cls = class$org$apache$commons$httpclient$HttpParser;
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

    private HttpParser() {
    }

    public static byte[] readRawLine(InputStream inputStream) throws IOException {
        int ch;
        LOG.trace("enter HttpParser.readRawLine()");
        ByteArrayOutputStream buf = new ByteArrayOutputStream();
        do {
            int read = inputStream.read();
            ch = read;
            if (read < 0) {
                break;
            }
            buf.write(ch);
        } while (ch != 10);
        if (buf.size() == 0) {
            return null;
        }
        return buf.toByteArray();
    }

    public static String readLine(InputStream inputStream, String charset) throws IOException {
        LOG.trace("enter HttpParser.readLine(InputStream, String)");
        byte[] rawdata = readRawLine(inputStream);
        if (rawdata == null) {
            return null;
        }
        int len = rawdata.length;
        int offset = 0;
        if (len > 0 && rawdata[len - 1] == 10) {
            offset = 0 + 1;
            if (len > 1 && rawdata[len - 2] == 13) {
                offset++;
            }
        }
        String result = EncodingUtil.getString(rawdata, 0, len - offset, charset);
        if (Wire.HEADER_WIRE.enabled()) {
            String logoutput = result;
            if (offset == 2) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append(result);
                stringBuffer.append("\r\n");
                logoutput = stringBuffer.toString();
            } else if (offset == 1) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append(result);
                stringBuffer2.append("\n");
                logoutput = stringBuffer2.toString();
            }
            Wire.HEADER_WIRE.input(logoutput);
        }
        return result;
    }

    public static String readLine(InputStream inputStream) throws IOException {
        LOG.trace("enter HttpParser.readLine(InputStream)");
        return readLine(inputStream, "US-ASCII");
    }

    /* JADX WARNING: Removed duplicated region for block: B:19:0x0087  */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static org.apache.commons.httpclient.Header[] parseHeaders(java.io.InputStream r8, java.lang.String r9) throws java.io.IOException, org.apache.commons.httpclient.HttpException {
        /*
            org.apache.commons.logging.Log r0 = LOG
            java.lang.String r1 = "enter HeaderParser.parseHeaders(InputStream, String)"
            r0.trace(r1)
            java.util.ArrayList r0 = new java.util.ArrayList
            r0.<init>()
            r1 = 0
            r2 = 0
        L_0x000e:
            java.lang.String r3 = readLine(r8, r9)
            if (r3 == 0) goto L_0x0085
            java.lang.String r4 = r3.trim()
            int r4 = r4.length()
            r5 = 1
            if (r4 >= r5) goto L_0x0020
            goto L_0x0085
        L_0x0020:
            r4 = 0
            char r5 = r3.charAt(r4)
            r6 = 32
            if (r5 == r6) goto L_0x0078
            char r5 = r3.charAt(r4)
            r7 = 9
            if (r5 != r7) goto L_0x0032
            goto L_0x0078
        L_0x0032:
            if (r1 == 0) goto L_0x0040
            org.apache.commons.httpclient.Header r5 = new org.apache.commons.httpclient.Header
            java.lang.String r6 = r2.toString()
            r5.<init>(r1, r6)
            r0.add(r5)
        L_0x0040:
            java.lang.String r5 = ":"
            int r5 = r3.indexOf(r5)
            if (r5 < 0) goto L_0x0061
            java.lang.String r4 = r3.substring(r4, r5)
            java.lang.String r1 = r4.trim()
            java.lang.StringBuffer r4 = new java.lang.StringBuffer
            int r6 = r5 + 1
            java.lang.String r6 = r3.substring(r6)
            java.lang.String r6 = r6.trim()
            r4.<init>(r6)
            r2 = r4
            goto L_0x0084
        L_0x0061:
            org.apache.commons.httpclient.ProtocolException r4 = new org.apache.commons.httpclient.ProtocolException
            java.lang.StringBuffer r6 = new java.lang.StringBuffer
            r6.<init>()
            java.lang.String r7 = "Unable to parse header: "
            r6.append(r7)
            r6.append(r3)
            java.lang.String r6 = r6.toString()
            r4.<init>(r6)
            throw r4
        L_0x0078:
            if (r2 == 0) goto L_0x0084
            r2.append(r6)
            java.lang.String r4 = r3.trim()
            r2.append(r4)
        L_0x0084:
            goto L_0x000e
        L_0x0085:
            if (r1 == 0) goto L_0x0093
            org.apache.commons.httpclient.Header r3 = new org.apache.commons.httpclient.Header
            java.lang.String r4 = r2.toString()
            r3.<init>(r1, r4)
            r0.add(r3)
        L_0x0093:
            int r3 = r0.size()
            org.apache.commons.httpclient.Header[] r3 = new org.apache.commons.httpclient.Header[r3]
            java.lang.Object[] r3 = r0.toArray(r3)
            org.apache.commons.httpclient.Header[] r3 = (org.apache.commons.httpclient.Header[]) r3
            return r3
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.commons.httpclient.HttpParser.parseHeaders(java.io.InputStream, java.lang.String):org.apache.commons.httpclient.Header[]");
    }

    public static Header[] parseHeaders(InputStream is) throws IOException, HttpException {
        LOG.trace("enter HeaderParser.parseHeaders(InputStream, String)");
        return parseHeaders(is, "US-ASCII");
    }
}
