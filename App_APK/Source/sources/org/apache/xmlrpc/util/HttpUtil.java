package org.apache.xmlrpc.util;

import java.io.IOException;
import java.io.InputStream;
import java.io.UnsupportedEncodingException;
import java.util.Enumeration;
import java.util.StringTokenizer;
import org.apache.commons.httpclient.auth.AuthState;
import org.apache.ws.commons.util.Base64;
import org.apache.xmlrpc.common.XmlRpcHttpRequestConfigImpl;

public class HttpUtil {
    public static String encodeBasicAuthentication(String pUser, String pPassword, String pEncoding) throws UnsupportedEncodingException {
        if (pUser == null) {
            return null;
        }
        String s = pUser + ':' + pPassword;
        if (pEncoding == null) {
            pEncoding = "UTF-8";
        }
        return Base64.encode(s.getBytes(pEncoding), 0, s.getBytes(pEncoding).length, 0, (String) null);
    }

    public static boolean isUsingGzipEncoding(String pHeaderValue) {
        if (pHeaderValue == null) {
            return false;
        }
        StringTokenizer st = new StringTokenizer(pHeaderValue, ",");
        while (st.hasMoreTokens()) {
            String encoding = st.nextToken();
            int offset = encoding.indexOf(59);
            if (offset >= 0) {
                encoding = encoding.substring(0, offset);
            }
            if ("gzip".equalsIgnoreCase(encoding.trim())) {
                return true;
            }
        }
        return false;
    }

    public static String getNonIdentityTransferEncoding(String pHeaderValue) {
        if (pHeaderValue == null) {
            return null;
        }
        StringTokenizer st = new StringTokenizer(pHeaderValue, ",");
        while (st.hasMoreTokens()) {
            String encoding = st.nextToken();
            int offset = encoding.indexOf(59);
            if (offset >= 0) {
                encoding = encoding.substring(0, offset);
            }
            if (!"identity".equalsIgnoreCase(encoding.trim())) {
                return encoding.trim();
            }
        }
        return null;
    }

    public static boolean isUsingGzipEncoding(Enumeration pValues) {
        if (pValues == null) {
            return false;
        }
        while (pValues.hasMoreElements()) {
            if (isUsingGzipEncoding((String) pValues.nextElement())) {
                return true;
            }
        }
        return false;
    }

    public static String readLine(InputStream pIn, byte[] pBuffer) throws IOException {
        int count = 0;
        do {
            int next = pIn.read();
            if (next < 0 || next == 10) {
                return new String(pBuffer, 0, count, "US-ASCII");
            }
            if (next != 13) {
                pBuffer[count] = (byte) next;
                count++;
            }
        } while (count < pBuffer.length);
        throw new IOException("HTTP Header too long");
    }

    public static void parseAuthorization(XmlRpcHttpRequestConfigImpl pConfig, String pLine) {
        if (pLine != null) {
            StringTokenizer st = new StringTokenizer(pLine.trim());
            if (st.hasMoreTokens() && AuthState.PREEMPTIVE_AUTH_SCHEME.equalsIgnoreCase(st.nextToken()) && st.hasMoreTokens()) {
                String auth = st.nextToken();
                try {
                    byte[] c = Base64.decode(auth.toCharArray(), 0, auth.length());
                    String enc = pConfig.getBasicEncoding();
                    if (enc == null) {
                        enc = "UTF-8";
                    }
                    String str = new String(c, enc);
                    int col = str.indexOf(58);
                    if (col >= 0) {
                        pConfig.setBasicUserName(str.substring(0, col));
                        pConfig.setBasicPassword(str.substring(col + 1));
                    }
                } catch (Throwable th) {
                }
            }
        }
    }
}
