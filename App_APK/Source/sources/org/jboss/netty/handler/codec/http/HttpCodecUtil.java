package org.jboss.netty.handler.codec.http;

import java.util.List;
import org.apache.xmlrpc.serializer.TypeSerializerImpl;
import org.jboss.netty.handler.codec.http.HttpHeaders;

final class HttpCodecUtil {
    static void validateHeaderName(String name) {
        if (name != null) {
            int i = 0;
            while (i < name.length()) {
                char c = name.charAt(i);
                if (c <= 127) {
                    if (!(c == ' ' || c == ',' || c == '=')) {
                        switch (c) {
                            case 9:
                            case 10:
                            case 11:
                            case 12:
                            case 13:
                                break;
                            default:
                                switch (c) {
                                    case ':':
                                    case ';':
                                        break;
                                    default:
                                        i++;
                                }
                        }
                    }
                    throw new IllegalArgumentException("name contains one of the following prohibited characters: =,;: \\t\\r\\n\\v\\f: " + name);
                }
                throw new IllegalArgumentException("name contains non-ascii character: " + name);
            }
            return;
        }
        throw new NullPointerException("name");
    }

    static void validateHeaderValue(String value) {
        if (value != null) {
            int state = 0;
            int i = 0;
            while (i < value.length()) {
                char c = value.charAt(i);
                switch (c) {
                    case 11:
                        throw new IllegalArgumentException("value contains a prohibited character '\\v': " + value);
                    case 12:
                        throw new IllegalArgumentException("value contains a prohibited character '\\f': " + value);
                    default:
                        switch (state) {
                            case 0:
                                if (c != 10) {
                                    if (c == 13) {
                                        state = 1;
                                        break;
                                    } else {
                                        break;
                                    }
                                } else {
                                    state = 2;
                                    break;
                                }
                            case 1:
                                if (c == 10) {
                                    state = 2;
                                    break;
                                } else {
                                    throw new IllegalArgumentException("Only '\\n' is allowed after '\\r': " + value);
                                }
                            case 2:
                                if (c == 9 || c == ' ') {
                                    state = 0;
                                    break;
                                } else {
                                    throw new IllegalArgumentException("Only ' ' and '\\t' are allowed after '\\n': " + value);
                                }
                                break;
                        }
                        i++;
                        break;
                }
            }
            if (state != 0) {
                throw new IllegalArgumentException("value must not end with '\\r' or '\\n':" + value);
            }
            return;
        }
        throw new NullPointerException(TypeSerializerImpl.VALUE_TAG);
    }

    static boolean isTransferEncodingChunked(HttpMessage m) {
        List<String> chunked = m.getHeaders("Transfer-Encoding");
        if (chunked.isEmpty()) {
            return false;
        }
        for (String v : chunked) {
            if (v.equalsIgnoreCase(HttpHeaders.Values.CHUNKED)) {
                return true;
            }
        }
        return false;
    }

    private HttpCodecUtil() {
    }
}
