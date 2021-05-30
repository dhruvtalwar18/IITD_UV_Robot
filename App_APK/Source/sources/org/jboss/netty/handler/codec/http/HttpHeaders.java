package org.jboss.netty.handler.codec.http;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import org.apache.xmlrpc.serializer.TypeSerializerImpl;
import org.jboss.netty.util.internal.CaseIgnoringComparator;

public class HttpHeaders {
    private static final int BUCKET_SIZE = 17;
    private final Entry[] entries = new Entry[17];
    private final Entry head = new Entry(-1, (String) null, (String) null);

    public static final class Names {
        public static final String ACCEPT = "Accept";
        public static final String ACCEPT_CHARSET = "Accept-Charset";
        public static final String ACCEPT_ENCODING = "Accept-Encoding";
        public static final String ACCEPT_LANGUAGE = "Accept-Language";
        public static final String ACCEPT_PATCH = "Accept-Patch";
        public static final String ACCEPT_RANGES = "Accept-Ranges";
        public static final String AGE = "Age";
        public static final String ALLOW = "Allow";
        public static final String AUTHORIZATION = "Authorization";
        public static final String CACHE_CONTROL = "Cache-Control";
        public static final String CONNECTION = "Connection";
        public static final String CONTENT_BASE = "Content-Base";
        public static final String CONTENT_ENCODING = "Content-Encoding";
        public static final String CONTENT_LANGUAGE = "Content-Language";
        public static final String CONTENT_LENGTH = "Content-Length";
        public static final String CONTENT_LOCATION = "Content-Location";
        public static final String CONTENT_MD5 = "Content-MD5";
        public static final String CONTENT_RANGE = "Content-Range";
        public static final String CONTENT_TRANSFER_ENCODING = "Content-Transfer-Encoding";
        public static final String CONTENT_TYPE = "Content-Type";
        public static final String COOKIE = "Cookie";
        public static final String DATE = "Date";
        public static final String ETAG = "ETag";
        public static final String EXPECT = "Expect";
        public static final String EXPIRES = "Expires";
        public static final String FROM = "From";
        public static final String HOST = "Host";
        public static final String IF_MATCH = "If-Match";
        public static final String IF_MODIFIED_SINCE = "If-Modified-Since";
        public static final String IF_NONE_MATCH = "If-None-Match";
        public static final String IF_RANGE = "If-Range";
        public static final String IF_UNMODIFIED_SINCE = "If-Unmodified-Since";
        public static final String LAST_MODIFIED = "Last-Modified";
        public static final String LOCATION = "Location";
        public static final String MAX_FORWARDS = "Max-Forwards";
        public static final String ORIGIN = "Origin";
        public static final String PRAGMA = "Pragma";
        public static final String PROXY_AUTHENTICATE = "Proxy-Authenticate";
        public static final String PROXY_AUTHORIZATION = "Proxy-Authorization";
        public static final String RANGE = "Range";
        public static final String REFERER = "Referer";
        public static final String RETRY_AFTER = "Retry-After";
        public static final String SEC_WEBSOCKET_ACCEPT = "Sec-WebSocket-Accept";
        public static final String SEC_WEBSOCKET_KEY = "Sec-WebSocket-Key";
        public static final String SEC_WEBSOCKET_KEY1 = "Sec-WebSocket-Key1";
        public static final String SEC_WEBSOCKET_KEY2 = "Sec-WebSocket-Key2";
        public static final String SEC_WEBSOCKET_LOCATION = "Sec-WebSocket-Location";
        public static final String SEC_WEBSOCKET_ORIGIN = "Sec-WebSocket-Origin";
        public static final String SEC_WEBSOCKET_PROTOCOL = "Sec-WebSocket-Protocol";
        public static final String SEC_WEBSOCKET_VERSION = "Sec-WebSocket-Version";
        public static final String SERVER = "Server";
        public static final String SET_COOKIE = "Set-Cookie";
        public static final String SET_COOKIE2 = "Set-Cookie2";
        public static final String TE = "TE";
        public static final String TRAILER = "Trailer";
        public static final String TRANSFER_ENCODING = "Transfer-Encoding";
        public static final String UPGRADE = "Upgrade";
        public static final String USER_AGENT = "User-Agent";
        public static final String VARY = "Vary";
        public static final String VIA = "Via";
        public static final String WARNING = "Warning";
        public static final String WEBSOCKET_LOCATION = "WebSocket-Location";
        public static final String WEBSOCKET_ORIGIN = "WebSocket-Origin";
        public static final String WEBSOCKET_PROTOCOL = "WebSocket-Protocol";
        public static final String WWW_AUTHENTICATE = "WWW-Authenticate";

        private Names() {
        }
    }

    public static final class Values {
        public static final String APPLICATION_X_WWW_FORM_URLENCODED = "application/x-www-form-urlencoded";
        public static final String BASE64 = "base64";
        public static final String BINARY = "binary";
        public static final String BOUNDARY = "boundary";
        public static final String BYTES = "bytes";
        public static final String CHARSET = "charset";
        public static final String CHUNKED = "chunked";
        public static final String CLOSE = "close";
        public static final String COMPRESS = "compress";
        public static final String CONTINUE = "100-continue";
        public static final String DEFLATE = "deflate";
        public static final String GZIP = "gzip";
        public static final String IDENTITY = "identity";
        public static final String KEEP_ALIVE = "keep-alive";
        public static final String MAX_AGE = "max-age";
        public static final String MAX_STALE = "max-stale";
        public static final String MIN_FRESH = "min-fresh";
        public static final String MULTIPART_FORM_DATA = "multipart/form-data";
        public static final String MUST_REVALIDATE = "must-revalidate";
        public static final String NONE = "none";
        public static final String NO_CACHE = "no-cache";
        public static final String NO_STORE = "no-store";
        public static final String NO_TRANSFORM = "no-transform";
        public static final String ONLY_IF_CACHED = "only-if-cached";
        public static final String PRIVATE = "private";
        public static final String PROXY_REVALIDATE = "proxy-revalidate";
        public static final String PUBLIC = "public";
        public static final String QUOTED_PRINTABLE = "quoted-printable";
        public static final String S_MAXAGE = "s-maxage";
        public static final String TRAILERS = "trailers";
        public static final String UPGRADE = "Upgrade";
        public static final String WEBSOCKET = "WebSocket";

        private Values() {
        }
    }

    public static boolean isKeepAlive(HttpMessage message) {
        String connection = message.getHeader("Connection");
        if ("close".equalsIgnoreCase(connection)) {
            return false;
        }
        if (message.getProtocolVersion().isKeepAliveDefault()) {
            return !"close".equalsIgnoreCase(connection);
        }
        return "keep-alive".equalsIgnoreCase(connection);
    }

    public static void setKeepAlive(HttpMessage message, boolean keepAlive) {
        if (message.getProtocolVersion().isKeepAliveDefault()) {
            if (keepAlive) {
                message.removeHeader("Connection");
            } else {
                message.setHeader("Connection", (Object) "close");
            }
        } else if (keepAlive) {
            message.setHeader("Connection", (Object) "keep-alive");
        } else {
            message.removeHeader("Connection");
        }
    }

    public static String getHeader(HttpMessage message, String name) {
        return message.getHeader(name);
    }

    public static String getHeader(HttpMessage message, String name, String defaultValue) {
        String value = message.getHeader(name);
        if (value == null) {
            return defaultValue;
        }
        return value;
    }

    public static void setHeader(HttpMessage message, String name, Object value) {
        message.setHeader(name, value);
    }

    public static void setHeader(HttpMessage message, String name, Iterable<?> values) {
        message.setHeader(name, values);
    }

    public static void addHeader(HttpMessage message, String name, Object value) {
        message.addHeader(name, value);
    }

    public static int getIntHeader(HttpMessage message, String name) {
        String value = getHeader(message, name);
        if (value != null) {
            return Integer.parseInt(value);
        }
        throw new NumberFormatException("null");
    }

    public static int getIntHeader(HttpMessage message, String name, int defaultValue) {
        String value = getHeader(message, name);
        if (value == null) {
            return defaultValue;
        }
        try {
            return Integer.parseInt(value);
        } catch (NumberFormatException e) {
            return defaultValue;
        }
    }

    public static void setIntHeader(HttpMessage message, String name, int value) {
        message.setHeader(name, (Object) Integer.valueOf(value));
    }

    public static void setIntHeader(HttpMessage message, String name, Iterable<Integer> values) {
        message.setHeader(name, (Iterable<?>) values);
    }

    public static void addIntHeader(HttpMessage message, String name, int value) {
        message.addHeader(name, Integer.valueOf(value));
    }

    public static long getContentLength(HttpMessage message) {
        return getContentLength(message, 0);
    }

    /* JADX WARNING: Removed duplicated region for block: B:22:0x005a A[RETURN] */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public static long getContentLength(org.jboss.netty.handler.codec.http.HttpMessage r4, long r5) {
        /*
            java.lang.String r0 = "Content-Length"
            java.lang.String r0 = r4.getHeader(r0)
            if (r0 == 0) goto L_0x000d
            long r1 = java.lang.Long.parseLong(r0)
            return r1
        L_0x000d:
            boolean r1 = r4 instanceof org.jboss.netty.handler.codec.http.HttpRequest
            if (r1 == 0) goto L_0x0034
            r1 = r4
            org.jboss.netty.handler.codec.http.HttpRequest r1 = (org.jboss.netty.handler.codec.http.HttpRequest) r1
            org.jboss.netty.handler.codec.http.HttpMethod r2 = org.jboss.netty.handler.codec.http.HttpMethod.GET
            org.jboss.netty.handler.codec.http.HttpMethod r3 = r1.getMethod()
            boolean r2 = r2.equals(r3)
            if (r2 == 0) goto L_0x0033
            java.lang.String r2 = "Sec-WebSocket-Key1"
            boolean r2 = r1.containsHeader(r2)
            if (r2 == 0) goto L_0x0033
            java.lang.String r2 = "Sec-WebSocket-Key2"
            boolean r2 = r1.containsHeader(r2)
            if (r2 == 0) goto L_0x0033
            r2 = 8
            return r2
        L_0x0033:
            goto L_0x005a
        L_0x0034:
            boolean r1 = r4 instanceof org.jboss.netty.handler.codec.http.HttpResponse
            if (r1 == 0) goto L_0x005a
            r1 = r4
            org.jboss.netty.handler.codec.http.HttpResponse r1 = (org.jboss.netty.handler.codec.http.HttpResponse) r1
            org.jboss.netty.handler.codec.http.HttpResponseStatus r2 = r1.getStatus()
            int r2 = r2.getCode()
            r3 = 101(0x65, float:1.42E-43)
            if (r2 != r3) goto L_0x005a
            java.lang.String r2 = "Sec-WebSocket-Origin"
            boolean r2 = r1.containsHeader(r2)
            if (r2 == 0) goto L_0x005a
            java.lang.String r2 = "Sec-WebSocket-Location"
            boolean r2 = r1.containsHeader(r2)
            if (r2 == 0) goto L_0x005a
            r2 = 16
            return r2
        L_0x005a:
            return r5
        */
        throw new UnsupportedOperationException("Method not decompiled: org.jboss.netty.handler.codec.http.HttpHeaders.getContentLength(org.jboss.netty.handler.codec.http.HttpMessage, long):long");
    }

    public static void setContentLength(HttpMessage message, long length) {
        message.setHeader("Content-Length", (Object) Long.valueOf(length));
    }

    public static String getHost(HttpMessage message) {
        return message.getHeader("Host");
    }

    public static String getHost(HttpMessage message, String defaultValue) {
        return getHeader(message, "Host", defaultValue);
    }

    public static void setHost(HttpMessage message, String value) {
        message.setHeader("Host", (Object) value);
    }

    public static boolean is100ContinueExpected(HttpMessage message) {
        String value;
        if (!(message instanceof HttpRequest) || message.getProtocolVersion().compareTo(HttpVersion.HTTP_1_1) < 0 || (value = message.getHeader("Expect")) == null) {
            return false;
        }
        if ("100-continue".equalsIgnoreCase(value)) {
            return true;
        }
        for (String v : message.getHeaders("Expect")) {
            if ("100-continue".equalsIgnoreCase(v)) {
                return true;
            }
        }
        return false;
    }

    public static void set100ContinueExpected(HttpMessage message) {
        set100ContinueExpected(message, true);
    }

    public static void set100ContinueExpected(HttpMessage message, boolean set) {
        if (set) {
            message.setHeader("Expect", (Object) "100-continue");
        } else {
            message.removeHeader("Expect");
        }
    }

    private static int hash(String name) {
        int h = 0;
        for (int i = name.length() - 1; i >= 0; i--) {
            char c = name.charAt(i);
            if (c >= 'A' && c <= 'Z') {
                c = (char) (c + ' ');
            }
            h = (h * 31) + c;
        }
        if (h > 0) {
            return h;
        }
        if (h == Integer.MIN_VALUE) {
            return Integer.MAX_VALUE;
        }
        return -h;
    }

    private static boolean eq(String name1, String name2) {
        int nameLen = name1.length();
        if (nameLen != name2.length()) {
            return false;
        }
        for (int i = nameLen - 1; i >= 0; i--) {
            char c1 = name1.charAt(i);
            char c2 = name2.charAt(i);
            if (c1 != c2) {
                if (c1 >= 'A' && c1 <= 'Z') {
                    c1 = (char) (c1 + ' ');
                }
                if (c2 >= 'A' && c2 <= 'Z') {
                    c2 = (char) (c2 + ' ');
                }
                if (c1 != c2) {
                    return false;
                }
            }
        }
        return true;
    }

    private static int index(int hash) {
        return hash % 17;
    }

    HttpHeaders() {
        Entry entry = this.head;
        Entry entry2 = this.head;
        Entry entry3 = this.head;
        entry2.after = entry3;
        entry.before = entry3;
    }

    /* access modifiers changed from: package-private */
    public void validateHeaderName(String name) {
        HttpCodecUtil.validateHeaderName(name);
    }

    /* access modifiers changed from: package-private */
    public void addHeader(String name, Object value) {
        validateHeaderName(name);
        String strVal = toString(value);
        HttpCodecUtil.validateHeaderValue(strVal);
        int h = hash(name);
        addHeader0(h, index(h), name, strVal);
    }

    private void addHeader0(int h, int i, String name, String value) {
        Entry e = this.entries[i];
        Entry[] entryArr = this.entries;
        Entry entry = new Entry(h, name, value);
        Entry newEntry = entry;
        entryArr[i] = entry;
        newEntry.next = e;
        newEntry.addBefore(this.head);
    }

    /* access modifiers changed from: package-private */
    public void removeHeader(String name) {
        if (name != null) {
            int h = hash(name);
            removeHeader0(h, index(h), name);
            return;
        }
        throw new NullPointerException("name");
    }

    private void removeHeader0(int h, int i, String name) {
        Entry e = this.entries[i];
        if (e != null) {
            while (e.hash == h && eq(name, e.key)) {
                e.remove();
                Entry next = e.next;
                if (next != null) {
                    this.entries[i] = next;
                    e = next;
                } else {
                    this.entries[i] = null;
                    return;
                }
            }
            while (true) {
                Entry next2 = e.next;
                if (next2 != null) {
                    if (next2.hash != h || !eq(name, next2.key)) {
                        e = next2;
                    } else {
                        e.next = next2.next;
                        next2.remove();
                    }
                } else {
                    return;
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public void setHeader(String name, Object value) {
        validateHeaderName(name);
        String strVal = toString(value);
        HttpCodecUtil.validateHeaderValue(strVal);
        int h = hash(name);
        int i = index(h);
        removeHeader0(h, i, name);
        addHeader0(h, i, name, strVal);
    }

    /* access modifiers changed from: package-private */
    public void setHeader(String name, Iterable<?> values) {
        Object v;
        if (values != null) {
            validateHeaderName(name);
            int h = hash(name);
            int i = index(h);
            removeHeader0(h, i, name);
            Iterator i$ = values.iterator();
            while (i$.hasNext() && (v = i$.next()) != null) {
                String strVal = toString(v);
                HttpCodecUtil.validateHeaderValue(strVal);
                addHeader0(h, i, name, strVal);
            }
            return;
        }
        throw new NullPointerException("values");
    }

    /* access modifiers changed from: package-private */
    public void clearHeaders() {
        for (int i = 0; i < this.entries.length; i++) {
            this.entries[i] = null;
        }
        Entry entry = this.head;
        Entry entry2 = this.head;
        Entry entry3 = this.head;
        entry2.after = entry3;
        entry.before = entry3;
    }

    /* access modifiers changed from: package-private */
    public String getHeader(String name) {
        if (name != null) {
            int h = hash(name);
            for (Entry e = this.entries[index(h)]; e != null; e = e.next) {
                if (e.hash == h && eq(name, e.key)) {
                    return e.value;
                }
            }
            return null;
        }
        throw new NullPointerException("name");
    }

    /* access modifiers changed from: package-private */
    public List<String> getHeaders(String name) {
        if (name != null) {
            LinkedList<String> values = new LinkedList<>();
            int h = hash(name);
            for (Entry e = this.entries[index(h)]; e != null; e = e.next) {
                if (e.hash == h && eq(name, e.key)) {
                    values.addFirst(e.value);
                }
            }
            return values;
        }
        throw new NullPointerException("name");
    }

    /* access modifiers changed from: package-private */
    public List<Map.Entry<String, String>> getHeaders() {
        List<Map.Entry<String, String>> all = new LinkedList<>();
        for (Entry e = this.head.after; e != this.head; e = e.after) {
            all.add(e);
        }
        return all;
    }

    /* access modifiers changed from: package-private */
    public boolean containsHeader(String name) {
        return getHeader(name) != null;
    }

    /* access modifiers changed from: package-private */
    public Set<String> getHeaderNames() {
        Set<String> names = new TreeSet<>(CaseIgnoringComparator.INSTANCE);
        for (Entry e = this.head.after; e != this.head; e = e.after) {
            names.add(e.key);
        }
        return names;
    }

    private static String toString(Object value) {
        if (value == null) {
            return null;
        }
        return value.toString();
    }

    private static final class Entry implements Map.Entry<String, String> {
        Entry after;
        Entry before;
        final int hash;
        final String key;
        Entry next;
        String value;

        Entry(int hash2, String key2, String value2) {
            this.hash = hash2;
            this.key = key2;
            this.value = value2;
        }

        /* access modifiers changed from: package-private */
        public void remove() {
            this.before.after = this.after;
            this.after.before = this.before;
        }

        /* access modifiers changed from: package-private */
        public void addBefore(Entry e) {
            this.after = e;
            this.before = e.before;
            this.before.after = this;
            this.after.before = this;
        }

        public String getKey() {
            return this.key;
        }

        public String getValue() {
            return this.value;
        }

        public String setValue(String value2) {
            if (value2 != null) {
                HttpCodecUtil.validateHeaderValue(value2);
                String oldValue = this.value;
                this.value = value2;
                return oldValue;
            }
            throw new NullPointerException(TypeSerializerImpl.VALUE_TAG);
        }

        public String toString() {
            return this.key + "=" + this.value;
        }
    }
}
