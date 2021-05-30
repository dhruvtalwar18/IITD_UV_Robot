package org.jboss.netty.handler.codec.spdy;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import org.apache.xmlrpc.serializer.TypeSerializerImpl;
import org.jboss.netty.handler.codec.http.HttpMethod;
import org.jboss.netty.handler.codec.http.HttpResponseStatus;
import org.jboss.netty.handler.codec.http.HttpVersion;

public class SpdyHeaders {
    private static final int BUCKET_SIZE = 17;
    private final Entry[] entries = new Entry[17];
    private final Entry head = new Entry(-1, (String) null, (String) null);

    public static final class HttpNames {
        public static final String HOST = ":host";
        public static final String METHOD = ":method";
        public static final String PATH = ":path";
        public static final String SCHEME = ":scheme";
        public static final String STATUS = ":status";
        public static final String VERSION = ":version";

        private HttpNames() {
        }
    }

    public static final class Spdy2HttpNames {
        public static final String METHOD = "method";
        public static final String SCHEME = "scheme";
        public static final String STATUS = "status";
        public static final String URL = "url";
        public static final String VERSION = "version";

        private Spdy2HttpNames() {
        }
    }

    public static String getHeader(SpdyHeaderBlock block, String name) {
        return block.getHeader(name);
    }

    public static String getHeader(SpdyHeaderBlock block, String name, String defaultValue) {
        String value = block.getHeader(name);
        if (value == null) {
            return defaultValue;
        }
        return value;
    }

    public static void setHeader(SpdyHeaderBlock block, String name, Object value) {
        block.setHeader(name, value);
    }

    public static void setHeader(SpdyHeaderBlock block, String name, Iterable<?> values) {
        block.setHeader(name, values);
    }

    public static void addHeader(SpdyHeaderBlock block, String name, Object value) {
        block.addHeader(name, value);
    }

    public static void removeHost(SpdyHeaderBlock block) {
        block.removeHeader(HttpNames.HOST);
    }

    public static String getHost(SpdyHeaderBlock block) {
        return block.getHeader(HttpNames.HOST);
    }

    public static void setHost(SpdyHeaderBlock block, String host) {
        block.setHeader(HttpNames.HOST, (Object) host);
    }

    @Deprecated
    public static void removeMethod(SpdyHeaderBlock block) {
        removeMethod(2, block);
    }

    public static void removeMethod(int spdyVersion, SpdyHeaderBlock block) {
        if (spdyVersion < 3) {
            block.removeHeader(Spdy2HttpNames.METHOD);
        } else {
            block.removeHeader(HttpNames.METHOD);
        }
    }

    @Deprecated
    public static HttpMethod getMethod(SpdyHeaderBlock block) {
        return getMethod(2, block);
    }

    public static HttpMethod getMethod(int spdyVersion, SpdyHeaderBlock block) {
        if (spdyVersion >= 3) {
            return HttpMethod.valueOf(block.getHeader(HttpNames.METHOD));
        }
        try {
            return HttpMethod.valueOf(block.getHeader(Spdy2HttpNames.METHOD));
        } catch (Exception e) {
            return null;
        }
    }

    @Deprecated
    public static void setMethod(SpdyHeaderBlock block, HttpMethod method) {
        setMethod(2, block, method);
    }

    public static void setMethod(int spdyVersion, SpdyHeaderBlock block, HttpMethod method) {
        if (spdyVersion < 3) {
            block.setHeader(Spdy2HttpNames.METHOD, (Object) method.getName());
        } else {
            block.setHeader(HttpNames.METHOD, (Object) method.getName());
        }
    }

    @Deprecated
    public static void removeScheme(SpdyHeaderBlock block) {
        removeMethod(2, block);
    }

    public static void removeScheme(int spdyVersion, SpdyHeaderBlock block) {
        if (spdyVersion < 2) {
            block.removeHeader(Spdy2HttpNames.SCHEME);
        } else {
            block.removeHeader(HttpNames.SCHEME);
        }
    }

    @Deprecated
    public static String getScheme(SpdyHeaderBlock block) {
        return getScheme(2, block);
    }

    public static String getScheme(int spdyVersion, SpdyHeaderBlock block) {
        if (spdyVersion < 3) {
            return block.getHeader(Spdy2HttpNames.SCHEME);
        }
        return block.getHeader(HttpNames.SCHEME);
    }

    @Deprecated
    public static void setScheme(SpdyHeaderBlock block, String scheme) {
        setScheme(2, block, scheme);
    }

    public static void setScheme(int spdyVersion, SpdyHeaderBlock block, String scheme) {
        if (spdyVersion < 3) {
            block.setHeader(Spdy2HttpNames.SCHEME, (Object) scheme);
        } else {
            block.setHeader(HttpNames.SCHEME, (Object) scheme);
        }
    }

    @Deprecated
    public static void removeStatus(SpdyHeaderBlock block) {
        removeMethod(2, block);
    }

    public static void removeStatus(int spdyVersion, SpdyHeaderBlock block) {
        if (spdyVersion < 3) {
            block.removeHeader("status");
        } else {
            block.removeHeader(HttpNames.STATUS);
        }
    }

    @Deprecated
    public static HttpResponseStatus getStatus(SpdyHeaderBlock block) {
        return getStatus(2, block);
    }

    public static HttpResponseStatus getStatus(int spdyVersion, SpdyHeaderBlock block) {
        String status;
        if (spdyVersion < 3) {
            try {
                status = block.getHeader("status");
            } catch (Exception e) {
                return null;
            }
        } else {
            status = block.getHeader(HttpNames.STATUS);
        }
        int space = status.indexOf(32);
        if (space == -1) {
            return HttpResponseStatus.valueOf(Integer.parseInt(status));
        }
        int code = Integer.parseInt(status.substring(0, space));
        String reasonPhrase = status.substring(space + 1);
        HttpResponseStatus responseStatus = HttpResponseStatus.valueOf(code);
        if (responseStatus.getReasonPhrase().equals(reasonPhrase)) {
            return responseStatus;
        }
        return new HttpResponseStatus(code, reasonPhrase);
    }

    @Deprecated
    public static void setStatus(SpdyHeaderBlock block, HttpResponseStatus status) {
        setStatus(2, block, status);
    }

    public static void setStatus(int spdyVersion, SpdyHeaderBlock block, HttpResponseStatus status) {
        if (spdyVersion < 3) {
            block.setHeader("status", (Object) status.toString());
        } else {
            block.setHeader(HttpNames.STATUS, (Object) status.toString());
        }
    }

    @Deprecated
    public static void removeUrl(SpdyHeaderBlock block) {
        removeUrl(2, block);
    }

    public static void removeUrl(int spdyVersion, SpdyHeaderBlock block) {
        if (spdyVersion < 3) {
            block.removeHeader("url");
        } else {
            block.removeHeader(HttpNames.PATH);
        }
    }

    @Deprecated
    public static String getUrl(SpdyHeaderBlock block) {
        return getUrl(2, block);
    }

    public static String getUrl(int spdyVersion, SpdyHeaderBlock block) {
        if (spdyVersion < 3) {
            return block.getHeader("url");
        }
        return block.getHeader(HttpNames.PATH);
    }

    @Deprecated
    public static void setUrl(SpdyHeaderBlock block, String path) {
        setUrl(2, block, path);
    }

    public static void setUrl(int spdyVersion, SpdyHeaderBlock block, String path) {
        if (spdyVersion < 3) {
            block.setHeader("url", (Object) path);
        } else {
            block.setHeader(HttpNames.PATH, (Object) path);
        }
    }

    @Deprecated
    public static void removeVersion(SpdyHeaderBlock block) {
        removeVersion(2, block);
    }

    public static void removeVersion(int spdyVersion, SpdyHeaderBlock block) {
        if (spdyVersion < 3) {
            block.removeHeader("version");
        } else {
            block.removeHeader(HttpNames.VERSION);
        }
    }

    @Deprecated
    public static HttpVersion getVersion(SpdyHeaderBlock block) {
        return getVersion(2, block);
    }

    public static HttpVersion getVersion(int spdyVersion, SpdyHeaderBlock block) {
        if (spdyVersion >= 3) {
            return HttpVersion.valueOf(block.getHeader(HttpNames.VERSION));
        }
        try {
            return HttpVersion.valueOf(block.getHeader("version"));
        } catch (Exception e) {
            return null;
        }
    }

    @Deprecated
    public static void setVersion(SpdyHeaderBlock block, HttpVersion httpVersion) {
        setVersion(2, block, httpVersion);
    }

    public static void setVersion(int spdyVersion, SpdyHeaderBlock block, HttpVersion httpVersion) {
        if (spdyVersion < 3) {
            block.setHeader("version", (Object) httpVersion.getText());
        } else {
            block.setHeader(HttpNames.VERSION, (Object) httpVersion.getText());
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

    SpdyHeaders() {
        Entry entry = this.head;
        Entry entry2 = this.head;
        Entry entry3 = this.head;
        entry2.after = entry3;
        entry.before = entry3;
    }

    /* access modifiers changed from: package-private */
    public void addHeader(String name, Object value) {
        String lowerCaseName = name.toLowerCase();
        SpdyCodecUtil.validateHeaderName(lowerCaseName);
        String strVal = toString(value);
        SpdyCodecUtil.validateHeaderValue(strVal);
        int h = hash(lowerCaseName);
        addHeader0(h, index(h), lowerCaseName, strVal);
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
            String lowerCaseName = name.toLowerCase();
            int h = hash(lowerCaseName);
            removeHeader0(h, index(h), lowerCaseName);
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
        String lowerCaseName = name.toLowerCase();
        SpdyCodecUtil.validateHeaderName(lowerCaseName);
        String strVal = toString(value);
        SpdyCodecUtil.validateHeaderValue(strVal);
        int h = hash(lowerCaseName);
        int i = index(h);
        removeHeader0(h, i, lowerCaseName);
        addHeader0(h, i, lowerCaseName, strVal);
    }

    /* access modifiers changed from: package-private */
    public void setHeader(String name, Iterable<?> values) {
        Object v;
        if (values != null) {
            String lowerCaseName = name.toLowerCase();
            SpdyCodecUtil.validateHeaderName(lowerCaseName);
            int h = hash(lowerCaseName);
            int i = index(h);
            removeHeader0(h, i, lowerCaseName);
            Iterator i$ = values.iterator();
            while (i$.hasNext() && (v = i$.next()) != null) {
                String strVal = toString(v);
                SpdyCodecUtil.validateHeaderValue(strVal);
                addHeader0(h, i, lowerCaseName, strVal);
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
        Set<String> names = new TreeSet<>();
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
                SpdyCodecUtil.validateHeaderValue(value2);
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
