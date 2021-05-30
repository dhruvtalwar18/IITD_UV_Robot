package org.jboss.netty.handler.codec.http;

import java.util.HashMap;
import java.util.Map;
import org.apache.commons.httpclient.ConnectMethod;

public class HttpMethod implements Comparable<HttpMethod> {
    public static final HttpMethod CONNECT = new HttpMethod(ConnectMethod.NAME);
    public static final HttpMethod DELETE = new HttpMethod("DELETE");
    public static final HttpMethod GET = new HttpMethod("GET");
    public static final HttpMethod HEAD = new HttpMethod("HEAD");
    public static final HttpMethod OPTIONS = new HttpMethod("OPTIONS");
    public static final HttpMethod PATCH = new HttpMethod("PATCH");
    public static final HttpMethod POST = new HttpMethod("POST");
    public static final HttpMethod PUT = new HttpMethod("PUT");
    public static final HttpMethod TRACE = new HttpMethod("TRACE");
    private static final Map<String, HttpMethod> methodMap = new HashMap();
    private final String name;

    static {
        methodMap.put(OPTIONS.toString(), OPTIONS);
        methodMap.put(GET.toString(), GET);
        methodMap.put(HEAD.toString(), HEAD);
        methodMap.put(POST.toString(), POST);
        methodMap.put(PUT.toString(), PUT);
        methodMap.put(PATCH.toString(), PATCH);
        methodMap.put(DELETE.toString(), DELETE);
        methodMap.put(TRACE.toString(), TRACE);
        methodMap.put(CONNECT.toString(), CONNECT);
    }

    public static HttpMethod valueOf(String name2) {
        if (name2 != null) {
            String name3 = name2.trim().toUpperCase();
            if (name3.length() != 0) {
                HttpMethod result = methodMap.get(name3);
                if (result != null) {
                    return result;
                }
                return new HttpMethod(name3);
            }
            throw new IllegalArgumentException("empty name");
        }
        throw new NullPointerException("name");
    }

    public HttpMethod(String name2) {
        if (name2 != null) {
            String name3 = name2.trim().toUpperCase();
            if (name3.length() != 0) {
                for (int i = 0; i < name3.length(); i++) {
                    if (Character.isISOControl(name3.charAt(i)) || Character.isWhitespace(name3.charAt(i))) {
                        throw new IllegalArgumentException("invalid character in name");
                    }
                }
                this.name = name3;
                return;
            }
            throw new IllegalArgumentException("empty name");
        }
        throw new NullPointerException("name");
    }

    public String getName() {
        return this.name;
    }

    public int hashCode() {
        return getName().hashCode();
    }

    public boolean equals(Object o) {
        if (!(o instanceof HttpMethod)) {
            return false;
        }
        return getName().equals(((HttpMethod) o).getName());
    }

    public String toString() {
        return getName();
    }

    public int compareTo(HttpMethod o) {
        return getName().compareTo(o.getName());
    }
}
