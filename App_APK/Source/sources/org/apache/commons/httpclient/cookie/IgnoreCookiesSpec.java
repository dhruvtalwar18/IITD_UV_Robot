package org.apache.commons.httpclient.cookie;

import java.util.Collection;
import org.apache.commons.httpclient.Cookie;
import org.apache.commons.httpclient.Header;
import org.apache.commons.httpclient.NameValuePair;

public class IgnoreCookiesSpec implements CookieSpec {
    public Cookie[] parse(String host, int port, String path, boolean secure, String header) throws MalformedCookieException {
        return new Cookie[0];
    }

    public Collection getValidDateFormats() {
        return null;
    }

    public void setValidDateFormats(Collection datepatterns) {
    }

    public String formatCookie(Cookie cookie) {
        return null;
    }

    public Header formatCookieHeader(Cookie cookie) throws IllegalArgumentException {
        return null;
    }

    public Header formatCookieHeader(Cookie[] cookies) throws IllegalArgumentException {
        return null;
    }

    public String formatCookies(Cookie[] cookies) throws IllegalArgumentException {
        return null;
    }

    public boolean match(String host, int port, String path, boolean secure, Cookie cookie) {
        return false;
    }

    public Cookie[] match(String host, int port, String path, boolean secure, Cookie[] cookies) {
        return new Cookie[0];
    }

    public Cookie[] parse(String host, int port, String path, boolean secure, Header header) throws MalformedCookieException, IllegalArgumentException {
        return new Cookie[0];
    }

    public void parseAttribute(NameValuePair attribute, Cookie cookie) throws MalformedCookieException, IllegalArgumentException {
    }

    public void validate(String host, int port, String path, boolean secure, Cookie cookie) throws MalformedCookieException, IllegalArgumentException {
    }

    public boolean domainMatch(String host, String domain) {
        return false;
    }

    public boolean pathMatch(String path, String topmostPath) {
        return false;
    }
}
