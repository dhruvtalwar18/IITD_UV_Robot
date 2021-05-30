package org.apache.commons.httpclient.cookie;

import java.util.Collection;
import java.util.Date;
import java.util.LinkedList;
import java.util.List;
import org.apache.commons.httpclient.Cookie;
import org.apache.commons.httpclient.Header;
import org.apache.commons.httpclient.HeaderElement;
import org.apache.commons.httpclient.NameValuePair;
import org.apache.commons.httpclient.util.DateParseException;
import org.apache.commons.httpclient.util.DateUtil;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class CookieSpecBase implements CookieSpec {
    protected static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$cookie$CookieSpec;
    private Collection datepatterns = null;

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$cookie$CookieSpec == null) {
            cls = class$("org.apache.commons.httpclient.cookie.CookieSpec");
            class$org$apache$commons$httpclient$cookie$CookieSpec = cls;
        } else {
            cls = class$org$apache$commons$httpclient$cookie$CookieSpec;
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

    public Cookie[] parse(String host, int port, String path, boolean secure, String header) throws MalformedCookieException {
        String path2;
        boolean isNetscapeCookie;
        int i = port;
        String str = header;
        LOG.trace("enter CookieSpecBase.parse(String, port, path, boolean, Header)");
        if (host == null) {
            throw new IllegalArgumentException("Host of origin may not be null");
        } else if (host.trim().equals("")) {
            throw new IllegalArgumentException("Host of origin may not be blank");
        } else if (i < 0) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid port: ");
            stringBuffer.append(i);
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (path == null) {
            throw new IllegalArgumentException("Path of origin may not be null.");
        } else if (str != null) {
            if (path.trim().equals("")) {
                path2 = CookieSpec.PATH_DELIM;
            } else {
                path2 = path;
            }
            String host2 = host.toLowerCase();
            String defaultPath = path2;
            int lastSlashIndex = defaultPath.lastIndexOf(CookieSpec.PATH_DELIM);
            if (lastSlashIndex >= 0) {
                if (lastSlashIndex == 0) {
                    lastSlashIndex = 1;
                }
                defaultPath = defaultPath.substring(0, lastSlashIndex);
            }
            String defaultPath2 = defaultPath;
            boolean isNetscapeCookie2 = false;
            int i1 = header.toLowerCase().indexOf("expires=");
            if (i1 != -1) {
                int i12 = "expires=".length() + i1;
                int i2 = str.indexOf(";", i12);
                if (i2 == -1) {
                    i2 = header.length();
                }
                try {
                    DateUtil.parseDate(str.substring(i12, i2), this.datepatterns);
                    isNetscapeCookie2 = true;
                } catch (DateParseException e) {
                }
                isNetscapeCookie = isNetscapeCookie2;
            } else {
                isNetscapeCookie = false;
            }
            HeaderElement[] headerElements = isNetscapeCookie ? new HeaderElement[]{new HeaderElement(header.toCharArray())} : HeaderElement.parseElements(header.toCharArray());
            Cookie[] cookies = new Cookie[headerElements.length];
            int i3 = 0;
            while (true) {
                int i4 = i3;
                if (i4 < headerElements.length) {
                    HeaderElement headerelement = headerElements[i4];
                    try {
                        int i5 = i4;
                        Cookie[] cookies2 = cookies;
                        HeaderElement[] headerElements2 = headerElements;
                        try {
                            Cookie cookie = new Cookie(host2, headerelement.getName(), headerelement.getValue(), defaultPath2, (Date) null, false);
                            NameValuePair[] parameters = headerelement.getParameters();
                            if (parameters != null) {
                                for (NameValuePair parseAttribute : parameters) {
                                    parseAttribute(parseAttribute, cookie);
                                }
                            }
                            cookies2[i5] = cookie;
                            i3 = i5 + 1;
                            headerElements = headerElements2;
                            cookies = cookies2;
                        } catch (IllegalArgumentException e2) {
                            e = e2;
                            throw new MalformedCookieException(e.getMessage());
                        }
                    } catch (IllegalArgumentException e3) {
                        e = e3;
                        int i6 = i4;
                        Cookie[] cookieArr = cookies;
                        HeaderElement[] headerElementArr = headerElements;
                        throw new MalformedCookieException(e.getMessage());
                    }
                } else {
                    HeaderElement[] headerElementArr2 = headerElements;
                    return cookies;
                }
            }
        } else {
            throw new IllegalArgumentException("Header may not be null.");
        }
    }

    public Cookie[] parse(String host, int port, String path, boolean secure, Header header) throws MalformedCookieException {
        LOG.trace("enter CookieSpecBase.parse(String, port, path, boolean, String)");
        if (header != null) {
            return parse(host, port, path, secure, header.getValue());
        }
        throw new IllegalArgumentException("Header may not be null.");
    }

    public void parseAttribute(NameValuePair attribute, Cookie cookie) throws MalformedCookieException {
        if (attribute == null) {
            throw new IllegalArgumentException("Attribute may not be null.");
        } else if (cookie != null) {
            String paramName = attribute.getName().toLowerCase();
            String paramValue = attribute.getValue();
            if (paramName.equals(Cookie2.PATH)) {
                if (paramValue == null || paramValue.trim().equals("")) {
                    paramValue = CookieSpec.PATH_DELIM;
                }
                cookie.setPath(paramValue);
                cookie.setPathAttributeSpecified(true);
            } else if (paramName.equals(Cookie2.DOMAIN)) {
                if (paramValue == null) {
                    throw new MalformedCookieException("Missing value for domain attribute");
                } else if (!paramValue.trim().equals("")) {
                    cookie.setDomain(paramValue);
                    cookie.setDomainAttributeSpecified(true);
                } else {
                    throw new MalformedCookieException("Blank value for domain attribute");
                }
            } else if (paramName.equals("max-age")) {
                if (paramValue != null) {
                    try {
                        cookie.setExpiryDate(new Date(System.currentTimeMillis() + (((long) Integer.parseInt(paramValue)) * 1000)));
                    } catch (NumberFormatException e) {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Invalid max-age attribute: ");
                        stringBuffer.append(e.getMessage());
                        throw new MalformedCookieException(stringBuffer.toString());
                    }
                } else {
                    throw new MalformedCookieException("Missing value for max-age attribute");
                }
            } else if (paramName.equals(Cookie2.SECURE)) {
                cookie.setSecure(true);
            } else if (paramName.equals(Cookie2.COMMENT)) {
                cookie.setComment(paramValue);
            } else if (paramName.equals("expires")) {
                if (paramValue != null) {
                    try {
                        cookie.setExpiryDate(DateUtil.parseDate(paramValue, this.datepatterns));
                    } catch (DateParseException dpe) {
                        LOG.debug("Error parsing cookie date", dpe);
                        StringBuffer stringBuffer2 = new StringBuffer();
                        stringBuffer2.append("Unable to parse expiration date parameter: ");
                        stringBuffer2.append(paramValue);
                        throw new MalformedCookieException(stringBuffer2.toString());
                    }
                } else {
                    throw new MalformedCookieException("Missing value for expires attribute");
                }
            } else if (LOG.isDebugEnabled()) {
                Log log = LOG;
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Unrecognized cookie attribute: ");
                stringBuffer3.append(attribute.toString());
                log.debug(stringBuffer3.toString());
            }
        } else {
            throw new IllegalArgumentException("Cookie may not be null.");
        }
    }

    public Collection getValidDateFormats() {
        return this.datepatterns;
    }

    public void setValidDateFormats(Collection datepatterns2) {
        this.datepatterns = datepatterns2;
    }

    public void validate(String host, int port, String path, boolean secure, Cookie cookie) throws MalformedCookieException {
        LOG.trace("enter CookieSpecBase.validate(String, port, path, boolean, Cookie)");
        if (host == null) {
            throw new IllegalArgumentException("Host of origin may not be null");
        } else if (host.trim().equals("")) {
            throw new IllegalArgumentException("Host of origin may not be blank");
        } else if (port < 0) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid port: ");
            stringBuffer.append(port);
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (path != null) {
            if (path.trim().equals("")) {
                path = CookieSpec.PATH_DELIM;
            }
            String host2 = host.toLowerCase();
            if (cookie.getVersion() >= 0) {
                if (host2.indexOf(".") >= 0) {
                    if (!host2.endsWith(cookie.getDomain())) {
                        String s = cookie.getDomain();
                        if (s.startsWith(".")) {
                            s = s.substring(1, s.length());
                        }
                        if (!host2.equals(s)) {
                            StringBuffer stringBuffer2 = new StringBuffer();
                            stringBuffer2.append("Illegal domain attribute \"");
                            stringBuffer2.append(cookie.getDomain());
                            stringBuffer2.append("\". Domain of origin: \"");
                            stringBuffer2.append(host2);
                            stringBuffer2.append("\"");
                            throw new MalformedCookieException(stringBuffer2.toString());
                        }
                    }
                } else if (!host2.equals(cookie.getDomain())) {
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append("Illegal domain attribute \"");
                    stringBuffer3.append(cookie.getDomain());
                    stringBuffer3.append("\". Domain of origin: \"");
                    stringBuffer3.append(host2);
                    stringBuffer3.append("\"");
                    throw new MalformedCookieException(stringBuffer3.toString());
                }
                if (!path.startsWith(cookie.getPath())) {
                    StringBuffer stringBuffer4 = new StringBuffer();
                    stringBuffer4.append("Illegal path attribute \"");
                    stringBuffer4.append(cookie.getPath());
                    stringBuffer4.append("\". Path of origin: \"");
                    stringBuffer4.append(path);
                    stringBuffer4.append("\"");
                    throw new MalformedCookieException(stringBuffer4.toString());
                }
                return;
            }
            StringBuffer stringBuffer5 = new StringBuffer();
            stringBuffer5.append("Illegal version number ");
            stringBuffer5.append(cookie.getValue());
            throw new MalformedCookieException(stringBuffer5.toString());
        } else {
            throw new IllegalArgumentException("Path of origin may not be null.");
        }
    }

    public boolean match(String host, int port, String path, boolean secure, Cookie cookie) {
        LOG.trace("enter CookieSpecBase.match(String, int, String, boolean, Cookie");
        if (host == null) {
            throw new IllegalArgumentException("Host of origin may not be null");
        } else if (host.trim().equals("")) {
            throw new IllegalArgumentException("Host of origin may not be blank");
        } else if (port < 0) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid port: ");
            stringBuffer.append(port);
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (path == null) {
            throw new IllegalArgumentException("Path of origin may not be null.");
        } else if (cookie != null) {
            if (path.trim().equals("")) {
                path = CookieSpec.PATH_DELIM;
            }
            String host2 = host.toLowerCase();
            if (cookie.getDomain() == null) {
                LOG.warn("Invalid cookie state: domain not specified");
                return false;
            } else if (cookie.getPath() == null) {
                LOG.warn("Invalid cookie state: path not specified");
                return false;
            } else if ((cookie.getExpiryDate() != null && !cookie.getExpiryDate().after(new Date())) || !domainMatch(host2, cookie.getDomain()) || !pathMatch(path, cookie.getPath())) {
                return false;
            } else {
                if (!cookie.getSecure() || secure) {
                    return true;
                }
                return false;
            }
        } else {
            throw new IllegalArgumentException("Cookie may not be null");
        }
    }

    public boolean domainMatch(String host, String domain) {
        if (host.equals(domain)) {
            return true;
        }
        if (!domain.startsWith(".")) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(".");
            stringBuffer.append(domain);
            domain = stringBuffer.toString();
        }
        if (host.endsWith(domain) || host.equals(domain.substring(1))) {
            return true;
        }
        return false;
    }

    public boolean pathMatch(String path, String topmostPath) {
        boolean match = path.startsWith(topmostPath);
        if (!match || path.length() == topmostPath.length() || topmostPath.endsWith(CookieSpec.PATH_DELIM)) {
            return match;
        }
        return path.charAt(topmostPath.length()) == CookieSpec.PATH_DELIM_CHAR;
    }

    public Cookie[] match(String host, int port, String path, boolean secure, Cookie[] cookies) {
        LOG.trace("enter CookieSpecBase.match(String, int, String, boolean, Cookie[])");
        if (cookies == null) {
            return null;
        }
        List matching = new LinkedList();
        for (int i = 0; i < cookies.length; i++) {
            if (match(host, port, path, secure, cookies[i])) {
                addInPathOrder(matching, cookies[i]);
            }
        }
        return (Cookie[]) matching.toArray(new Cookie[matching.size()]);
    }

    private static void addInPathOrder(List list, Cookie addCookie) {
        int i = 0;
        while (i < list.size() && addCookie.compare(addCookie, (Cookie) list.get(i)) <= 0) {
            i++;
        }
        list.add(i, addCookie);
    }

    public String formatCookie(Cookie cookie) {
        LOG.trace("enter CookieSpecBase.formatCookie(Cookie)");
        if (cookie != null) {
            StringBuffer buf = new StringBuffer();
            buf.append(cookie.getName());
            buf.append("=");
            String s = cookie.getValue();
            if (s != null) {
                buf.append(s);
            }
            return buf.toString();
        }
        throw new IllegalArgumentException("Cookie may not be null");
    }

    public String formatCookies(Cookie[] cookies) throws IllegalArgumentException {
        LOG.trace("enter CookieSpecBase.formatCookies(Cookie[])");
        if (cookies == null) {
            throw new IllegalArgumentException("Cookie array may not be null");
        } else if (cookies.length != 0) {
            StringBuffer buffer = new StringBuffer();
            for (int i = 0; i < cookies.length; i++) {
                if (i > 0) {
                    buffer.append("; ");
                }
                buffer.append(formatCookie(cookies[i]));
            }
            return buffer.toString();
        } else {
            throw new IllegalArgumentException("Cookie array may not be empty");
        }
    }

    public Header formatCookieHeader(Cookie[] cookies) {
        LOG.trace("enter CookieSpecBase.formatCookieHeader(Cookie[])");
        return new Header("Cookie", formatCookies(cookies));
    }

    public Header formatCookieHeader(Cookie cookie) {
        LOG.trace("enter CookieSpecBase.formatCookieHeader(Cookie)");
        return new Header("Cookie", formatCookie(cookie));
    }
}
