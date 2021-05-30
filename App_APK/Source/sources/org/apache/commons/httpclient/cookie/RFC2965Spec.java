package org.apache.commons.httpclient.cookie;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.StringTokenizer;
import org.apache.commons.httpclient.Cookie;
import org.apache.commons.httpclient.Header;
import org.apache.commons.httpclient.HeaderElement;
import org.apache.commons.httpclient.NameValuePair;
import org.apache.commons.httpclient.util.ParameterFormatter;
import org.apache.commons.logging.Log;

public class RFC2965Spec extends CookieSpecBase implements CookieVersionSupport {
    private static final Comparator PATH_COMPOARATOR = new CookiePathComparator();
    public static final String SET_COOKIE2_KEY = "set-cookie2";
    private final List attribHandlerList;
    private final Map attribHandlerMap;
    private final ParameterFormatter formatter = new ParameterFormatter();
    private final CookieSpec rfc2109;

    public RFC2965Spec() {
        this.formatter.setAlwaysUseQuotes(true);
        this.attribHandlerMap = new HashMap(10);
        this.attribHandlerList = new ArrayList(10);
        this.rfc2109 = new RFC2109Spec();
        registerAttribHandler(Cookie2.PATH, new Cookie2PathAttributeHandler());
        registerAttribHandler(Cookie2.DOMAIN, new Cookie2DomainAttributeHandler());
        registerAttribHandler("port", new Cookie2PortAttributeHandler());
        registerAttribHandler("max-age", new Cookie2MaxageAttributeHandler());
        registerAttribHandler(Cookie2.SECURE, new CookieSecureAttributeHandler());
        registerAttribHandler(Cookie2.COMMENT, new CookieCommentAttributeHandler());
        registerAttribHandler(Cookie2.COMMENTURL, new CookieCommentUrlAttributeHandler());
        registerAttribHandler(Cookie2.DISCARD, new CookieDiscardAttributeHandler());
        registerAttribHandler("version", new Cookie2VersionAttributeHandler());
    }

    /* access modifiers changed from: protected */
    public void registerAttribHandler(String name, CookieAttributeHandler handler) {
        if (name == null) {
            throw new IllegalArgumentException("Attribute name may not be null");
        } else if (handler != null) {
            if (!this.attribHandlerList.contains(handler)) {
                this.attribHandlerList.add(handler);
            }
            this.attribHandlerMap.put(name, handler);
        } else {
            throw new IllegalArgumentException("Attribute handler may not be null");
        }
    }

    /* access modifiers changed from: protected */
    public CookieAttributeHandler findAttribHandler(String name) {
        return (CookieAttributeHandler) this.attribHandlerMap.get(name);
    }

    /* access modifiers changed from: protected */
    public CookieAttributeHandler getAttribHandler(String name) {
        CookieAttributeHandler handler = findAttribHandler(name);
        if (handler != null) {
            return handler;
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Handler not registered for ");
        stringBuffer.append(name);
        stringBuffer.append(" attribute.");
        throw new IllegalStateException(stringBuffer.toString());
    }

    /* access modifiers changed from: protected */
    public Iterator getAttribHandlerIterator() {
        return this.attribHandlerList.iterator();
    }

    public Cookie[] parse(String host, int port, String path, boolean secure, Header header) throws MalformedCookieException {
        LOG.trace("enter RFC2965.parse(String, int, String, boolean, Header)");
        if (header == null) {
            throw new IllegalArgumentException("Header may not be null.");
        } else if (header.getName() == null) {
            throw new IllegalArgumentException("Header name may not be null.");
        } else if (header.getName().equalsIgnoreCase(SET_COOKIE2_KEY)) {
            return parse(host, port, path, secure, header.getValue());
        } else if (header.getName().equalsIgnoreCase(RFC2109Spec.SET_COOKIE_KEY)) {
            return this.rfc2109.parse(host, port, path, secure, header.getValue());
        } else {
            throw new MalformedCookieException("Header name is not valid. RFC 2965 supports \"set-cookie\" and \"set-cookie2\" headers.");
        }
    }

    public Cookie[] parse(String host, int port, String path, boolean secure, String header) throws MalformedCookieException {
        String path2;
        int i = port;
        LOG.trace("enter RFC2965Spec.parse(String, int, String, boolean, String)");
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
        } else if (header != null) {
            if (path.trim().equals("")) {
                path2 = CookieSpec.PATH_DELIM;
            } else {
                path2 = path;
            }
            String host2 = getEffectiveHost(host);
            HeaderElement[] headerElements = HeaderElement.parseElements(header.toCharArray());
            List cookies = new LinkedList();
            int i2 = 0;
            while (true) {
                int i3 = i2;
                if (i3 < headerElements.length) {
                    HeaderElement headerelement = headerElements[i3];
                    try {
                        Cookie2 cookie = new Cookie2(host2, headerelement.getName(), headerelement.getValue(), path2, (Date) null, false, new int[]{i});
                        NameValuePair[] parameters = headerelement.getParameters();
                        if (parameters != null) {
                            Map attribmap = new HashMap(parameters.length);
                            for (int j = parameters.length - 1; j >= 0; j--) {
                                NameValuePair param = parameters[j];
                                attribmap.put(param.getName().toLowerCase(), param);
                            }
                            for (Map.Entry entry : attribmap.entrySet()) {
                                parseAttribute((NameValuePair) entry.getValue(), cookie);
                            }
                        }
                        cookies.add(cookie);
                        i2 = i3 + 1;
                    } catch (IllegalArgumentException ex) {
                        throw new MalformedCookieException(ex.getMessage());
                    }
                } else {
                    return (Cookie[]) cookies.toArray(new Cookie[cookies.size()]);
                }
            }
        } else {
            throw new IllegalArgumentException("Header may not be null.");
        }
    }

    public void parseAttribute(NameValuePair attribute, Cookie cookie) throws MalformedCookieException {
        if (attribute == null) {
            throw new IllegalArgumentException("Attribute may not be null.");
        } else if (attribute.getName() == null) {
            throw new IllegalArgumentException("Attribute Name may not be null.");
        } else if (cookie != null) {
            String paramName = attribute.getName().toLowerCase();
            String paramValue = attribute.getValue();
            CookieAttributeHandler handler = findAttribHandler(paramName);
            if (handler != null) {
                handler.parse(cookie, paramValue);
            } else if (LOG.isDebugEnabled()) {
                Log log = LOG;
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Unrecognized cookie attribute: ");
                stringBuffer.append(attribute.toString());
                log.debug(stringBuffer.toString());
            }
        } else {
            throw new IllegalArgumentException("Cookie may not be null.");
        }
    }

    public void validate(String host, int port, String path, boolean secure, Cookie cookie) throws MalformedCookieException {
        LOG.trace("enter RFC2965Spec.validate(String, int, String, boolean, Cookie)");
        if (!(cookie instanceof Cookie2)) {
            this.rfc2109.validate(host, port, path, secure, cookie);
        } else if (cookie.getName().indexOf(32) != -1) {
            throw new MalformedCookieException("Cookie name may not contain blanks");
        } else if (!cookie.getName().startsWith("$")) {
            CookieOrigin origin = new CookieOrigin(getEffectiveHost(host), port, path, secure);
            Iterator i = getAttribHandlerIterator();
            while (i.hasNext()) {
                ((CookieAttributeHandler) i.next()).validate(cookie, origin);
            }
        } else {
            throw new MalformedCookieException("Cookie name may not start with $");
        }
    }

    public boolean match(String host, int port, String path, boolean secure, Cookie cookie) {
        LOG.trace("enter RFC2965.match(String, int, String, boolean, Cookie");
        if (cookie == null) {
            throw new IllegalArgumentException("Cookie may not be null");
        } else if (!(cookie instanceof Cookie2)) {
            return this.rfc2109.match(host, port, path, secure, cookie);
        } else {
            if (cookie.isPersistent() && cookie.isExpired()) {
                return false;
            }
            CookieOrigin origin = new CookieOrigin(getEffectiveHost(host), port, path, secure);
            Iterator i = getAttribHandlerIterator();
            while (i.hasNext()) {
                if (!((CookieAttributeHandler) i.next()).match(cookie, origin)) {
                    return false;
                }
            }
            return true;
        }
    }

    private void doFormatCookie2(Cookie2 cookie, StringBuffer buffer) {
        String name = cookie.getName();
        String value = cookie.getValue();
        if (value == null) {
            value = "";
        }
        this.formatter.format(buffer, new NameValuePair(name, value));
        if (cookie.getDomain() != null && cookie.isDomainAttributeSpecified()) {
            buffer.append("; ");
            this.formatter.format(buffer, new NameValuePair("$Domain", cookie.getDomain()));
        }
        if (cookie.getPath() != null && cookie.isPathAttributeSpecified()) {
            buffer.append("; ");
            this.formatter.format(buffer, new NameValuePair("$Path", cookie.getPath()));
        }
        if (cookie.isPortAttributeSpecified()) {
            String portValue = "";
            if (!cookie.isPortAttributeBlank()) {
                portValue = createPortAttribute(cookie.getPorts());
            }
            buffer.append("; ");
            this.formatter.format(buffer, new NameValuePair("$Port", portValue));
        }
    }

    public String formatCookie(Cookie cookie) {
        LOG.trace("enter RFC2965Spec.formatCookie(Cookie)");
        if (cookie == null) {
            throw new IllegalArgumentException("Cookie may not be null");
        } else if (!(cookie instanceof Cookie2)) {
            return this.rfc2109.formatCookie(cookie);
        } else {
            Cookie2 cookie2 = (Cookie2) cookie;
            int version = cookie2.getVersion();
            StringBuffer buffer = new StringBuffer();
            this.formatter.format(buffer, new NameValuePair("$Version", Integer.toString(version)));
            buffer.append("; ");
            doFormatCookie2(cookie2, buffer);
            return buffer.toString();
        }
    }

    public String formatCookies(Cookie[] cookies) {
        LOG.trace("enter RFC2965Spec.formatCookieHeader(Cookie[])");
        if (cookies != null) {
            boolean hasOldStyleCookie = false;
            int version = -1;
            int i = 0;
            while (true) {
                if (i >= cookies.length) {
                    break;
                }
                Cookie cookie = cookies[i];
                if (!(cookie instanceof Cookie2)) {
                    hasOldStyleCookie = true;
                    break;
                }
                if (cookie.getVersion() > version) {
                    version = cookie.getVersion();
                }
                i++;
            }
            if (version < 0) {
                version = 0;
            }
            if (hasOldStyleCookie || version < 1) {
                return this.rfc2109.formatCookies(cookies);
            }
            Arrays.sort(cookies, PATH_COMPOARATOR);
            StringBuffer buffer = new StringBuffer();
            this.formatter.format(buffer, new NameValuePair("$Version", Integer.toString(version)));
            for (Cookie2 cookie2 : cookies) {
                buffer.append("; ");
                doFormatCookie2(cookie2, buffer);
            }
            return buffer.toString();
        }
        throw new IllegalArgumentException("Cookies may not be null");
    }

    private String createPortAttribute(int[] ports) {
        StringBuffer portValue = new StringBuffer();
        int len = ports.length;
        for (int i = 0; i < len; i++) {
            if (i > 0) {
                portValue.append(",");
            }
            portValue.append(ports[i]);
        }
        return portValue.toString();
    }

    /* access modifiers changed from: private */
    public int[] parsePortAttribute(String portValue) throws MalformedCookieException {
        StringTokenizer st = new StringTokenizer(portValue, ",");
        int[] ports = new int[st.countTokens()];
        int i = 0;
        while (st.hasMoreTokens()) {
            try {
                ports[i] = Integer.parseInt(st.nextToken().trim());
                if (ports[i] >= 0) {
                    i++;
                } else {
                    throw new MalformedCookieException("Invalid Port attribute.");
                }
            } catch (NumberFormatException e) {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Invalid Port attribute: ");
                stringBuffer.append(e.getMessage());
                throw new MalformedCookieException(stringBuffer.toString());
            }
        }
        return ports;
    }

    private static String getEffectiveHost(String host) {
        String effectiveHost = host.toLowerCase();
        if (host.indexOf(46) >= 0) {
            return effectiveHost;
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(effectiveHost);
        stringBuffer.append(".local");
        return stringBuffer.toString();
    }

    public boolean domainMatch(String host, String domain) {
        return host.equals(domain) || (domain.startsWith(".") && host.endsWith(domain));
    }

    /* access modifiers changed from: private */
    public boolean portMatch(int port, int[] ports) {
        for (int i : ports) {
            if (port == i) {
                return true;
            }
        }
        return false;
    }

    private class Cookie2PathAttributeHandler implements CookieAttributeHandler {
        private Cookie2PathAttributeHandler() {
        }

        public void parse(Cookie cookie, String path) throws MalformedCookieException {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (path == null) {
                throw new MalformedCookieException("Missing value for path attribute");
            } else if (!path.trim().equals("")) {
                cookie.setPath(path);
                cookie.setPathAttributeSpecified(true);
            } else {
                throw new MalformedCookieException("Blank value for path attribute");
            }
        }

        public void validate(Cookie cookie, CookieOrigin origin) throws MalformedCookieException {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (origin != null) {
                String path = origin.getPath();
                if (path == null) {
                    throw new IllegalArgumentException("Path of origin host may not be null.");
                } else if (cookie.getPath() != null) {
                    if (path.trim().equals("")) {
                        path = CookieSpec.PATH_DELIM;
                    }
                    if (!RFC2965Spec.this.pathMatch(path, cookie.getPath())) {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Illegal path attribute \"");
                        stringBuffer.append(cookie.getPath());
                        stringBuffer.append("\". Path of origin: \"");
                        stringBuffer.append(path);
                        stringBuffer.append("\"");
                        throw new MalformedCookieException(stringBuffer.toString());
                    }
                } else {
                    throw new MalformedCookieException("Invalid cookie state: path attribute is null.");
                }
            } else {
                throw new IllegalArgumentException("Cookie origin may not be null");
            }
        }

        public boolean match(Cookie cookie, CookieOrigin origin) {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (origin != null) {
                String path = origin.getPath();
                if (cookie.getPath() == null) {
                    CookieSpecBase.LOG.warn("Invalid cookie state: path attribute is null.");
                    return false;
                }
                if (path.trim().equals("")) {
                    path = CookieSpec.PATH_DELIM;
                }
                if (!RFC2965Spec.this.pathMatch(path, cookie.getPath())) {
                    return false;
                }
                return true;
            } else {
                throw new IllegalArgumentException("Cookie origin may not be null");
            }
        }
    }

    private class Cookie2DomainAttributeHandler implements CookieAttributeHandler {
        private Cookie2DomainAttributeHandler() {
        }

        public void parse(Cookie cookie, String domain) throws MalformedCookieException {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (domain == null) {
                throw new MalformedCookieException("Missing value for domain attribute");
            } else if (!domain.trim().equals("")) {
                String domain2 = domain.toLowerCase();
                if (!domain2.startsWith(".")) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append(".");
                    stringBuffer.append(domain2);
                    domain2 = stringBuffer.toString();
                }
                cookie.setDomain(domain2);
                cookie.setDomainAttributeSpecified(true);
            } else {
                throw new MalformedCookieException("Blank value for domain attribute");
            }
        }

        public void validate(Cookie cookie, CookieOrigin origin) throws MalformedCookieException {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (origin != null) {
                String host = origin.getHost().toLowerCase();
                if (cookie.getDomain() != null) {
                    String cookieDomain = cookie.getDomain().toLowerCase();
                    if (cookie.isDomainAttributeSpecified()) {
                        if (cookieDomain.startsWith(".")) {
                            int dotIndex = cookieDomain.indexOf(46, 1);
                            if ((dotIndex < 0 || dotIndex == cookieDomain.length() - 1) && !cookieDomain.equals(".local")) {
                                StringBuffer stringBuffer = new StringBuffer();
                                stringBuffer.append("Domain attribute \"");
                                stringBuffer.append(cookie.getDomain());
                                stringBuffer.append("\" violates RFC 2965: the value contains no embedded dots ");
                                stringBuffer.append("and the value is not .local");
                                throw new MalformedCookieException(stringBuffer.toString());
                            } else if (!RFC2965Spec.this.domainMatch(host, cookieDomain)) {
                                StringBuffer stringBuffer2 = new StringBuffer();
                                stringBuffer2.append("Domain attribute \"");
                                stringBuffer2.append(cookie.getDomain());
                                stringBuffer2.append("\" violates RFC 2965: effective host name does not ");
                                stringBuffer2.append("domain-match domain attribute.");
                                throw new MalformedCookieException(stringBuffer2.toString());
                            } else if (host.substring(0, host.length() - cookieDomain.length()).indexOf(46) != -1) {
                                StringBuffer stringBuffer3 = new StringBuffer();
                                stringBuffer3.append("Domain attribute \"");
                                stringBuffer3.append(cookie.getDomain());
                                stringBuffer3.append("\" violates RFC 2965: ");
                                stringBuffer3.append("effective host minus domain may not contain any dots");
                                throw new MalformedCookieException(stringBuffer3.toString());
                            }
                        } else {
                            StringBuffer stringBuffer4 = new StringBuffer();
                            stringBuffer4.append("Domain attribute \"");
                            stringBuffer4.append(cookie.getDomain());
                            stringBuffer4.append("\" violates RFC 2109: domain must start with a dot");
                            throw new MalformedCookieException(stringBuffer4.toString());
                        }
                    } else if (!cookie.getDomain().equals(host)) {
                        StringBuffer stringBuffer5 = new StringBuffer();
                        stringBuffer5.append("Illegal domain attribute: \"");
                        stringBuffer5.append(cookie.getDomain());
                        stringBuffer5.append("\".");
                        stringBuffer5.append("Domain of origin: \"");
                        stringBuffer5.append(host);
                        stringBuffer5.append("\"");
                        throw new MalformedCookieException(stringBuffer5.toString());
                    }
                } else {
                    throw new MalformedCookieException("Invalid cookie state: domain not specified");
                }
            } else {
                throw new IllegalArgumentException("Cookie origin may not be null");
            }
        }

        public boolean match(Cookie cookie, CookieOrigin origin) {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (origin != null) {
                String host = origin.getHost().toLowerCase();
                String cookieDomain = cookie.getDomain();
                if (RFC2965Spec.this.domainMatch(host, cookieDomain) && host.substring(0, host.length() - cookieDomain.length()).indexOf(46) == -1) {
                    return true;
                }
                return false;
            } else {
                throw new IllegalArgumentException("Cookie origin may not be null");
            }
        }
    }

    private class Cookie2PortAttributeHandler implements CookieAttributeHandler {
        private Cookie2PortAttributeHandler() {
        }

        public void parse(Cookie cookie, String portValue) throws MalformedCookieException {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (cookie instanceof Cookie2) {
                Cookie2 cookie2 = (Cookie2) cookie;
                if (portValue == null || portValue.trim().equals("")) {
                    cookie2.setPortAttributeBlank(true);
                } else {
                    cookie2.setPorts(RFC2965Spec.this.parsePortAttribute(portValue));
                }
                cookie2.setPortAttributeSpecified(true);
            }
        }

        public void validate(Cookie cookie, CookieOrigin origin) throws MalformedCookieException {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (origin == null) {
                throw new IllegalArgumentException("Cookie origin may not be null");
            } else if (cookie instanceof Cookie2) {
                Cookie2 cookie2 = (Cookie2) cookie;
                int port = origin.getPort();
                if (cookie2.isPortAttributeSpecified() && !RFC2965Spec.this.portMatch(port, cookie2.getPorts())) {
                    throw new MalformedCookieException("Port attribute violates RFC 2965: Request port not found in cookie's port list.");
                }
            }
        }

        public boolean match(Cookie cookie, CookieOrigin origin) {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (origin == null) {
                throw new IllegalArgumentException("Cookie origin may not be null");
            } else if (!(cookie instanceof Cookie2)) {
                return false;
            } else {
                Cookie2 cookie2 = (Cookie2) cookie;
                int port = origin.getPort();
                if (!cookie2.isPortAttributeSpecified()) {
                    return true;
                }
                if (cookie2.getPorts() == null) {
                    CookieSpecBase.LOG.warn("Invalid cookie state: port not specified");
                    return false;
                } else if (!RFC2965Spec.this.portMatch(port, cookie2.getPorts())) {
                    return false;
                } else {
                    return true;
                }
            }
        }
    }

    private class Cookie2MaxageAttributeHandler implements CookieAttributeHandler {
        private Cookie2MaxageAttributeHandler() {
        }

        public void parse(Cookie cookie, String value) throws MalformedCookieException {
            int age;
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (value != null) {
                try {
                    age = Integer.parseInt(value);
                } catch (NumberFormatException e) {
                    age = -1;
                }
                if (age >= 0) {
                    cookie.setExpiryDate(new Date(System.currentTimeMillis() + (((long) age) * 1000)));
                    return;
                }
                throw new MalformedCookieException("Invalid max-age attribute.");
            } else {
                throw new MalformedCookieException("Missing value for max-age attribute");
            }
        }

        public void validate(Cookie cookie, CookieOrigin origin) {
        }

        public boolean match(Cookie cookie, CookieOrigin origin) {
            return true;
        }
    }

    private class CookieSecureAttributeHandler implements CookieAttributeHandler {
        private CookieSecureAttributeHandler() {
        }

        public void parse(Cookie cookie, String secure) throws MalformedCookieException {
            cookie.setSecure(true);
        }

        public void validate(Cookie cookie, CookieOrigin origin) throws MalformedCookieException {
        }

        public boolean match(Cookie cookie, CookieOrigin origin) {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (origin != null) {
                return cookie.getSecure() == origin.isSecure();
            } else {
                throw new IllegalArgumentException("Cookie origin may not be null");
            }
        }
    }

    private class CookieCommentAttributeHandler implements CookieAttributeHandler {
        private CookieCommentAttributeHandler() {
        }

        public void parse(Cookie cookie, String comment) throws MalformedCookieException {
            cookie.setComment(comment);
        }

        public void validate(Cookie cookie, CookieOrigin origin) throws MalformedCookieException {
        }

        public boolean match(Cookie cookie, CookieOrigin origin) {
            return true;
        }
    }

    private class CookieCommentUrlAttributeHandler implements CookieAttributeHandler {
        private CookieCommentUrlAttributeHandler() {
        }

        public void parse(Cookie cookie, String commenturl) throws MalformedCookieException {
            if (cookie instanceof Cookie2) {
                ((Cookie2) cookie).setCommentURL(commenturl);
            }
        }

        public void validate(Cookie cookie, CookieOrigin origin) throws MalformedCookieException {
        }

        public boolean match(Cookie cookie, CookieOrigin origin) {
            return true;
        }
    }

    private class CookieDiscardAttributeHandler implements CookieAttributeHandler {
        private CookieDiscardAttributeHandler() {
        }

        public void parse(Cookie cookie, String commenturl) throws MalformedCookieException {
            if (cookie instanceof Cookie2) {
                ((Cookie2) cookie).setDiscard(true);
            }
        }

        public void validate(Cookie cookie, CookieOrigin origin) throws MalformedCookieException {
        }

        public boolean match(Cookie cookie, CookieOrigin origin) {
            return true;
        }
    }

    private class Cookie2VersionAttributeHandler implements CookieAttributeHandler {
        private Cookie2VersionAttributeHandler() {
        }

        public void parse(Cookie cookie, String value) throws MalformedCookieException {
            int version;
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if (cookie instanceof Cookie2) {
                Cookie2 cookie2 = (Cookie2) cookie;
                if (value != null) {
                    try {
                        version = Integer.parseInt(value);
                    } catch (NumberFormatException e) {
                        version = -1;
                    }
                    if (version >= 0) {
                        cookie2.setVersion(version);
                        cookie2.setVersionAttributeSpecified(true);
                        return;
                    }
                    throw new MalformedCookieException("Invalid cookie version.");
                }
                throw new MalformedCookieException("Missing value for version attribute");
            }
        }

        public void validate(Cookie cookie, CookieOrigin origin) throws MalformedCookieException {
            if (cookie == null) {
                throw new IllegalArgumentException("Cookie may not be null");
            } else if ((cookie instanceof Cookie2) && !((Cookie2) cookie).isVersionAttributeSpecified()) {
                throw new MalformedCookieException("Violates RFC 2965. Version attribute is required.");
            }
        }

        public boolean match(Cookie cookie, CookieOrigin origin) {
            return true;
        }
    }

    public int getVersion() {
        return 1;
    }

    public Header getVersionHeader() {
        ParameterFormatter formatter2 = new ParameterFormatter();
        StringBuffer buffer = new StringBuffer();
        formatter2.format(buffer, new NameValuePair("$Version", Integer.toString(getVersion())));
        return new Header("Cookie2", buffer.toString(), true);
    }
}
