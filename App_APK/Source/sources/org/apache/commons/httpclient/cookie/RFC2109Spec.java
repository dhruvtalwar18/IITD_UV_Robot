package org.apache.commons.httpclient.cookie;

import org.apache.commons.httpclient.Cookie;
import org.apache.commons.httpclient.NameValuePair;
import org.apache.commons.httpclient.util.ParameterFormatter;

public class RFC2109Spec extends CookieSpecBase {
    public static final String SET_COOKIE_KEY = "set-cookie";
    private final ParameterFormatter formatter = new ParameterFormatter();

    public RFC2109Spec() {
        this.formatter.setAlwaysUseQuotes(true);
    }

    public void parseAttribute(NameValuePair attribute, Cookie cookie) throws MalformedCookieException {
        if (attribute == null) {
            throw new IllegalArgumentException("Attribute may not be null.");
        } else if (cookie != null) {
            String paramName = attribute.getName().toLowerCase();
            String paramValue = attribute.getValue();
            if (paramName.equals(Cookie2.PATH)) {
                if (paramValue == null) {
                    throw new MalformedCookieException("Missing value for path attribute");
                } else if (!paramValue.trim().equals("")) {
                    cookie.setPath(paramValue);
                    cookie.setPathAttributeSpecified(true);
                } else {
                    throw new MalformedCookieException("Blank value for path attribute");
                }
            } else if (!paramName.equals("version")) {
                super.parseAttribute(attribute, cookie);
            } else if (paramValue != null) {
                try {
                    cookie.setVersion(Integer.parseInt(paramValue));
                } catch (NumberFormatException e) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Invalid version: ");
                    stringBuffer.append(e.getMessage());
                    throw new MalformedCookieException(stringBuffer.toString());
                }
            } else {
                throw new MalformedCookieException("Missing value for version attribute");
            }
        } else {
            throw new IllegalArgumentException("Cookie may not be null.");
        }
    }

    public void validate(String host, int port, String path, boolean secure, Cookie cookie) throws MalformedCookieException {
        LOG.trace("enter RFC2109Spec.validate(String, int, String, boolean, Cookie)");
        super.validate(host, port, path, secure, cookie);
        if (cookie.getName().indexOf(32) != -1) {
            throw new MalformedCookieException("Cookie name may not contain blanks");
        } else if (cookie.getName().startsWith("$")) {
            throw new MalformedCookieException("Cookie name may not start with $");
        } else if (cookie.isDomainAttributeSpecified() && !cookie.getDomain().equals(host)) {
            if (cookie.getDomain().startsWith(".")) {
                int dotIndex = cookie.getDomain().indexOf(46, 1);
                if (dotIndex < 0 || dotIndex == cookie.getDomain().length() - 1) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Domain attribute \"");
                    stringBuffer.append(cookie.getDomain());
                    stringBuffer.append("\" violates RFC 2109: domain must contain an embedded dot");
                    throw new MalformedCookieException(stringBuffer.toString());
                }
                String host2 = host.toLowerCase();
                if (!host2.endsWith(cookie.getDomain())) {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("Illegal domain attribute \"");
                    stringBuffer2.append(cookie.getDomain());
                    stringBuffer2.append("\". Domain of origin: \"");
                    stringBuffer2.append(host2);
                    stringBuffer2.append("\"");
                    throw new MalformedCookieException(stringBuffer2.toString());
                } else if (host2.substring(0, host2.length() - cookie.getDomain().length()).indexOf(46) != -1) {
                    StringBuffer stringBuffer3 = new StringBuffer();
                    stringBuffer3.append("Domain attribute \"");
                    stringBuffer3.append(cookie.getDomain());
                    stringBuffer3.append("\" violates RFC 2109: host minus domain may not contain any dots");
                    throw new MalformedCookieException(stringBuffer3.toString());
                }
            } else {
                StringBuffer stringBuffer4 = new StringBuffer();
                stringBuffer4.append("Domain attribute \"");
                stringBuffer4.append(cookie.getDomain());
                stringBuffer4.append("\" violates RFC 2109: domain must start with a dot");
                throw new MalformedCookieException(stringBuffer4.toString());
            }
        }
    }

    public boolean domainMatch(String host, String domain) {
        return host.equals(domain) || (domain.startsWith(".") && host.endsWith(domain));
    }

    private void formatParam(StringBuffer buffer, NameValuePair param, int version) {
        if (version < 1) {
            buffer.append(param.getName());
            buffer.append("=");
            if (param.getValue() != null) {
                buffer.append(param.getValue());
                return;
            }
            return;
        }
        this.formatter.format(buffer, param);
    }

    private void formatCookieAsVer(StringBuffer buffer, Cookie cookie, int version) {
        String value = cookie.getValue();
        if (value == null) {
            value = "";
        }
        formatParam(buffer, new NameValuePair(cookie.getName(), value), version);
        if (cookie.getPath() != null && cookie.isPathAttributeSpecified()) {
            buffer.append("; ");
            formatParam(buffer, new NameValuePair("$Path", cookie.getPath()), version);
        }
        if (cookie.getDomain() != null && cookie.isDomainAttributeSpecified()) {
            buffer.append("; ");
            formatParam(buffer, new NameValuePair("$Domain", cookie.getDomain()), version);
        }
    }

    public String formatCookie(Cookie cookie) {
        LOG.trace("enter RFC2109Spec.formatCookie(Cookie)");
        if (cookie != null) {
            int version = cookie.getVersion();
            StringBuffer buffer = new StringBuffer();
            formatParam(buffer, new NameValuePair("$Version", Integer.toString(version)), version);
            buffer.append("; ");
            formatCookieAsVer(buffer, cookie, version);
            return buffer.toString();
        }
        throw new IllegalArgumentException("Cookie may not be null");
    }

    public String formatCookies(Cookie[] cookies) {
        LOG.trace("enter RFC2109Spec.formatCookieHeader(Cookie[])");
        int version = Integer.MAX_VALUE;
        for (Cookie cookie : cookies) {
            if (cookie.getVersion() < version) {
                version = cookie.getVersion();
            }
        }
        StringBuffer buffer = new StringBuffer();
        formatParam(buffer, new NameValuePair("$Version", Integer.toString(version)), version);
        for (Cookie formatCookieAsVer : cookies) {
            buffer.append("; ");
            formatCookieAsVer(buffer, formatCookieAsVer, version);
        }
        return buffer.toString();
    }
}
