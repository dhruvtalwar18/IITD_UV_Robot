package org.apache.commons.httpclient.cookie;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.StringTokenizer;
import org.apache.commons.httpclient.Cookie;
import org.apache.commons.httpclient.HeaderElement;
import org.apache.commons.httpclient.NameValuePair;

public class NetscapeDraftSpec extends CookieSpecBase {
    public Cookie[] parse(String host, int port, String path, boolean secure, String header) throws MalformedCookieException {
        String path2;
        int i = port;
        LOG.trace("enter NetscapeDraftSpec.parse(String, port, path, boolean, Header)");
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
            String host2 = host.toLowerCase();
            String defaultPath = path2;
            int lastSlashIndex = defaultPath.lastIndexOf(CookieSpec.PATH_DELIM);
            if (lastSlashIndex >= 0) {
                if (lastSlashIndex == 0) {
                    lastSlashIndex = 1;
                }
                defaultPath = defaultPath.substring(0, lastSlashIndex);
            }
            HeaderElement headerelement = new HeaderElement(header.toCharArray());
            Cookie cookie = new Cookie(host2, headerelement.getName(), headerelement.getValue(), defaultPath, (Date) null, false);
            NameValuePair[] parameters = headerelement.getParameters();
            if (parameters != null) {
                for (NameValuePair parseAttribute : parameters) {
                    parseAttribute(parseAttribute, cookie);
                }
            }
            return new Cookie[]{cookie};
        } else {
            throw new IllegalArgumentException("Header may not be null.");
        }
    }

    public void parseAttribute(NameValuePair attribute, Cookie cookie) throws MalformedCookieException {
        if (attribute == null) {
            throw new IllegalArgumentException("Attribute may not be null.");
        } else if (cookie != null) {
            String paramName = attribute.getName().toLowerCase();
            String paramValue = attribute.getValue();
            if (!paramName.equals("expires")) {
                super.parseAttribute(attribute, cookie);
            } else if (paramValue != null) {
                try {
                    cookie.setExpiryDate(new SimpleDateFormat("EEE, dd-MMM-yyyy HH:mm:ss z", Locale.US).parse(paramValue));
                } catch (ParseException e) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Invalid expires attribute: ");
                    stringBuffer.append(e.getMessage());
                    throw new MalformedCookieException(stringBuffer.toString());
                }
            } else {
                throw new MalformedCookieException("Missing value for expires attribute");
            }
        } else {
            throw new IllegalArgumentException("Cookie may not be null.");
        }
    }

    public boolean domainMatch(String host, String domain) {
        return host.endsWith(domain);
    }

    public void validate(String host, int port, String path, boolean secure, Cookie cookie) throws MalformedCookieException {
        LOG.trace("enterNetscapeDraftCookieProcessor RCF2109CookieProcessor.validate(Cookie)");
        super.validate(host, port, path, secure, cookie);
        if (host.indexOf(".") >= 0) {
            int domainParts = new StringTokenizer(cookie.getDomain(), ".").countTokens();
            if (isSpecialDomain(cookie.getDomain())) {
                if (domainParts < 2) {
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("Domain attribute \"");
                    stringBuffer.append(cookie.getDomain());
                    stringBuffer.append("\" violates the Netscape cookie specification for ");
                    stringBuffer.append("special domains");
                    throw new MalformedCookieException(stringBuffer.toString());
                }
            } else if (domainParts < 3) {
                StringBuffer stringBuffer2 = new StringBuffer();
                stringBuffer2.append("Domain attribute \"");
                stringBuffer2.append(cookie.getDomain());
                stringBuffer2.append("\" violates the Netscape cookie specification");
                throw new MalformedCookieException(stringBuffer2.toString());
            }
        }
    }

    private static boolean isSpecialDomain(String domain) {
        String ucDomain = domain.toUpperCase();
        if (ucDomain.endsWith(".COM") || ucDomain.endsWith(".EDU") || ucDomain.endsWith(".NET") || ucDomain.endsWith(".GOV") || ucDomain.endsWith(".MIL") || ucDomain.endsWith(".ORG") || ucDomain.endsWith(".INT")) {
            return true;
        }
        return false;
    }
}
