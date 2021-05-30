package org.apache.commons.httpclient;

import java.io.Serializable;
import java.util.Comparator;
import java.util.Date;
import org.apache.commons.httpclient.cookie.CookiePolicy;
import org.apache.commons.httpclient.cookie.CookieSpec;
import org.apache.commons.httpclient.util.LangUtils;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class Cookie extends NameValuePair implements Serializable, Comparator {
    private static final Log LOG;
    static /* synthetic */ Class class$org$apache$commons$httpclient$Cookie;
    private String cookieComment;
    private String cookieDomain;
    private Date cookieExpiryDate;
    private String cookiePath;
    private int cookieVersion;
    private boolean hasDomainAttribute;
    private boolean hasPathAttribute;
    private boolean isSecure;

    public Cookie() {
        this((String) null, "noname", (String) null, (String) null, (Date) null, false);
    }

    public Cookie(String domain, String name, String value) {
        this(domain, name, value, (String) null, (Date) null, false);
    }

    public Cookie(String domain, String name, String value, String path, Date expires, boolean secure) {
        super(name, value);
        this.hasPathAttribute = false;
        this.hasDomainAttribute = false;
        this.cookieVersion = 0;
        LOG.trace("enter Cookie(String, String, String, String, Date, boolean)");
        if (name == null) {
            throw new IllegalArgumentException("Cookie name may not be null");
        } else if (!name.trim().equals("")) {
            setPath(path);
            setDomain(domain);
            setExpiryDate(expires);
            setSecure(secure);
        } else {
            throw new IllegalArgumentException("Cookie name may not be blank");
        }
    }

    public Cookie(String domain, String name, String value, String path, int maxAge, boolean secure) {
        this(domain, name, value, path, (Date) null, secure);
        if (maxAge < -1) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid max age:  ");
            stringBuffer.append(Integer.toString(maxAge));
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (maxAge >= 0) {
            setExpiryDate(new Date(System.currentTimeMillis() + (((long) maxAge) * 1000)));
        }
    }

    public String getComment() {
        return this.cookieComment;
    }

    public void setComment(String comment) {
        this.cookieComment = comment;
    }

    public Date getExpiryDate() {
        return this.cookieExpiryDate;
    }

    public void setExpiryDate(Date expiryDate) {
        this.cookieExpiryDate = expiryDate;
    }

    public boolean isPersistent() {
        return this.cookieExpiryDate != null;
    }

    public String getDomain() {
        return this.cookieDomain;
    }

    public void setDomain(String domain) {
        if (domain != null) {
            int ndx = domain.indexOf(":");
            if (ndx != -1) {
                domain = domain.substring(0, ndx);
            }
            this.cookieDomain = domain.toLowerCase();
        }
    }

    public String getPath() {
        return this.cookiePath;
    }

    public void setPath(String path) {
        this.cookiePath = path;
    }

    public boolean getSecure() {
        return this.isSecure;
    }

    public void setSecure(boolean secure) {
        this.isSecure = secure;
    }

    public int getVersion() {
        return this.cookieVersion;
    }

    public void setVersion(int version) {
        this.cookieVersion = version;
    }

    public boolean isExpired() {
        return this.cookieExpiryDate != null && this.cookieExpiryDate.getTime() <= System.currentTimeMillis();
    }

    public boolean isExpired(Date now) {
        return this.cookieExpiryDate != null && this.cookieExpiryDate.getTime() <= now.getTime();
    }

    public void setPathAttributeSpecified(boolean value) {
        this.hasPathAttribute = value;
    }

    public boolean isPathAttributeSpecified() {
        return this.hasPathAttribute;
    }

    public void setDomainAttributeSpecified(boolean value) {
        this.hasDomainAttribute = value;
    }

    public boolean isDomainAttributeSpecified() {
        return this.hasDomainAttribute;
    }

    public int hashCode() {
        return LangUtils.hashCode(LangUtils.hashCode(LangUtils.hashCode(17, (Object) getName()), (Object) this.cookieDomain), (Object) this.cookiePath);
    }

    public boolean equals(Object obj) {
        if (obj == null) {
            return false;
        }
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof Cookie)) {
            return false;
        }
        Cookie that = (Cookie) obj;
        if (!LangUtils.equals(getName(), that.getName()) || !LangUtils.equals(this.cookieDomain, that.cookieDomain) || !LangUtils.equals(this.cookiePath, that.cookiePath)) {
            return false;
        }
        return true;
    }

    public String toExternalForm() {
        CookieSpec spec;
        if (getVersion() > 0) {
            spec = CookiePolicy.getDefaultSpec();
        } else {
            spec = CookiePolicy.getCookieSpec(CookiePolicy.NETSCAPE);
        }
        return spec.formatCookie(this);
    }

    public int compare(Object o1, Object o2) {
        LOG.trace("enter Cookie.compare(Object, Object)");
        if (!(o1 instanceof Cookie)) {
            throw new ClassCastException(o1.getClass().getName());
        } else if (o2 instanceof Cookie) {
            Cookie c1 = (Cookie) o1;
            Cookie c2 = (Cookie) o2;
            if (c1.getPath() == null && c2.getPath() == null) {
                return 0;
            }
            if (c1.getPath() == null) {
                if (c2.getPath().equals(CookieSpec.PATH_DELIM)) {
                    return 0;
                }
                return -1;
            } else if (c2.getPath() != null) {
                return c1.getPath().compareTo(c2.getPath());
            } else {
                if (c1.getPath().equals(CookieSpec.PATH_DELIM)) {
                    return 0;
                }
                return 1;
            }
        } else {
            throw new ClassCastException(o2.getClass().getName());
        }
    }

    public String toString() {
        return toExternalForm();
    }

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$Cookie == null) {
            cls = class$("org.apache.commons.httpclient.Cookie");
            class$org$apache$commons$httpclient$Cookie = cls;
        } else {
            cls = class$org$apache$commons$httpclient$Cookie;
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
}
