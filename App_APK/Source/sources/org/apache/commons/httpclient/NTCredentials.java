package org.apache.commons.httpclient;

import org.apache.commons.httpclient.util.LangUtils;

public class NTCredentials extends UsernamePasswordCredentials {
    private String domain;
    private String host;

    public NTCredentials() {
    }

    public NTCredentials(String userName, String password, String host2, String domain2) {
        super(userName, password);
        if (domain2 != null) {
            this.domain = domain2;
            if (host2 != null) {
                this.host = host2;
                return;
            }
            throw new IllegalArgumentException("Host may not be null");
        }
        throw new IllegalArgumentException("Domain may not be null");
    }

    public void setDomain(String domain2) {
        if (domain2 != null) {
            this.domain = domain2;
            return;
        }
        throw new IllegalArgumentException("Domain may not be null");
    }

    public String getDomain() {
        return this.domain;
    }

    public void setHost(String host2) {
        if (host2 != null) {
            this.host = host2;
            return;
        }
        throw new IllegalArgumentException("Host may not be null");
    }

    public String getHost() {
        return this.host;
    }

    public String toString() {
        StringBuffer sbResult = new StringBuffer(super.toString());
        sbResult.append("@");
        sbResult.append(this.host);
        sbResult.append(".");
        sbResult.append(this.domain);
        return sbResult.toString();
    }

    public int hashCode() {
        return LangUtils.hashCode(LangUtils.hashCode(super.hashCode(), (Object) this.host), (Object) this.domain);
    }

    public boolean equals(Object o) {
        if (o == null) {
            return false;
        }
        if (this == o) {
            return true;
        }
        if (!super.equals(o) || !(o instanceof NTCredentials)) {
            return false;
        }
        NTCredentials that = (NTCredentials) o;
        if (!LangUtils.equals(this.domain, that.domain) || !LangUtils.equals(this.host, that.host)) {
            return false;
        }
        return true;
    }
}
