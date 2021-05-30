package org.apache.commons.httpclient;

import org.apache.commons.httpclient.util.LangUtils;

public class UsernamePasswordCredentials implements Credentials {
    private String password;
    private String userName;

    public UsernamePasswordCredentials() {
    }

    public UsernamePasswordCredentials(String usernamePassword) {
        if (usernamePassword != null) {
            int atColon = usernamePassword.indexOf(58);
            if (atColon >= 0) {
                this.userName = usernamePassword.substring(0, atColon);
                this.password = usernamePassword.substring(atColon + 1);
                return;
            }
            this.userName = usernamePassword;
            return;
        }
        throw new IllegalArgumentException("Username:password string may not be null");
    }

    public UsernamePasswordCredentials(String userName2, String password2) {
        if (userName2 != null) {
            this.userName = userName2;
            this.password = password2;
            return;
        }
        throw new IllegalArgumentException("Username may not be null");
    }

    public void setUserName(String userName2) {
        if (userName2 != null) {
            this.userName = userName2;
            return;
        }
        throw new IllegalArgumentException("Username may not be null");
    }

    public String getUserName() {
        return this.userName;
    }

    public void setPassword(String password2) {
        this.password = password2;
    }

    public String getPassword() {
        return this.password;
    }

    public String toString() {
        StringBuffer result = new StringBuffer();
        result.append(this.userName);
        result.append(":");
        result.append(this.password == null ? "null" : this.password);
        return result.toString();
    }

    public int hashCode() {
        return LangUtils.hashCode(LangUtils.hashCode(17, (Object) this.userName), (Object) this.password);
    }

    public boolean equals(Object o) {
        if (o == null) {
            return false;
        }
        if (this == o) {
            return true;
        }
        if (getClass().equals(o.getClass())) {
            UsernamePasswordCredentials that = (UsernamePasswordCredentials) o;
            if (!LangUtils.equals(this.userName, that.userName) || !LangUtils.equals(this.password, that.password)) {
                return false;
            }
            return true;
        }
        return false;
    }
}
