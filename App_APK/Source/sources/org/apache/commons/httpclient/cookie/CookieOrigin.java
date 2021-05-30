package org.apache.commons.httpclient.cookie;

public final class CookieOrigin {
    private final String host;
    private final String path;
    private final int port;
    private final boolean secure;

    public CookieOrigin(String host2, int port2, String path2, boolean secure2) {
        if (host2 == null) {
            throw new IllegalArgumentException("Host of origin may not be null");
        } else if (host2.trim().equals("")) {
            throw new IllegalArgumentException("Host of origin may not be blank");
        } else if (port2 < 0) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Invalid port: ");
            stringBuffer.append(port2);
            throw new IllegalArgumentException(stringBuffer.toString());
        } else if (path2 != null) {
            this.host = host2;
            this.port = port2;
            this.path = path2;
            this.secure = secure2;
        } else {
            throw new IllegalArgumentException("Path of origin may not be null.");
        }
    }

    public String getHost() {
        return this.host;
    }

    public String getPath() {
        return this.path;
    }

    public int getPort() {
        return this.port;
    }

    public boolean isSecure() {
        return this.secure;
    }
}
