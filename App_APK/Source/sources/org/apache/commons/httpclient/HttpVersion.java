package org.apache.commons.httpclient;

public class HttpVersion implements Comparable {
    public static final HttpVersion HTTP_0_9 = new HttpVersion(0, 9);
    public static final HttpVersion HTTP_1_0 = new HttpVersion(1, 0);
    public static final HttpVersion HTTP_1_1 = new HttpVersion(1, 1);
    private int major = 0;
    private int minor = 0;

    public HttpVersion(int major2, int minor2) {
        if (major2 >= 0) {
            this.major = major2;
            if (minor2 >= 0) {
                this.minor = minor2;
                return;
            }
            throw new IllegalArgumentException("HTTP minor version number may not be negative");
        }
        throw new IllegalArgumentException("HTTP major version number may not be negative");
    }

    public int getMajor() {
        return this.major;
    }

    public int getMinor() {
        return this.minor;
    }

    public int hashCode() {
        return (this.major * 100000) + this.minor;
    }

    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof HttpVersion)) {
            return false;
        }
        return equals((HttpVersion) obj);
    }

    public int compareTo(HttpVersion anotherVer) {
        if (anotherVer != null) {
            int delta = getMajor() - anotherVer.getMajor();
            if (delta == 0) {
                return getMinor() - anotherVer.getMinor();
            }
            return delta;
        }
        throw new IllegalArgumentException("Version parameter may not be null");
    }

    public int compareTo(Object o) {
        return compareTo((HttpVersion) o);
    }

    public boolean equals(HttpVersion version) {
        return compareTo(version) == 0;
    }

    public boolean greaterEquals(HttpVersion version) {
        return compareTo(version) >= 0;
    }

    public boolean lessEquals(HttpVersion version) {
        return compareTo(version) <= 0;
    }

    public String toString() {
        StringBuffer buffer = new StringBuffer();
        buffer.append("HTTP/");
        buffer.append(this.major);
        buffer.append('.');
        buffer.append(this.minor);
        return buffer.toString();
    }

    public static HttpVersion parse(String s) throws ProtocolException {
        if (s == null) {
            throw new IllegalArgumentException("String may not be null");
        } else if (s.startsWith("HTTP/")) {
            int i1 = "HTTP/".length();
            int i2 = s.indexOf(".", i1);
            if (i2 != -1) {
                try {
                    try {
                        return new HttpVersion(Integer.parseInt(s.substring(i1, i2)), Integer.parseInt(s.substring(i2 + 1, s.length())));
                    } catch (NumberFormatException e) {
                        StringBuffer stringBuffer = new StringBuffer();
                        stringBuffer.append("Invalid HTTP minor version number: ");
                        stringBuffer.append(s);
                        throw new ProtocolException(stringBuffer.toString());
                    }
                } catch (NumberFormatException e2) {
                    StringBuffer stringBuffer2 = new StringBuffer();
                    stringBuffer2.append("Invalid HTTP major version number: ");
                    stringBuffer2.append(s);
                    throw new ProtocolException(stringBuffer2.toString());
                }
            } else {
                StringBuffer stringBuffer3 = new StringBuffer();
                stringBuffer3.append("Invalid HTTP version number: ");
                stringBuffer3.append(s);
                throw new ProtocolException(stringBuffer3.toString());
            }
        } else {
            StringBuffer stringBuffer4 = new StringBuffer();
            stringBuffer4.append("Invalid HTTP version string: ");
            stringBuffer4.append(s);
            throw new ProtocolException(stringBuffer4.toString());
        }
    }
}
