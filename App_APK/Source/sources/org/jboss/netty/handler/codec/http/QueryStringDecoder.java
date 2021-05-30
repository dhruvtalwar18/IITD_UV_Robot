package org.jboss.netty.handler.codec.http;

import java.io.UnsupportedEncodingException;
import java.net.URI;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class QueryStringDecoder {
    private static final int DEFAULT_MAX_PARAMS = 1024;
    private final Charset charset;
    private final boolean hasPath;
    private final int maxParams;
    private int nParams;
    private Map<String, List<String>> params;
    private String path;
    private final String uri;

    public QueryStringDecoder(String uri2) {
        this(uri2, HttpConstants.DEFAULT_CHARSET);
    }

    public QueryStringDecoder(String uri2, boolean hasPath2) {
        this(uri2, HttpConstants.DEFAULT_CHARSET, hasPath2);
    }

    public QueryStringDecoder(String uri2, Charset charset2) {
        this(uri2, charset2, true);
    }

    public QueryStringDecoder(String uri2, Charset charset2, boolean hasPath2) {
        this(uri2, charset2, hasPath2, 1024);
    }

    public QueryStringDecoder(String uri2, Charset charset2, boolean hasPath2, int maxParams2) {
        if (uri2 == null) {
            throw new NullPointerException("uri");
        } else if (charset2 == null) {
            throw new NullPointerException("charset");
        } else if (maxParams2 > 0) {
            this.uri = uri2.replace(';', '&');
            this.charset = charset2;
            this.maxParams = maxParams2;
            this.hasPath = hasPath2;
        } else {
            throw new IllegalArgumentException("maxParams: " + maxParams2 + " (expected: a positive integer)");
        }
    }

    @Deprecated
    public QueryStringDecoder(String uri2, String charset2) {
        this(uri2, Charset.forName(charset2));
    }

    public QueryStringDecoder(URI uri2) {
        this(uri2, HttpConstants.DEFAULT_CHARSET);
    }

    public QueryStringDecoder(URI uri2, Charset charset2) {
        this(uri2, charset2, 1024);
    }

    public QueryStringDecoder(URI uri2, Charset charset2, int maxParams2) {
        if (uri2 == null) {
            throw new NullPointerException("uri");
        } else if (charset2 == null) {
            throw new NullPointerException("charset");
        } else if (maxParams2 > 0) {
            String rawPath = uri2.getRawPath();
            if (rawPath != null) {
                this.hasPath = true;
            } else {
                rawPath = "";
                this.hasPath = false;
            }
            this.uri = (rawPath + "?" + uri2.getRawQuery()).replace(';', '&');
            this.charset = charset2;
            this.maxParams = maxParams2;
        } else {
            throw new IllegalArgumentException("maxParams: " + maxParams2 + " (expected: a positive integer)");
        }
    }

    @Deprecated
    public QueryStringDecoder(URI uri2, String charset2) {
        this(uri2, Charset.forName(charset2));
    }

    public String getPath() {
        if (this.path == null) {
            if (!this.hasPath) {
                this.path = "";
                return "";
            }
            int pathEndPos = this.uri.indexOf(63);
            if (pathEndPos < 0) {
                this.path = this.uri;
            } else {
                String substring = this.uri.substring(0, pathEndPos);
                this.path = substring;
                return substring;
            }
        }
        return this.path;
    }

    public Map<String, List<String>> getParameters() {
        if (this.params == null) {
            if (this.hasPath) {
                int pathLength = getPath().length();
                if (this.uri.length() == pathLength) {
                    return Collections.emptyMap();
                }
                decodeParams(this.uri.substring(pathLength + 1));
            } else if (this.uri.length() == 0) {
                return Collections.emptyMap();
            } else {
                decodeParams(this.uri);
            }
        }
        return this.params;
    }

    private void decodeParams(String s) {
        Map<String, List<String>> params2 = new LinkedHashMap<>();
        this.params = params2;
        int i = 0;
        this.nParams = 0;
        String name = null;
        int pos = 0;
        while (i < s.length()) {
            char c = s.charAt(i);
            if (c == '=' && name == null) {
                if (pos != i) {
                    name = decodeComponent(s.substring(pos, i), this.charset);
                }
                pos = i + 1;
            } else if (c != '&') {
                continue;
            } else {
                if (name != null || pos == i) {
                    if (name != null) {
                        if (addParam(params2, name, decodeComponent(s.substring(pos, i), this.charset))) {
                            name = null;
                        } else {
                            return;
                        }
                    }
                } else if (!addParam(params2, decodeComponent(s.substring(pos, i), this.charset), "")) {
                    return;
                }
                pos = i + 1;
            }
            i++;
        }
        if (pos != i) {
            if (name == null) {
                if (addParam(params2, decodeComponent(s.substring(pos, i), this.charset), "")) {
                }
            } else if (addParam(params2, name, decodeComponent(s.substring(pos, i), this.charset))) {
            }
        } else if (name != null && !addParam(params2, name, "")) {
        }
    }

    private boolean addParam(Map<String, List<String>> params2, String name, String value) {
        if (this.nParams >= this.maxParams) {
            return false;
        }
        List<String> values = params2.get(name);
        if (values == null) {
            values = new ArrayList<>(1);
            params2.put(name, values);
        }
        values.add(value);
        this.nParams++;
        return true;
    }

    public static String decodeComponent(String s) {
        return decodeComponent(s, HttpConstants.DEFAULT_CHARSET);
    }

    public static String decodeComponent(String s, Charset charset2) {
        int pos;
        char c;
        char c2;
        if (s == null) {
            return "";
        }
        int size = s.length();
        boolean modified = false;
        int i = 0;
        while (i < size) {
            char c3 = s.charAt(i);
            if (c3 == '%') {
                i++;
            } else if (c3 != '+') {
                i++;
            }
            modified = true;
            i++;
        }
        if (!modified) {
            return s;
        }
        byte[] buf = new byte[size];
        int pos2 = 0;
        int i2 = 0;
        char c4 = 0;
        while (i2 < size) {
            char c5 = s.charAt(i2);
            if (c5 != '%') {
                if (c5 != '+') {
                    char c6 = c5;
                    c = c4;
                    c2 = c6;
                } else {
                    pos = pos2 + 1;
                    buf[pos2] = 32;
                    pos2 = pos;
                    i2++;
                }
            } else if (i2 != size - 1) {
                i2++;
                char c7 = s.charAt(i2);
                if (c7 == '%') {
                    pos = pos2 + 1;
                    buf[pos2] = 37;
                    pos2 = pos;
                    i2++;
                } else if (i2 != size - 1) {
                    char c8 = decodeHexNibble(c7);
                    i2++;
                    c = decodeHexNibble(s.charAt(i2));
                    if (c8 == 65535 || c == 65535) {
                        StringBuilder sb = new StringBuilder();
                        sb.append("invalid escape sequence `%");
                        sb.append(s.charAt(i2 - 1));
                        sb.append(s.charAt(i2));
                        sb.append("' at index ");
                        sb.append(i2 - 2);
                        sb.append(" of: ");
                        sb.append(s);
                        throw new IllegalArgumentException(sb.toString());
                    }
                    c2 = (char) ((c8 * 16) + c);
                } else {
                    throw new IllegalArgumentException("partial escape sequence at end of string: " + s);
                }
            } else {
                throw new IllegalArgumentException("unterminated escape sequence at end of string: " + s);
            }
            pos = pos2 + 1;
            buf[pos2] = (byte) c2;
            c4 = c;
            pos2 = pos;
            i2++;
        }
        try {
            return new String(buf, 0, pos2, charset2.name());
        } catch (UnsupportedEncodingException e) {
            throw new IllegalArgumentException("unsupported encoding: " + charset2.name());
        }
    }

    private static char decodeHexNibble(char c) {
        if ('0' <= c && c <= '9') {
            return (char) (c - '0');
        }
        if ('a' <= c && c <= 'f') {
            return (char) ((c - 'a') + 10);
        }
        if ('A' > c || c > 'F') {
            return 65535;
        }
        return (char) ((c - 'A') + 10);
    }
}
