package org.jboss.netty.handler.codec.http;

import java.util.Date;
import java.util.Set;
import java.util.TreeSet;

public class CookieEncoder {
    private final Set<Cookie> cookies = new TreeSet();
    private final boolean server;

    public CookieEncoder(boolean server2) {
        this.server = server2;
    }

    public void addCookie(String name, String value) {
        this.cookies.add(new DefaultCookie(name, value));
    }

    public void addCookie(Cookie cookie) {
        this.cookies.add(cookie);
    }

    public String encode() {
        String answer;
        if (this.server) {
            answer = encodeServerSide();
        } else {
            answer = encodeClientSide();
        }
        this.cookies.clear();
        return answer;
    }

    private String encodeServerSide() {
        if (this.cookies.size() <= 1) {
            StringBuilder sb = new StringBuilder();
            for (Cookie cookie : this.cookies) {
                add(sb, cookie.getName(), cookie.getValue());
                if (cookie.getMaxAge() >= 0) {
                    if (cookie.getVersion() == 0) {
                        addUnquoted(sb, "Expires", new CookieDateFormat().format(new Date(System.currentTimeMillis() + (((long) cookie.getMaxAge()) * 1000))));
                    } else {
                        add(sb, "Max-Age", cookie.getMaxAge());
                    }
                }
                if (cookie.getPath() != null) {
                    if (cookie.getVersion() > 0) {
                        add(sb, "Path", cookie.getPath());
                    } else {
                        addUnquoted(sb, "Path", cookie.getPath());
                    }
                }
                if (cookie.getDomain() != null) {
                    if (cookie.getVersion() > 0) {
                        add(sb, "Domain", cookie.getDomain());
                    } else {
                        addUnquoted(sb, "Domain", cookie.getDomain());
                    }
                }
                if (cookie.isSecure()) {
                    sb.append("Secure");
                    sb.append(';');
                    sb.append(' ');
                }
                if (cookie.isHttpOnly()) {
                    sb.append("HTTPOnly");
                    sb.append(';');
                    sb.append(' ');
                }
                if (cookie.getVersion() >= 1) {
                    if (cookie.getComment() != null) {
                        add(sb, "Comment", cookie.getComment());
                    }
                    add(sb, "Version", 1);
                    if (cookie.getCommentUrl() != null) {
                        addQuoted(sb, "CommentURL", cookie.getCommentUrl());
                    }
                    if (!cookie.getPorts().isEmpty()) {
                        sb.append("Port");
                        sb.append('=');
                        sb.append('\"');
                        for (Integer intValue : cookie.getPorts()) {
                            sb.append(intValue.intValue());
                            sb.append(',');
                        }
                        sb.setCharAt(sb.length() - 1, '\"');
                        sb.append(';');
                        sb.append(' ');
                    }
                    if (cookie.isDiscard()) {
                        sb.append("Discard");
                        sb.append(';');
                        sb.append(' ');
                    }
                }
            }
            if (sb.length() > 0) {
                sb.setLength(sb.length() - 2);
            }
            return sb.toString();
        }
        throw new IllegalStateException("encode() can encode only one cookie on server mode: " + this.cookies.size() + " cookies added");
    }

    private String encodeClientSide() {
        StringBuilder sb = new StringBuilder();
        for (Cookie cookie : this.cookies) {
            if (cookie.getVersion() >= 1) {
                add(sb, "$Version", 1);
            }
            add(sb, cookie.getName(), cookie.getValue());
            if (cookie.getPath() != null) {
                add(sb, "$Path", cookie.getPath());
            }
            if (cookie.getDomain() != null) {
                add(sb, "$Domain", cookie.getDomain());
            }
            if (cookie.getVersion() >= 1 && !cookie.getPorts().isEmpty()) {
                sb.append('$');
                sb.append("Port");
                sb.append('=');
                sb.append('\"');
                for (Integer intValue : cookie.getPorts()) {
                    sb.append(intValue.intValue());
                    sb.append(',');
                }
                sb.setCharAt(sb.length() - 1, '\"');
                sb.append(';');
                sb.append(' ');
            }
        }
        if (sb.length() > 0) {
            sb.setLength(sb.length() - 2);
        }
        return sb.toString();
    }

    private static void add(StringBuilder sb, String name, String val) {
        if (val == null) {
            addQuoted(sb, name, "");
            return;
        }
        int i = 0;
        while (i < val.length()) {
            char c = val.charAt(i);
            if (!(c == 9 || c == ' ' || c == '\"' || c == ',' || c == '/' || c == '{' || c == '}')) {
                switch (c) {
                    case '(':
                    case ')':
                        break;
                    default:
                        switch (c) {
                            case ':':
                            case ';':
                            case '<':
                            case '=':
                            case '>':
                            case '?':
                            case '@':
                                break;
                            default:
                                switch (c) {
                                    case '[':
                                    case '\\':
                                    case ']':
                                        break;
                                    default:
                                        i++;
                                }
                        }
                }
            }
            addQuoted(sb, name, val);
            return;
        }
        addUnquoted(sb, name, val);
    }

    private static void addUnquoted(StringBuilder sb, String name, String val) {
        sb.append(name);
        sb.append('=');
        sb.append(val);
        sb.append(';');
        sb.append(' ');
    }

    private static void addQuoted(StringBuilder sb, String name, String val) {
        if (val == null) {
            val = "";
        }
        sb.append(name);
        sb.append('=');
        sb.append('\"');
        sb.append(val.replace("\\", "\\\\").replace("\"", "\\\""));
        sb.append('\"');
        sb.append(';');
        sb.append(' ');
    }

    private static void add(StringBuilder sb, String name, int val) {
        sb.append(name);
        sb.append('=');
        sb.append(val);
        sb.append(';');
        sb.append(' ');
    }
}
