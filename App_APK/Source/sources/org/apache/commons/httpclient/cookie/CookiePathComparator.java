package org.apache.commons.httpclient.cookie;

import java.util.Comparator;
import org.apache.commons.httpclient.Cookie;

public class CookiePathComparator implements Comparator {
    private String normalizePath(Cookie cookie) {
        String path = cookie.getPath();
        if (path == null) {
            path = CookieSpec.PATH_DELIM;
        }
        if (path.endsWith(CookieSpec.PATH_DELIM)) {
            return path;
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append(path);
        stringBuffer.append(CookieSpec.PATH_DELIM);
        return stringBuffer.toString();
    }

    public int compare(Object o1, Object o2) {
        String path1 = normalizePath((Cookie) o1);
        String path2 = normalizePath((Cookie) o2);
        if (path1.equals(path2)) {
            return 0;
        }
        if (path1.startsWith(path2)) {
            return -1;
        }
        if (path2.startsWith(path1)) {
            return 1;
        }
        return 0;
    }
}
