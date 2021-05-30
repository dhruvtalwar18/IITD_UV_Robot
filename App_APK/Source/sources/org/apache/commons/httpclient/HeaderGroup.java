package org.apache.commons.httpclient;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class HeaderGroup {
    private List headers = new ArrayList();

    public void clear() {
        this.headers.clear();
    }

    public void addHeader(Header header) {
        this.headers.add(header);
    }

    public void removeHeader(Header header) {
        this.headers.remove(header);
    }

    public void setHeaders(Header[] headers2) {
        clear();
        for (Header addHeader : headers2) {
            addHeader(addHeader);
        }
    }

    public Header getCondensedHeader(String name) {
        Header[] headers2 = getHeaders(name);
        if (headers2.length == 0) {
            return null;
        }
        int i = 1;
        if (headers2.length == 1) {
            return new Header(headers2[0].getName(), headers2[0].getValue());
        }
        StringBuffer valueBuffer = new StringBuffer(headers2[0].getValue());
        while (true) {
            int i2 = i;
            if (i2 >= headers2.length) {
                return new Header(name.toLowerCase(), valueBuffer.toString());
            }
            valueBuffer.append(", ");
            valueBuffer.append(headers2[i2].getValue());
            i = i2 + 1;
        }
    }

    public Header[] getHeaders(String name) {
        ArrayList headersFound = new ArrayList();
        for (Header header : this.headers) {
            if (header.getName().equalsIgnoreCase(name)) {
                headersFound.add(header);
            }
        }
        return (Header[]) headersFound.toArray(new Header[headersFound.size()]);
    }

    public Header getFirstHeader(String name) {
        for (Header header : this.headers) {
            if (header.getName().equalsIgnoreCase(name)) {
                return header;
            }
        }
        return null;
    }

    public Header getLastHeader(String name) {
        for (int i = this.headers.size() - 1; i >= 0; i--) {
            Header header = (Header) this.headers.get(i);
            if (header.getName().equalsIgnoreCase(name)) {
                return header;
            }
        }
        return null;
    }

    public Header[] getAllHeaders() {
        return (Header[]) this.headers.toArray(new Header[this.headers.size()]);
    }

    public boolean containsHeader(String name) {
        for (Header header : this.headers) {
            if (header.getName().equalsIgnoreCase(name)) {
                return true;
            }
        }
        return false;
    }

    public Iterator getIterator() {
        return this.headers.iterator();
    }
}
