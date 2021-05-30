package org.jboss.netty.handler.codec.http.multipart;

import java.nio.charset.Charset;
import org.jboss.netty.handler.codec.http.HttpConstants;

public abstract class AbstractHttpData implements HttpData {
    protected Charset charset = HttpConstants.DEFAULT_CHARSET;
    protected boolean completed;
    protected long definedSize;
    protected final String name;
    protected long size;

    public AbstractHttpData(String name2, Charset charset2, long size2) {
        if (name2 != null) {
            String name3 = name2.trim();
            if (name3.length() != 0) {
                int i = 0;
                while (i < name3.length()) {
                    char c = name3.charAt(i);
                    if (c <= 127) {
                        if (!(c == ' ' || c == ',' || c == ';' || c == '=')) {
                            switch (c) {
                                case 9:
                                case 10:
                                case 11:
                                case 12:
                                case 13:
                                    break;
                                default:
                                    i++;
                            }
                        }
                        throw new IllegalArgumentException("name contains one of the following prohibited characters: =,; \\t\\r\\n\\v\\f: " + name3);
                    }
                    throw new IllegalArgumentException("name contains non-ascii character: " + name3);
                }
                this.name = name3;
                if (charset2 != null) {
                    setCharset(charset2);
                }
                this.definedSize = size2;
                return;
            }
            throw new IllegalArgumentException("empty name");
        }
        throw new NullPointerException("name");
    }

    public String getName() {
        return this.name;
    }

    public boolean isCompleted() {
        return this.completed;
    }

    public Charset getCharset() {
        return this.charset;
    }

    public void setCharset(Charset charset2) {
        if (charset2 != null) {
            this.charset = charset2;
            return;
        }
        throw new NullPointerException("charset");
    }

    public long length() {
        return this.size;
    }
}
