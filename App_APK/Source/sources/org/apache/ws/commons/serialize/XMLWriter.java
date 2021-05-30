package org.apache.ws.commons.serialize;

import java.io.Writer;
import org.xml.sax.ContentHandler;

public interface XMLWriter extends ContentHandler {
    boolean canEncode(char c);

    String getEncoding();

    String getIndentString();

    String getLineFeed();

    Writer getWriter();

    boolean isDeclarating();

    boolean isFlushing();

    boolean isIndenting();

    void setDeclarating(boolean z);

    void setEncoding(String str);

    void setFlushing(boolean z);

    void setIndentString(String str);

    void setIndenting(boolean z);

    void setLineFeed(String str);

    void setWriter(Writer writer);
}
