package org.apache.commons.net.ftp;

import java.io.BufferedReader;
import java.io.IOException;
import java.util.Iterator;
import java.util.List;

public abstract class FTPFileEntryParserImpl implements FTPFileEntryParser {
    public String readNextEntry(BufferedReader reader) throws IOException {
        return reader.readLine();
    }

    public List<String> preParse(List<String> original) {
        Iterator<String> it = original.iterator();
        while (it.hasNext()) {
            if (parseFTPEntry(it.next()) == null) {
                it.remove();
            }
        }
        return original;
    }
}
