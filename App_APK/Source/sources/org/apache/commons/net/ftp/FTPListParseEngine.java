package org.apache.commons.net.ftp;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

public class FTPListParseEngine {
    private ListIterator<String> _internalIterator = this.entries.listIterator();
    private List<String> entries = new LinkedList();
    FTPFileEntryParser parser = null;

    public FTPListParseEngine(FTPFileEntryParser parser2) {
        this.parser = parser2;
    }

    public void readServerList(InputStream stream, String encoding) throws IOException {
        this.entries = new LinkedList();
        readStream(stream, encoding);
        this.parser.preParse(this.entries);
        resetIterator();
    }

    public void readServerList(InputStream stream) throws IOException {
        readServerList(stream, (String) null);
    }

    private void readStream(InputStream stream, String encoding) throws IOException {
        BufferedReader reader;
        if (encoding == null) {
            reader = new BufferedReader(new InputStreamReader(stream));
        } else {
            reader = new BufferedReader(new InputStreamReader(stream, encoding));
        }
        String line = this.parser.readNextEntry(reader);
        while (line != null) {
            this.entries.add(line);
            line = this.parser.readNextEntry(reader);
        }
        reader.close();
    }

    public FTPFile[] getNext(int quantityRequested) {
        List<FTPFile> tmpResults = new LinkedList<>();
        for (int count = quantityRequested; count > 0 && this._internalIterator.hasNext(); count--) {
            tmpResults.add(this.parser.parseFTPEntry(this._internalIterator.next()));
        }
        return (FTPFile[]) tmpResults.toArray(new FTPFile[0]);
    }

    public FTPFile[] getPrevious(int quantityRequested) {
        List<FTPFile> tmpResults = new LinkedList<>();
        for (int count = quantityRequested; count > 0 && this._internalIterator.hasPrevious(); count--) {
            tmpResults.add(0, this.parser.parseFTPEntry(this._internalIterator.previous()));
        }
        return (FTPFile[]) tmpResults.toArray(new FTPFile[0]);
    }

    public FTPFile[] getFiles() throws IOException {
        List<FTPFile> tmpResults = new LinkedList<>();
        for (String entry : this.entries) {
            tmpResults.add(this.parser.parseFTPEntry(entry));
        }
        return (FTPFile[]) tmpResults.toArray(new FTPFile[0]);
    }

    public boolean hasNext() {
        return this._internalIterator.hasNext();
    }

    public boolean hasPrevious() {
        return this._internalIterator.hasPrevious();
    }

    public void resetIterator() {
        this._internalIterator = this.entries.listIterator();
    }
}
