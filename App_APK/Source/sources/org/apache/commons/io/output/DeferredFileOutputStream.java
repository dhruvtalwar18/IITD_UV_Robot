package org.apache.commons.io.output;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import org.apache.commons.io.IOUtils;

public class DeferredFileOutputStream extends ThresholdingOutputStream {
    private boolean closed;
    private OutputStream currentOutputStream;
    private File directory;
    private ByteArrayOutputStream memoryOutputStream;
    private File outputFile;
    private String prefix;
    private String suffix;

    public DeferredFileOutputStream(int threshold, File outputFile2) {
        super(threshold);
        this.closed = false;
        this.outputFile = outputFile2;
        this.memoryOutputStream = new ByteArrayOutputStream();
        this.currentOutputStream = this.memoryOutputStream;
    }

    public DeferredFileOutputStream(int threshold, String prefix2, String suffix2, File directory2) {
        this(threshold, (File) null);
        if (prefix2 != null) {
            this.prefix = prefix2;
            this.suffix = suffix2;
            this.directory = directory2;
            return;
        }
        throw new IllegalArgumentException("Temporary file prefix is missing");
    }

    /* access modifiers changed from: protected */
    public OutputStream getStream() throws IOException {
        return this.currentOutputStream;
    }

    /* access modifiers changed from: protected */
    public void thresholdReached() throws IOException {
        if (this.prefix != null) {
            this.outputFile = File.createTempFile(this.prefix, this.suffix, this.directory);
        }
        FileOutputStream fos = new FileOutputStream(this.outputFile);
        this.memoryOutputStream.writeTo(fos);
        this.currentOutputStream = fos;
        this.memoryOutputStream = null;
    }

    public boolean isInMemory() {
        return !isThresholdExceeded();
    }

    public byte[] getData() {
        if (this.memoryOutputStream != null) {
            return this.memoryOutputStream.toByteArray();
        }
        return null;
    }

    public File getFile() {
        return this.outputFile;
    }

    public void close() throws IOException {
        super.close();
        this.closed = true;
    }

    public void writeTo(OutputStream out) throws IOException {
        if (!this.closed) {
            throw new IOException("Stream not closed");
        } else if (isInMemory()) {
            this.memoryOutputStream.writeTo(out);
        } else {
            FileInputStream fis = new FileInputStream(this.outputFile);
            try {
                IOUtils.copy((InputStream) fis, out);
            } finally {
                IOUtils.closeQuietly((InputStream) fis);
            }
        }
    }
}
