package org.apache.commons.httpclient.methods;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public class FileRequestEntity implements RequestEntity {
    final String contentType;
    final File file;

    public FileRequestEntity(File file2, String contentType2) {
        if (file2 != null) {
            this.file = file2;
            this.contentType = contentType2;
            return;
        }
        throw new IllegalArgumentException("File may not be null");
    }

    public long getContentLength() {
        return this.file.length();
    }

    public String getContentType() {
        return this.contentType;
    }

    public boolean isRepeatable() {
        return true;
    }

    public void writeRequest(OutputStream out) throws IOException {
        byte[] tmp = new byte[4096];
        InputStream instream = new FileInputStream(this.file);
        while (true) {
            try {
                int read = instream.read(tmp);
                int i = read;
                if (read >= 0) {
                    out.write(tmp, 0, i);
                } else {
                    return;
                }
            } finally {
                instream.close();
            }
        }
    }
}
