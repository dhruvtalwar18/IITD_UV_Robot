package org.bytedeco.javacpp.tools;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;

public class EncodingFileWriter extends OutputStreamWriter {
    String newline = "\n";

    public EncodingFileWriter(File file, String newline2) throws IOException {
        super(new FileOutputStream(file));
        if (newline2 != null) {
            this.newline = newline2;
        }
    }

    public EncodingFileWriter(File file, String encoding, String newline2) throws IOException {
        super(new FileOutputStream(file), encoding);
        if (newline2 != null) {
            this.newline = newline2;
        }
    }

    public Writer append(CharSequence text) throws IOException {
        return super.append(((String) text).replace("\n", this.newline).replace("\\u", "\\u005Cu"));
    }
}
