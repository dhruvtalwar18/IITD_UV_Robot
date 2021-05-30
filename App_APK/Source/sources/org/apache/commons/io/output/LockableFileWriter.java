package org.apache.commons.io.output;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.IOUtils;

public class LockableFileWriter extends Writer {
    private static final String LCK = ".lck";
    static /* synthetic */ Class class$org$apache$commons$io$output$LockableFileWriter;
    private final File lockFile;
    private final Writer out;

    public LockableFileWriter(String fileName) throws IOException {
        this(fileName, false, (String) null);
    }

    public LockableFileWriter(String fileName, boolean append) throws IOException {
        this(fileName, append, (String) null);
    }

    public LockableFileWriter(String fileName, boolean append, String lockDir) throws IOException {
        this(new File(fileName), append, lockDir);
    }

    public LockableFileWriter(File file) throws IOException {
        this(file, false, (String) null);
    }

    public LockableFileWriter(File file, boolean append) throws IOException {
        this(file, append, (String) null);
    }

    public LockableFileWriter(File file, boolean append, String lockDir) throws IOException {
        this(file, (String) null, append, lockDir);
    }

    public LockableFileWriter(File file, String encoding) throws IOException {
        this(file, encoding, false, (String) null);
    }

    public LockableFileWriter(File file, String encoding, boolean append, String lockDir) throws IOException {
        File file2 = file.getAbsoluteFile();
        if (file2.getParentFile() != null) {
            FileUtils.forceMkdir(file2.getParentFile());
        }
        if (!file2.isDirectory()) {
            File lockDirFile = new File(lockDir == null ? System.getProperty("java.io.tmpdir") : lockDir);
            FileUtils.forceMkdir(lockDirFile);
            testLockDir(lockDirFile);
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(file2.getName());
            stringBuffer.append(LCK);
            this.lockFile = new File(lockDirFile, stringBuffer.toString());
            createLock();
            this.out = initWriter(file2, encoding, append);
            return;
        }
        throw new IOException("File specified is a directory");
    }

    private void testLockDir(File lockDir) throws IOException {
        if (!lockDir.exists()) {
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append("Could not find lockDir: ");
            stringBuffer.append(lockDir.getAbsolutePath());
            throw new IOException(stringBuffer.toString());
        } else if (!lockDir.canWrite()) {
            StringBuffer stringBuffer2 = new StringBuffer();
            stringBuffer2.append("Could not write to lockDir: ");
            stringBuffer2.append(lockDir.getAbsolutePath());
            throw new IOException(stringBuffer2.toString());
        }
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    private void createLock() throws IOException {
        Class cls;
        if (class$org$apache$commons$io$output$LockableFileWriter == null) {
            cls = class$("org.apache.commons.io.output.LockableFileWriter");
            class$org$apache$commons$io$output$LockableFileWriter = cls;
        } else {
            cls = class$org$apache$commons$io$output$LockableFileWriter;
        }
        synchronized (cls) {
            if (this.lockFile.createNewFile()) {
                this.lockFile.deleteOnExit();
            } else {
                StringBuffer stringBuffer = new StringBuffer();
                stringBuffer.append("Can't write file, lock ");
                stringBuffer.append(this.lockFile.getAbsolutePath());
                stringBuffer.append(" exists");
                throw new IOException(stringBuffer.toString());
            }
        }
    }

    private Writer initWriter(File file, String encoding, boolean append) throws IOException {
        boolean fileExistedAlready = file.exists();
        if (encoding != null) {
            return new OutputStreamWriter(new FileOutputStream(file.getAbsolutePath(), append), encoding);
        }
        try {
            return new FileWriter(file.getAbsolutePath(), append);
        } catch (IOException ex) {
            IOUtils.closeQuietly((Writer) null);
            IOUtils.closeQuietly((OutputStream) null);
            this.lockFile.delete();
            if (!fileExistedAlready) {
                file.delete();
            }
            throw ex;
        } catch (RuntimeException ex2) {
            IOUtils.closeQuietly((Writer) null);
            IOUtils.closeQuietly((OutputStream) null);
            this.lockFile.delete();
            if (!fileExistedAlready) {
                file.delete();
            }
            throw ex2;
        }
    }

    public void close() throws IOException {
        try {
            this.out.close();
        } finally {
            this.lockFile.delete();
        }
    }

    public void write(int idx) throws IOException {
        this.out.write(idx);
    }

    public void write(char[] chr) throws IOException {
        this.out.write(chr);
    }

    public void write(char[] chr, int st, int end) throws IOException {
        this.out.write(chr, st, end);
    }

    public void write(String str) throws IOException {
        this.out.write(str);
    }

    public void write(String str, int st, int end) throws IOException {
        this.out.write(str, st, end);
    }

    public void flush() throws IOException {
        this.out.flush();
    }
}
