package org.jboss.netty.handler.codec.http.multipart;

import java.io.File;
import java.nio.charset.Charset;
import org.jboss.netty.handler.codec.http.multipart.InterfaceHttpData;

public class DiskFileUpload extends AbstractDiskHttpData implements FileUpload {
    public static String baseDirectory;
    public static boolean deleteOnExitTemporaryFile = true;
    public static String postfix = ".tmp";
    public static String prefix = "FUp_";
    private String contentTransferEncoding;
    private String contentType;
    private String filename;

    public DiskFileUpload(String name, String filename2, String contentType2, String contentTransferEncoding2, Charset charset, long size) {
        super(name, charset, size);
        setFilename(filename2);
        setContentType(contentType2);
        setContentTransferEncoding(contentTransferEncoding2);
    }

    public InterfaceHttpData.HttpDataType getHttpDataType() {
        return InterfaceHttpData.HttpDataType.FileUpload;
    }

    public String getFilename() {
        return this.filename;
    }

    public void setFilename(String filename2) {
        if (filename2 != null) {
            this.filename = filename2;
            return;
        }
        throw new NullPointerException(HttpPostBodyUtil.FILENAME);
    }

    public int hashCode() {
        return getName().hashCode();
    }

    public boolean equals(Object o) {
        if (!(o instanceof Attribute)) {
            return false;
        }
        return getName().equalsIgnoreCase(((Attribute) o).getName());
    }

    public int compareTo(InterfaceHttpData arg0) {
        if (arg0 instanceof FileUpload) {
            return compareTo((FileUpload) arg0);
        }
        throw new ClassCastException("Cannot compare " + getHttpDataType() + " with " + arg0.getHttpDataType());
    }

    public int compareTo(FileUpload o) {
        int v = getName().compareToIgnoreCase(o.getName());
        if (v != 0) {
            return v;
        }
        return v;
    }

    public void setContentType(String contentType2) {
        if (contentType2 != null) {
            this.contentType = contentType2;
            return;
        }
        throw new NullPointerException("contentType");
    }

    public String getContentType() {
        return this.contentType;
    }

    public String getContentTransferEncoding() {
        return this.contentTransferEncoding;
    }

    public void setContentTransferEncoding(String contentTransferEncoding2) {
        this.contentTransferEncoding = contentTransferEncoding2;
    }

    public String toString() {
        String str;
        StringBuilder sb = new StringBuilder();
        sb.append("Content-Disposition: form-data; name=\"");
        sb.append(getName());
        sb.append("\"; ");
        sb.append(HttpPostBodyUtil.FILENAME);
        sb.append("=\"");
        sb.append(this.filename);
        sb.append("\"\r\n");
        sb.append("Content-Type");
        sb.append(": ");
        sb.append(this.contentType);
        if (this.charset != null) {
            str = "; charset=" + this.charset + "\r\n";
        } else {
            str = "\r\n";
        }
        sb.append(str);
        sb.append("Content-Length");
        sb.append(": ");
        sb.append(length());
        sb.append("\r\n");
        sb.append("Completed: ");
        sb.append(isCompleted());
        sb.append("\r\nIsInMemory: ");
        sb.append(isInMemory());
        sb.append("\r\nRealFile: ");
        sb.append(this.file.getAbsolutePath());
        sb.append(" DefaultDeleteAfter: ");
        sb.append(deleteOnExitTemporaryFile);
        return sb.toString();
    }

    /* access modifiers changed from: protected */
    public boolean deleteOnExit() {
        return deleteOnExitTemporaryFile;
    }

    /* access modifiers changed from: protected */
    public String getBaseDirectory() {
        return baseDirectory;
    }

    /* access modifiers changed from: protected */
    public String getDiskFilename() {
        return new File(this.filename).getName();
    }

    /* access modifiers changed from: protected */
    public String getPostfix() {
        return postfix;
    }

    /* access modifiers changed from: protected */
    public String getPrefix() {
        return prefix;
    }
}
