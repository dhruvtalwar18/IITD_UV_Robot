package org.jboss.netty.handler.codec.http.multipart;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.Charset;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.handler.codec.http.multipart.InterfaceHttpData;

public class MixedFileUpload implements FileUpload {
    private final long definedSize;
    private FileUpload fileUpload;
    private final long limitSize;

    public MixedFileUpload(String name, String filename, String contentType, String contentTransferEncoding, Charset charset, long size, long limitSize2) {
        long j = size;
        this.limitSize = limitSize2;
        if (j > this.limitSize) {
            this.fileUpload = new DiskFileUpload(name, filename, contentType, contentTransferEncoding, charset, size);
        } else {
            this.fileUpload = new MemoryFileUpload(name, filename, contentType, contentTransferEncoding, charset, size);
        }
        this.definedSize = j;
    }

    public void addContent(ChannelBuffer buffer, boolean last) throws IOException {
        if ((this.fileUpload instanceof MemoryFileUpload) && this.fileUpload.length() + ((long) buffer.readableBytes()) > this.limitSize) {
            DiskFileUpload diskFileUpload = new DiskFileUpload(this.fileUpload.getName(), this.fileUpload.getFilename(), this.fileUpload.getContentType(), this.fileUpload.getContentTransferEncoding(), this.fileUpload.getCharset(), this.definedSize);
            if (((MemoryFileUpload) this.fileUpload).getChannelBuffer() != null) {
                diskFileUpload.addContent(((MemoryFileUpload) this.fileUpload).getChannelBuffer(), false);
            }
            this.fileUpload = diskFileUpload;
        }
        this.fileUpload.addContent(buffer, last);
    }

    public void delete() {
        this.fileUpload.delete();
    }

    public byte[] get() throws IOException {
        return this.fileUpload.get();
    }

    public ChannelBuffer getChannelBuffer() throws IOException {
        return this.fileUpload.getChannelBuffer();
    }

    public Charset getCharset() {
        return this.fileUpload.getCharset();
    }

    public String getContentType() {
        return this.fileUpload.getContentType();
    }

    public String getContentTransferEncoding() {
        return this.fileUpload.getContentTransferEncoding();
    }

    public String getFilename() {
        return this.fileUpload.getFilename();
    }

    public String getString() throws IOException {
        return this.fileUpload.getString();
    }

    public String getString(Charset encoding) throws IOException {
        return this.fileUpload.getString(encoding);
    }

    public boolean isCompleted() {
        return this.fileUpload.isCompleted();
    }

    public boolean isInMemory() {
        return this.fileUpload.isInMemory();
    }

    public long length() {
        return this.fileUpload.length();
    }

    public boolean renameTo(File dest) throws IOException {
        return this.fileUpload.renameTo(dest);
    }

    public void setCharset(Charset charset) {
        this.fileUpload.setCharset(charset);
    }

    public void setContent(ChannelBuffer buffer) throws IOException {
        if (((long) buffer.readableBytes()) > this.limitSize && (this.fileUpload instanceof MemoryFileUpload)) {
            this.fileUpload = new DiskFileUpload(this.fileUpload.getName(), this.fileUpload.getFilename(), this.fileUpload.getContentType(), this.fileUpload.getContentTransferEncoding(), this.fileUpload.getCharset(), this.definedSize);
        }
        this.fileUpload.setContent(buffer);
    }

    public void setContent(File file) throws IOException {
        if (file.length() > this.limitSize && (this.fileUpload instanceof MemoryFileUpload)) {
            this.fileUpload = new DiskFileUpload(this.fileUpload.getName(), this.fileUpload.getFilename(), this.fileUpload.getContentType(), this.fileUpload.getContentTransferEncoding(), this.fileUpload.getCharset(), this.definedSize);
        }
        this.fileUpload.setContent(file);
    }

    public void setContent(InputStream inputStream) throws IOException {
        if (this.fileUpload instanceof MemoryFileUpload) {
            this.fileUpload = new DiskFileUpload(this.fileUpload.getName(), this.fileUpload.getFilename(), this.fileUpload.getContentType(), this.fileUpload.getContentTransferEncoding(), this.fileUpload.getCharset(), this.definedSize);
        }
        this.fileUpload.setContent(inputStream);
    }

    public void setContentType(String contentType) {
        this.fileUpload.setContentType(contentType);
    }

    public void setContentTransferEncoding(String contentTransferEncoding) {
        this.fileUpload.setContentTransferEncoding(contentTransferEncoding);
    }

    public void setFilename(String filename) {
        this.fileUpload.setFilename(filename);
    }

    public InterfaceHttpData.HttpDataType getHttpDataType() {
        return this.fileUpload.getHttpDataType();
    }

    public String getName() {
        return this.fileUpload.getName();
    }

    public int compareTo(InterfaceHttpData o) {
        return this.fileUpload.compareTo(o);
    }

    public String toString() {
        return "Mixed: " + this.fileUpload.toString();
    }

    public ChannelBuffer getChunk(int length) throws IOException {
        return this.fileUpload.getChunk(length);
    }

    public File getFile() throws IOException {
        return this.fileUpload.getFile();
    }
}
