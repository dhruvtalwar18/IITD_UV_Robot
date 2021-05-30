package org.jboss.netty.handler.codec.http.multipart;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.Charset;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.handler.codec.http.HttpConstants;
import org.xbill.DNS.TTL;

public abstract class AbstractMemoryHttpData extends AbstractHttpData {
    private ChannelBuffer channelBuffer;
    private int chunkPosition;
    protected boolean isRenamed;

    public AbstractMemoryHttpData(String name, Charset charset, long size) {
        super(name, charset, size);
    }

    public void setContent(ChannelBuffer buffer) throws IOException {
        if (buffer != null) {
            long localsize = (long) buffer.readableBytes();
            if (this.definedSize <= 0 || this.definedSize >= localsize) {
                this.channelBuffer = buffer;
                this.size = localsize;
                this.completed = true;
                return;
            }
            throw new IOException("Out of size: " + localsize + " > " + this.definedSize);
        }
        throw new NullPointerException("buffer");
    }

    public void setContent(InputStream inputStream) throws IOException {
        if (inputStream != null) {
            ChannelBuffer buffer = ChannelBuffers.dynamicBuffer();
            byte[] bytes = new byte[16384];
            int read = inputStream.read(bytes);
            int written = 0;
            while (read > 0) {
                buffer.writeBytes(bytes);
                written += read;
                read = inputStream.read(bytes);
            }
            this.size = (long) written;
            if (this.definedSize <= 0 || this.definedSize >= this.size) {
                this.channelBuffer = buffer;
                this.completed = true;
                return;
            }
            throw new IOException("Out of size: " + this.size + " > " + this.definedSize);
        }
        throw new NullPointerException("inputStream");
    }

    public void addContent(ChannelBuffer buffer, boolean last) throws IOException {
        if (buffer != null) {
            long localsize = (long) buffer.readableBytes();
            if (this.definedSize <= 0 || this.definedSize >= this.size + localsize) {
                this.size += localsize;
                if (this.channelBuffer == null) {
                    this.channelBuffer = buffer;
                } else {
                    this.channelBuffer = ChannelBuffers.wrappedBuffer(this.channelBuffer, buffer);
                }
            } else {
                throw new IOException("Out of size: " + (this.size + localsize) + " > " + this.definedSize);
            }
        }
        if (last) {
            this.completed = true;
        } else if (buffer == null) {
            throw new NullPointerException("buffer");
        }
    }

    public void setContent(File file) throws IOException {
        if (file != null) {
            long newsize = file.length();
            if (newsize <= TTL.MAX_VALUE) {
                FileChannel fileChannel = new FileInputStream(file).getChannel();
                ByteBuffer byteBuffer = ByteBuffer.wrap(new byte[((int) newsize)]);
                for (int read = 0; ((long) read) < newsize; read += fileChannel.read(byteBuffer)) {
                }
                fileChannel.close();
                byteBuffer.flip();
                this.channelBuffer = ChannelBuffers.wrappedBuffer(byteBuffer);
                this.size = newsize;
                this.completed = true;
                return;
            }
            throw new IllegalArgumentException("File too big to be loaded in memory");
        }
        throw new NullPointerException(HttpPostBodyUtil.FILE);
    }

    public void delete() {
    }

    public byte[] get() {
        if (this.channelBuffer == null) {
            return new byte[0];
        }
        byte[] array = new byte[this.channelBuffer.readableBytes()];
        this.channelBuffer.getBytes(this.channelBuffer.readerIndex(), array);
        return array;
    }

    public String getString() {
        return getString(HttpConstants.DEFAULT_CHARSET);
    }

    public String getString(Charset encoding) {
        if (this.channelBuffer == null) {
            return "";
        }
        if (encoding == null) {
            return getString(HttpConstants.DEFAULT_CHARSET);
        }
        return this.channelBuffer.toString(encoding);
    }

    public ChannelBuffer getChannelBuffer() {
        return this.channelBuffer;
    }

    public ChannelBuffer getChunk(int length) throws IOException {
        if (this.channelBuffer == null || length == 0 || this.channelBuffer.readableBytes() == 0) {
            this.chunkPosition = 0;
            return ChannelBuffers.EMPTY_BUFFER;
        }
        int sizeLeft = this.channelBuffer.readableBytes() - this.chunkPosition;
        if (sizeLeft == 0) {
            this.chunkPosition = 0;
            return ChannelBuffers.EMPTY_BUFFER;
        }
        int sliceLength = length;
        if (sizeLeft < length) {
            sliceLength = sizeLeft;
        }
        ChannelBuffer chunk = this.channelBuffer.slice(this.chunkPosition, sliceLength);
        this.chunkPosition += sliceLength;
        return chunk;
    }

    public boolean isInMemory() {
        return true;
    }

    public boolean renameTo(File dest) throws IOException {
        if (dest == null) {
            throw new NullPointerException("dest");
        } else if (this.channelBuffer == null) {
            dest.createNewFile();
            this.isRenamed = true;
            return true;
        } else {
            int length = this.channelBuffer.readableBytes();
            FileChannel fileChannel = new FileOutputStream(dest).getChannel();
            ByteBuffer byteBuffer = this.channelBuffer.toByteBuffer();
            int written = 0;
            while (written < length) {
                written += fileChannel.write(byteBuffer);
            }
            fileChannel.force(false);
            fileChannel.close();
            this.isRenamed = true;
            if (written == length) {
                return true;
            }
            return false;
        }
    }

    public File getFile() throws IOException {
        throw new IOException("Not represented by a file");
    }
}
