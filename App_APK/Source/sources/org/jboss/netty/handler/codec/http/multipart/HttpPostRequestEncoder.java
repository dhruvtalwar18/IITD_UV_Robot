package org.jboss.netty.handler.codec.http.multipart;

import java.io.File;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.URLEncoder;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;
import java.util.ListIterator;
import java.util.Random;
import org.apache.xmlrpc.serializer.ObjectArraySerializer;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.handler.codec.http.DefaultHttpChunk;
import org.jboss.netty.handler.codec.http.HttpChunk;
import org.jboss.netty.handler.codec.http.HttpConstants;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.handler.codec.http.HttpMethod;
import org.jboss.netty.handler.codec.http.HttpRequest;
import org.jboss.netty.handler.codec.http.multipart.HttpPostBodyUtil;
import org.jboss.netty.handler.stream.ChunkedInput;

public class HttpPostRequestEncoder implements ChunkedInput {
    private final List<InterfaceHttpData> bodyListDatas;
    private final Charset charset;
    private ChannelBuffer currentBuffer;
    private InterfaceHttpData currentData;
    private FileUpload currentFileUpload;
    private boolean duringMixedMode;
    private final HttpDataFactory factory;
    private long globalBodySize;
    private boolean headerFinalized;
    private boolean isChunked;
    private boolean isKey;
    private boolean isLastChunk;
    private boolean isLastChunkSent;
    private final boolean isMultipart;
    private ListIterator<InterfaceHttpData> iterator;
    private String multipartDataBoundary;
    private final List<InterfaceHttpData> multipartHttpDatas;
    private String multipartMixedBoundary;
    private final HttpRequest request;

    public HttpPostRequestEncoder(HttpRequest request2, boolean multipart) throws ErrorDataEncoderException {
        this(new DefaultHttpDataFactory(DefaultHttpDataFactory.MINSIZE), request2, multipart, HttpConstants.DEFAULT_CHARSET);
    }

    public HttpPostRequestEncoder(HttpDataFactory factory2, HttpRequest request2, boolean multipart) throws ErrorDataEncoderException {
        this(factory2, request2, multipart, HttpConstants.DEFAULT_CHARSET);
    }

    public HttpPostRequestEncoder(HttpDataFactory factory2, HttpRequest request2, boolean multipart, Charset charset2) throws ErrorDataEncoderException {
        this.isKey = true;
        if (factory2 == null) {
            throw new NullPointerException("factory");
        } else if (request2 == null) {
            throw new NullPointerException("request");
        } else if (charset2 == null) {
            throw new NullPointerException("charset");
        } else if (request2.getMethod() == HttpMethod.POST) {
            this.request = request2;
            this.charset = charset2;
            this.factory = factory2;
            this.bodyListDatas = new ArrayList();
            this.isLastChunk = false;
            this.isLastChunkSent = false;
            this.isMultipart = multipart;
            this.multipartHttpDatas = new ArrayList();
            if (this.isMultipart) {
                initDataMultipart();
            }
        } else {
            throw new ErrorDataEncoderException("Cannot create a Encoder if not a POST");
        }
    }

    public void cleanFiles() {
        this.factory.cleanRequestHttpDatas(this.request);
    }

    public boolean isMultipart() {
        return this.isMultipart;
    }

    private void initDataMultipart() {
        this.multipartDataBoundary = getNewMultipartDelimiter();
    }

    private void initMixedMultipart() {
        this.multipartMixedBoundary = getNewMultipartDelimiter();
    }

    private static String getNewMultipartDelimiter() {
        return Long.toHexString(new Random().nextLong()).toLowerCase();
    }

    public List<InterfaceHttpData> getBodyListAttributes() {
        return this.bodyListDatas;
    }

    public void setBodyHttpDatas(List<InterfaceHttpData> datas) throws ErrorDataEncoderException {
        if (datas != null) {
            this.globalBodySize = 0;
            this.bodyListDatas.clear();
            this.currentFileUpload = null;
            this.duringMixedMode = false;
            this.multipartHttpDatas.clear();
            for (InterfaceHttpData data : datas) {
                addBodyHttpData(data);
            }
            return;
        }
        throw new NullPointerException("datas");
    }

    public void addBodyAttribute(String name, String value) throws ErrorDataEncoderException {
        if (name != null) {
            String svalue = value;
            if (value == null) {
                svalue = "";
            }
            addBodyHttpData(this.factory.createAttribute(this.request, name, svalue));
            return;
        }
        throw new NullPointerException("name");
    }

    public void addBodyFileUpload(String name, File file, String contentType, boolean isText) throws ErrorDataEncoderException {
        if (name == null) {
            throw new NullPointerException("name");
        } else if (file != null) {
            String scontentType = contentType;
            String contentTransferEncoding = null;
            if (contentType == null) {
                if (isText) {
                    scontentType = "text/plain";
                } else {
                    scontentType = "application/octet-stream";
                }
            }
            if (!isText) {
                contentTransferEncoding = HttpPostBodyUtil.TransferEncodingMechanism.BINARY.value;
            }
            FileUpload fileUpload = this.factory.createFileUpload(this.request, name, file.getName(), scontentType, contentTransferEncoding, (Charset) null, file.length());
            try {
                fileUpload.setContent(file);
                addBodyHttpData(fileUpload);
            } catch (IOException e) {
                throw new ErrorDataEncoderException((Throwable) e);
            }
        } else {
            throw new NullPointerException(HttpPostBodyUtil.FILE);
        }
    }

    public void addBodyFileUploads(String name, File[] file, String[] contentType, boolean[] isText) throws ErrorDataEncoderException {
        if (file.length == contentType.length || file.length == isText.length) {
            for (int i = 0; i < file.length; i++) {
                addBodyFileUpload(name, file[i], contentType[i], isText[i]);
            }
            return;
        }
        throw new NullPointerException("Different array length");
    }

    public void addBodyHttpData(InterfaceHttpData data) throws ErrorDataEncoderException {
        boolean localMixed;
        if (this.headerFinalized) {
            throw new ErrorDataEncoderException("Cannot add value once finalized");
        } else if (data != null) {
            this.bodyListDatas.add(data);
            if (!this.isMultipart) {
                if (data instanceof Attribute) {
                    Attribute attribute = (Attribute) data;
                    try {
                        Attribute newattribute = this.factory.createAttribute(this.request, encodeAttribute(attribute.getName(), this.charset), encodeAttribute(attribute.getValue(), this.charset));
                        this.multipartHttpDatas.add(newattribute);
                        this.globalBodySize += ((long) (newattribute.getName().length() + 1)) + newattribute.length() + 1;
                    } catch (IOException e) {
                        throw new ErrorDataEncoderException((Throwable) e);
                    }
                } else if (data instanceof FileUpload) {
                    FileUpload fileUpload = (FileUpload) data;
                    Attribute newattribute2 = this.factory.createAttribute(this.request, encodeAttribute(fileUpload.getName(), this.charset), encodeAttribute(fileUpload.getFilename(), this.charset));
                    this.multipartHttpDatas.add(newattribute2);
                    this.globalBodySize += ((long) (newattribute2.getName().length() + 1)) + newattribute2.length() + 1;
                }
            } else if (data instanceof Attribute) {
                if (this.duringMixedMode) {
                    InternalAttribute internal = new InternalAttribute();
                    internal.addValue("\r\n--" + this.multipartMixedBoundary + "--");
                    this.multipartHttpDatas.add(internal);
                    this.multipartMixedBoundary = null;
                    this.currentFileUpload = null;
                    this.duringMixedMode = false;
                }
                InternalAttribute internal2 = new InternalAttribute();
                if (this.multipartHttpDatas.size() > 0) {
                    internal2.addValue("\r\n");
                }
                internal2.addValue("--" + this.multipartDataBoundary + "\r\n");
                Attribute attribute2 = (Attribute) data;
                internal2.addValue("Content-Disposition: form-data; name=\"" + encodeAttribute(attribute2.getName(), this.charset) + "\"\r\n");
                Charset localcharset = attribute2.getCharset();
                if (localcharset != null) {
                    internal2.addValue("Content-Type: charset=" + localcharset + "\r\n");
                }
                internal2.addValue("\r\n");
                this.multipartHttpDatas.add(internal2);
                this.multipartHttpDatas.add(data);
                this.globalBodySize += attribute2.length() + ((long) internal2.size());
            } else if (data instanceof FileUpload) {
                FileUpload fileUpload2 = (FileUpload) data;
                InternalAttribute internal3 = new InternalAttribute();
                if (this.multipartHttpDatas.size() > 0) {
                    internal3.addValue("\r\n");
                }
                if (this.duringMixedMode) {
                    if (this.currentFileUpload == null || !this.currentFileUpload.getName().equals(fileUpload2.getName())) {
                        internal3.addValue("--" + this.multipartMixedBoundary + "--");
                        this.multipartHttpDatas.add(internal3);
                        this.multipartMixedBoundary = null;
                        internal3 = new InternalAttribute();
                        internal3.addValue("\r\n");
                        localMixed = false;
                        this.currentFileUpload = fileUpload2;
                        this.duringMixedMode = false;
                    } else {
                        localMixed = true;
                    }
                } else if (this.currentFileUpload == null || !this.currentFileUpload.getName().equals(fileUpload2.getName())) {
                    localMixed = false;
                    this.currentFileUpload = fileUpload2;
                    this.duringMixedMode = false;
                } else {
                    initMixedMultipart();
                    InternalAttribute pastAttribute = (InternalAttribute) this.multipartHttpDatas.get(this.multipartHttpDatas.size() - 2);
                    this.globalBodySize -= (long) pastAttribute.size();
                    pastAttribute.setValue(((("Content-Disposition: form-data; name=\"" + encodeAttribute(fileUpload2.getName(), this.charset) + "\"\r\n") + "Content-Type: multipart/mixed; boundary=" + this.multipartMixedBoundary + "\r\n\r\n") + "--" + this.multipartMixedBoundary + "\r\n") + "Content-Disposition: file; filename=\"" + encodeAttribute(fileUpload2.getFilename(), this.charset) + "\"\r\n", 1);
                    this.globalBodySize = this.globalBodySize + ((long) pastAttribute.size());
                    this.duringMixedMode = true;
                    localMixed = true;
                }
                if (localMixed) {
                    internal3.addValue("--" + this.multipartMixedBoundary + "\r\n");
                    internal3.addValue("Content-Disposition: file; filename=\"" + encodeAttribute(fileUpload2.getFilename(), this.charset) + "\"\r\n");
                } else {
                    internal3.addValue("--" + this.multipartDataBoundary + "\r\n");
                    internal3.addValue("Content-Disposition: form-data; name=\"" + encodeAttribute(fileUpload2.getName(), this.charset) + "\"; " + HttpPostBodyUtil.FILENAME + "=\"" + encodeAttribute(fileUpload2.getFilename(), this.charset) + "\"\r\n");
                }
                internal3.addValue("Content-Type: " + fileUpload2.getContentType());
                String contentTransferEncoding = fileUpload2.getContentTransferEncoding();
                if (contentTransferEncoding != null && contentTransferEncoding.equals(HttpPostBodyUtil.TransferEncodingMechanism.BINARY.value)) {
                    internal3.addValue("\r\nContent-Transfer-Encoding: " + HttpPostBodyUtil.TransferEncodingMechanism.BINARY.value + "\r\n\r\n");
                } else if (fileUpload2.getCharset() != null) {
                    internal3.addValue("; charset=" + fileUpload2.getCharset() + "\r\n\r\n");
                } else {
                    internal3.addValue("\r\n\r\n");
                }
                this.multipartHttpDatas.add(internal3);
                this.multipartHttpDatas.add(data);
                this.globalBodySize += fileUpload2.length() + ((long) internal3.size());
            }
        } else {
            throw new NullPointerException(ObjectArraySerializer.DATA_TAG);
        }
    }

    public HttpRequest finalizeRequest() throws ErrorDataEncoderException {
        if (!this.headerFinalized) {
            if (this.isMultipart) {
                InternalAttribute internal = new InternalAttribute();
                if (this.duringMixedMode) {
                    internal.addValue("\r\n--" + this.multipartMixedBoundary + "--");
                }
                internal.addValue("\r\n--" + this.multipartDataBoundary + "--\r\n");
                this.multipartHttpDatas.add(internal);
                this.multipartMixedBoundary = null;
                this.currentFileUpload = null;
                this.duringMixedMode = false;
                this.globalBodySize += (long) internal.size();
            }
            this.headerFinalized = true;
            List<String> contentTypes = this.request.getHeaders("Content-Type");
            List<String> transferEncoding = this.request.getHeaders("Transfer-Encoding");
            if (contentTypes != null) {
                this.request.removeHeader("Content-Type");
                for (String contentType : contentTypes) {
                    if (!contentType.toLowerCase().startsWith("multipart/form-data") && !contentType.toLowerCase().startsWith("application/x-www-form-urlencoded")) {
                        this.request.addHeader("Content-Type", contentType);
                    }
                }
            }
            if (this.isMultipart) {
                this.request.addHeader("Content-Type", "multipart/form-data; boundary=" + this.multipartDataBoundary);
            } else {
                this.request.addHeader("Content-Type", "application/x-www-form-urlencoded");
            }
            long realSize = this.globalBodySize;
            if (this.isMultipart) {
                this.iterator = this.multipartHttpDatas.listIterator();
            } else {
                realSize--;
                this.iterator = this.multipartHttpDatas.listIterator();
            }
            this.request.setHeader("Content-Length", (Object) String.valueOf(realSize));
            if (realSize > ((long) HttpPostBodyUtil.chunkSize)) {
                this.isChunked = true;
                if (transferEncoding != null) {
                    this.request.removeHeader("Transfer-Encoding");
                    for (String v : transferEncoding) {
                        if (!v.equalsIgnoreCase(HttpHeaders.Values.CHUNKED)) {
                            this.request.addHeader("Transfer-Encoding", v);
                        }
                    }
                }
                this.request.addHeader("Transfer-Encoding", HttpHeaders.Values.CHUNKED);
                this.request.setContent(ChannelBuffers.EMPTY_BUFFER);
            } else {
                this.request.setContent(nextChunk().getContent());
            }
            return this.request;
        }
        throw new ErrorDataEncoderException("Header already encoded");
    }

    public boolean isChunked() {
        return this.isChunked;
    }

    private static String encodeAttribute(String s, Charset charset2) throws ErrorDataEncoderException {
        if (s == null) {
            return "";
        }
        try {
            return URLEncoder.encode(s, charset2.name());
        } catch (UnsupportedEncodingException e) {
            throw new ErrorDataEncoderException(charset2.name(), e);
        }
    }

    private ChannelBuffer fillChannelBuffer() {
        if (this.currentBuffer.readableBytes() > HttpPostBodyUtil.chunkSize) {
            ChannelBuffer slice = this.currentBuffer.slice(this.currentBuffer.readerIndex(), HttpPostBodyUtil.chunkSize);
            this.currentBuffer.skipBytes(HttpPostBodyUtil.chunkSize);
            return slice;
        }
        ChannelBuffer slice2 = this.currentBuffer;
        this.currentBuffer = null;
        return slice2;
    }

    private HttpChunk encodeNextChunkMultipart(int sizeleft) throws ErrorDataEncoderException {
        ChannelBuffer buffer;
        ChannelBuffer buffer2;
        if (this.currentData == null) {
            return null;
        }
        if (this.currentData instanceof InternalAttribute) {
            try {
                buffer = ChannelBuffers.wrappedBuffer(((InternalAttribute) this.currentData).toString().getBytes(NTLM.DEFAULT_CHARSET));
                this.currentData = null;
            } catch (UnsupportedEncodingException e) {
                throw new ErrorDataEncoderException((Throwable) e);
            }
        } else {
            if (this.currentData instanceof Attribute) {
                try {
                    buffer2 = ((Attribute) this.currentData).getChunk(sizeleft);
                } catch (IOException e2) {
                    throw new ErrorDataEncoderException((Throwable) e2);
                }
            } else {
                try {
                    buffer2 = ((FileUpload) this.currentData).getChunk(sizeleft);
                } catch (IOException e3) {
                    throw new ErrorDataEncoderException((Throwable) e3);
                }
            }
            buffer = buffer2;
            if (buffer.capacity() == 0) {
                this.currentData = null;
                return null;
            }
        }
        if (this.currentBuffer == null) {
            this.currentBuffer = buffer;
        } else {
            this.currentBuffer = ChannelBuffers.wrappedBuffer(this.currentBuffer, buffer);
        }
        if (this.currentBuffer.readableBytes() >= HttpPostBodyUtil.chunkSize) {
            return new DefaultHttpChunk(fillChannelBuffer());
        }
        this.currentData = null;
        return null;
    }

    private HttpChunk encodeNextChunkUrlEncoded(int sizeleft) throws ErrorDataEncoderException {
        if (this.currentData == null) {
            return null;
        }
        int size = sizeleft;
        if (this.isKey) {
            ChannelBuffer buffer = ChannelBuffers.wrappedBuffer(this.currentData.getName().getBytes());
            this.isKey = false;
            if (this.currentBuffer == null) {
                this.currentBuffer = ChannelBuffers.wrappedBuffer(buffer, ChannelBuffers.wrappedBuffer("=".getBytes()));
                size -= buffer.readableBytes() + 1;
            } else {
                this.currentBuffer = ChannelBuffers.wrappedBuffer(this.currentBuffer, buffer, ChannelBuffers.wrappedBuffer("=".getBytes()));
                size -= buffer.readableBytes() + 1;
            }
            if (this.currentBuffer.readableBytes() >= HttpPostBodyUtil.chunkSize) {
                return new DefaultHttpChunk(fillChannelBuffer());
            }
        }
        try {
            ChannelBuffer buffer2 = ((Attribute) this.currentData).getChunk(size);
            ChannelBuffer delimiter = null;
            if (buffer2.readableBytes() < size) {
                this.isKey = true;
                delimiter = this.iterator.hasNext() ? ChannelBuffers.wrappedBuffer("&".getBytes()) : null;
            }
            if (buffer2.capacity() == 0) {
                this.currentData = null;
                if (this.currentBuffer == null) {
                    this.currentBuffer = delimiter;
                } else if (delimiter != null) {
                    this.currentBuffer = ChannelBuffers.wrappedBuffer(this.currentBuffer, delimiter);
                }
                if (this.currentBuffer.readableBytes() >= HttpPostBodyUtil.chunkSize) {
                    return new DefaultHttpChunk(fillChannelBuffer());
                }
                return null;
            }
            if (this.currentBuffer == null) {
                if (delimiter != null) {
                    this.currentBuffer = ChannelBuffers.wrappedBuffer(buffer2, delimiter);
                } else {
                    this.currentBuffer = buffer2;
                }
            } else if (delimiter != null) {
                this.currentBuffer = ChannelBuffers.wrappedBuffer(this.currentBuffer, buffer2, delimiter);
            } else {
                this.currentBuffer = ChannelBuffers.wrappedBuffer(this.currentBuffer, buffer2);
            }
            if (this.currentBuffer.readableBytes() >= HttpPostBodyUtil.chunkSize) {
                return new DefaultHttpChunk(fillChannelBuffer());
            }
            this.currentData = null;
            this.isKey = true;
            return null;
        } catch (IOException e) {
            throw new ErrorDataEncoderException((Throwable) e);
        }
    }

    public void close() throws Exception {
    }

    public HttpChunk nextChunk() throws ErrorDataEncoderException {
        HttpChunk chunk;
        if (this.isLastChunk) {
            this.isLastChunkSent = true;
            return new DefaultHttpChunk(ChannelBuffers.EMPTY_BUFFER);
        }
        int size = HttpPostBodyUtil.chunkSize;
        if (this.currentBuffer != null) {
            size -= this.currentBuffer.readableBytes();
        }
        if (size <= 0) {
            return new DefaultHttpChunk(fillChannelBuffer());
        }
        if (this.currentData != null) {
            if (this.isMultipart) {
                HttpChunk chunk2 = encodeNextChunkMultipart(size);
                if (chunk2 != null) {
                    return chunk2;
                }
            } else {
                HttpChunk chunk3 = encodeNextChunkUrlEncoded(size);
                if (chunk3 != null) {
                    return chunk3;
                }
            }
            size = HttpPostBodyUtil.chunkSize - this.currentBuffer.readableBytes();
        }
        if (!this.iterator.hasNext()) {
            this.isLastChunk = true;
            ChannelBuffer buffer = this.currentBuffer;
            this.currentBuffer = null;
            return new DefaultHttpChunk(buffer);
        }
        while (size > 0 && this.iterator.hasNext()) {
            this.currentData = this.iterator.next();
            if (this.isMultipart) {
                chunk = encodeNextChunkMultipart(size);
            } else {
                chunk = encodeNextChunkUrlEncoded(size);
            }
            if (chunk != null) {
                return chunk;
            }
            size = HttpPostBodyUtil.chunkSize - this.currentBuffer.readableBytes();
        }
        this.isLastChunk = true;
        if (this.currentBuffer == null) {
            this.isLastChunkSent = true;
            return new DefaultHttpChunk(ChannelBuffers.EMPTY_BUFFER);
        }
        ChannelBuffer buffer2 = this.currentBuffer;
        this.currentBuffer = null;
        return new DefaultHttpChunk(buffer2);
    }

    public boolean isEndOfInput() throws Exception {
        return this.isLastChunkSent;
    }

    public boolean hasNextChunk() throws Exception {
        return !this.isLastChunkSent;
    }

    public static class ErrorDataEncoderException extends Exception {
        private static final long serialVersionUID = 5020247425493164465L;

        public ErrorDataEncoderException() {
        }

        public ErrorDataEncoderException(String arg0) {
            super(arg0);
        }

        public ErrorDataEncoderException(Throwable arg0) {
            super(arg0);
        }

        public ErrorDataEncoderException(String arg0, Throwable arg1) {
            super(arg0, arg1);
        }
    }
}
