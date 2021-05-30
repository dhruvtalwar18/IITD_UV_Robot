package org.jboss.netty.handler.codec.http.multipart;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.jboss.netty.handler.codec.http.HttpChunk;
import org.jboss.netty.handler.codec.http.HttpConstants;
import org.jboss.netty.handler.codec.http.HttpHeaders;
import org.jboss.netty.handler.codec.http.HttpMethod;
import org.jboss.netty.handler.codec.http.HttpRequest;
import org.jboss.netty.handler.codec.http.multipart.HttpPostBodyUtil;
import org.jboss.netty.util.internal.CaseIgnoringComparator;
import sensor_msgs.NavSatStatus;

public class HttpPostRequestDecoder {
    private final List<InterfaceHttpData> bodyListHttpData;
    private int bodyListHttpDataRank;
    private final Map<String, List<InterfaceHttpData>> bodyMapHttpData;
    private boolean bodyToDecode;
    private final Charset charset;
    private Attribute currentAttribute;
    private Map<String, Attribute> currentFieldAttributes;
    private FileUpload currentFileUpload;
    private MultiPartStatus currentStatus;
    private final HttpDataFactory factory;
    private boolean isLastChunk;
    private boolean isMultipart;
    private String multipartDataBoundary;
    private String multipartMixedBoundary;
    private final HttpRequest request;
    private ChannelBuffer undecodedChunk;

    public static class EndOfDataDecoderException extends Exception {
        private static final long serialVersionUID = 1336267941020800769L;
    }

    private enum MultiPartStatus {
        NOTSTARTED,
        PREAMBLE,
        HEADERDELIMITER,
        DISPOSITION,
        FIELD,
        FILEUPLOAD,
        MIXEDPREAMBLE,
        MIXEDDELIMITER,
        MIXEDDISPOSITION,
        MIXEDFILEUPLOAD,
        MIXEDCLOSEDELIMITER,
        CLOSEDELIMITER,
        PREEPILOGUE,
        EPILOGUE
    }

    public HttpPostRequestDecoder(HttpRequest request2) throws ErrorDataDecoderException, IncompatibleDataDecoderException {
        this(new DefaultHttpDataFactory(DefaultHttpDataFactory.MINSIZE), request2, HttpConstants.DEFAULT_CHARSET);
    }

    public HttpPostRequestDecoder(HttpDataFactory factory2, HttpRequest request2) throws ErrorDataDecoderException, IncompatibleDataDecoderException {
        this(factory2, request2, HttpConstants.DEFAULT_CHARSET);
    }

    public HttpPostRequestDecoder(HttpDataFactory factory2, HttpRequest request2, Charset charset2) throws ErrorDataDecoderException, IncompatibleDataDecoderException {
        this.bodyListHttpData = new ArrayList();
        this.bodyMapHttpData = new TreeMap(CaseIgnoringComparator.INSTANCE);
        this.currentStatus = MultiPartStatus.NOTSTARTED;
        if (factory2 == null) {
            throw new NullPointerException("factory");
        } else if (request2 == null) {
            throw new NullPointerException("request");
        } else if (charset2 != null) {
            this.request = request2;
            HttpMethod method = request2.getMethod();
            if (method.equals(HttpMethod.POST) || method.equals(HttpMethod.PUT) || method.equals(HttpMethod.PATCH)) {
                this.bodyToDecode = true;
            }
            this.charset = charset2;
            this.factory = factory2;
            if (this.request.containsHeader("Content-Type")) {
                checkMultipart(this.request.getHeader("Content-Type"));
            } else {
                this.isMultipart = false;
            }
            if (!this.bodyToDecode) {
                throw new IncompatibleDataDecoderException("No Body to decode");
            } else if (!this.request.isChunked()) {
                this.undecodedChunk = this.request.getContent();
                this.isLastChunk = true;
                parseBody();
            }
        } else {
            throw new NullPointerException("charset");
        }
    }

    private void checkMultipart(String contentType) throws ErrorDataDecoderException {
        String[] headerContentType = splitHeaderContentType(contentType);
        if (!headerContentType[0].toLowerCase().startsWith("multipart/form-data") || !headerContentType[1].toLowerCase().startsWith(HttpHeaders.Values.BOUNDARY)) {
            this.isMultipart = false;
            return;
        }
        String[] boundary = headerContentType[1].split("=");
        if (boundary.length == 2) {
            this.multipartDataBoundary = "--" + boundary[1];
            this.isMultipart = true;
            this.currentStatus = MultiPartStatus.HEADERDELIMITER;
            return;
        }
        throw new ErrorDataDecoderException("Needs a boundary value");
    }

    public boolean isMultipart() {
        return this.isMultipart;
    }

    public List<InterfaceHttpData> getBodyHttpDatas() throws NotEnoughDataDecoderException {
        if (this.isLastChunk) {
            return this.bodyListHttpData;
        }
        throw new NotEnoughDataDecoderException();
    }

    public List<InterfaceHttpData> getBodyHttpDatas(String name) throws NotEnoughDataDecoderException {
        if (this.isLastChunk) {
            return this.bodyMapHttpData.get(name);
        }
        throw new NotEnoughDataDecoderException();
    }

    public InterfaceHttpData getBodyHttpData(String name) throws NotEnoughDataDecoderException {
        if (this.isLastChunk) {
            List<InterfaceHttpData> list = this.bodyMapHttpData.get(name);
            if (list != null) {
                return list.get(0);
            }
            return null;
        }
        throw new NotEnoughDataDecoderException();
    }

    public void offer(HttpChunk chunk) throws ErrorDataDecoderException {
        ChannelBuffer chunked = chunk.getContent();
        if (this.undecodedChunk == null) {
            this.undecodedChunk = chunked;
        } else {
            this.undecodedChunk = ChannelBuffers.wrappedBuffer(this.undecodedChunk, chunked);
        }
        if (chunk.isLast()) {
            this.isLastChunk = true;
        }
        parseBody();
    }

    public boolean hasNext() throws EndOfDataDecoderException {
        if (this.currentStatus != MultiPartStatus.EPILOGUE || this.bodyListHttpDataRank < this.bodyListHttpData.size()) {
            return this.bodyListHttpData.size() > 0 && this.bodyListHttpDataRank < this.bodyListHttpData.size();
        }
        throw new EndOfDataDecoderException();
    }

    public InterfaceHttpData next() throws EndOfDataDecoderException {
        if (!hasNext()) {
            return null;
        }
        List<InterfaceHttpData> list = this.bodyListHttpData;
        int i = this.bodyListHttpDataRank;
        this.bodyListHttpDataRank = i + 1;
        return list.get(i);
    }

    private void parseBody() throws ErrorDataDecoderException {
        if (this.currentStatus == MultiPartStatus.PREEPILOGUE || this.currentStatus == MultiPartStatus.EPILOGUE) {
            if (this.isLastChunk) {
                this.currentStatus = MultiPartStatus.EPILOGUE;
            }
        } else if (this.isMultipart) {
            parseBodyMultipart();
        } else {
            parseBodyAttributes();
        }
    }

    private void addHttpData(InterfaceHttpData data) {
        if (data != null) {
            List<InterfaceHttpData> datas = this.bodyMapHttpData.get(data.getName());
            if (datas == null) {
                datas = new ArrayList<>(1);
                this.bodyMapHttpData.put(data.getName(), datas);
            }
            datas.add(data);
            this.bodyListHttpData.add(data);
        }
    }

    private void parseBodyAttributesStandard() throws ErrorDataDecoderException {
        int firstpos = this.undecodedChunk.readerIndex();
        int currentpos = firstpos;
        int i = firstpos;
        int i2 = firstpos;
        if (this.currentStatus == MultiPartStatus.NOTSTARTED) {
            this.currentStatus = MultiPartStatus.DISPOSITION;
        }
        boolean contRead = true;
        while (this.undecodedChunk.readable() && contRead) {
            try {
                char read = (char) this.undecodedChunk.readUnsignedByte();
                currentpos++;
                switch (this.currentStatus) {
                    case DISPOSITION:
                        if (read != '=') {
                            if (read != '&') {
                                break;
                            } else {
                                this.currentStatus = MultiPartStatus.DISPOSITION;
                                this.currentAttribute = this.factory.createAttribute(this.request, decodeAttribute(this.undecodedChunk.toString(firstpos, (currentpos - 1) - firstpos, this.charset), this.charset));
                                this.currentAttribute.setValue("");
                                addHttpData(this.currentAttribute);
                                this.currentAttribute = null;
                                firstpos = currentpos;
                                contRead = true;
                                break;
                            }
                        } else {
                            this.currentStatus = MultiPartStatus.FIELD;
                            this.currentAttribute = this.factory.createAttribute(this.request, decodeAttribute(this.undecodedChunk.toString(firstpos, (currentpos - 1) - firstpos, this.charset), this.charset));
                            firstpos = currentpos;
                            break;
                        }
                    case FIELD:
                        if (read != '&') {
                            if (read != 13) {
                                if (read != 10) {
                                    break;
                                } else {
                                    this.currentStatus = MultiPartStatus.PREEPILOGUE;
                                    setFinalBuffer(this.undecodedChunk.slice(firstpos, (currentpos - 1) - firstpos));
                                    firstpos = currentpos;
                                    contRead = false;
                                    break;
                                }
                            } else if (!this.undecodedChunk.readable()) {
                                currentpos--;
                                break;
                            } else {
                                currentpos++;
                                if (((char) this.undecodedChunk.readUnsignedByte()) == 10) {
                                    this.currentStatus = MultiPartStatus.PREEPILOGUE;
                                    setFinalBuffer(this.undecodedChunk.slice(firstpos, (currentpos - 2) - firstpos));
                                    firstpos = currentpos;
                                    contRead = false;
                                    break;
                                } else {
                                    throw new ErrorDataDecoderException("Bad end of line");
                                }
                            }
                        } else {
                            this.currentStatus = MultiPartStatus.DISPOSITION;
                            setFinalBuffer(this.undecodedChunk.slice(firstpos, (currentpos - 1) - firstpos));
                            firstpos = currentpos;
                            contRead = true;
                            break;
                        }
                    default:
                        contRead = false;
                        break;
                }
            } catch (ErrorDataDecoderException e) {
                this.undecodedChunk.readerIndex(firstpos);
                throw e;
            } catch (IOException e2) {
                this.undecodedChunk.readerIndex(firstpos);
                throw new ErrorDataDecoderException((Throwable) e2);
            }
        }
        if (this.isLastChunk && this.currentAttribute != null) {
            int ampersandpos = currentpos;
            if (ampersandpos > firstpos) {
                setFinalBuffer(this.undecodedChunk.slice(firstpos, ampersandpos - firstpos));
            } else if (!this.currentAttribute.isCompleted()) {
                setFinalBuffer(ChannelBuffers.EMPTY_BUFFER);
            }
            int firstpos2 = currentpos;
            this.currentStatus = MultiPartStatus.EPILOGUE;
        } else if (contRead && this.currentAttribute != null) {
            if (this.currentStatus == MultiPartStatus.FIELD) {
                this.currentAttribute.addContent(this.undecodedChunk.slice(firstpos, currentpos - firstpos), false);
                firstpos = currentpos;
            }
            this.undecodedChunk.readerIndex(firstpos);
        }
    }

    private void parseBodyAttributes() throws ErrorDataDecoderException {
        try {
            HttpPostBodyUtil.SeekAheadOptimize sao = new HttpPostBodyUtil.SeekAheadOptimize(this.undecodedChunk);
            int firstpos = this.undecodedChunk.readerIndex();
            int currentpos = firstpos;
            int i = firstpos;
            int i2 = firstpos;
            if (this.currentStatus == MultiPartStatus.NOTSTARTED) {
                this.currentStatus = MultiPartStatus.DISPOSITION;
            }
            boolean contRead = true;
            while (true) {
                try {
                    if (sao.pos < sao.limit) {
                        byte[] bArr = sao.bytes;
                        int i3 = sao.pos;
                        sao.pos = i3 + 1;
                        char read = (char) (bArr[i3] & NavSatStatus.STATUS_NO_FIX);
                        currentpos++;
                        switch (this.currentStatus) {
                            case DISPOSITION:
                                if (read != '=') {
                                    if (read != '&') {
                                        break;
                                    } else {
                                        this.currentStatus = MultiPartStatus.DISPOSITION;
                                        this.currentAttribute = this.factory.createAttribute(this.request, decodeAttribute(this.undecodedChunk.toString(firstpos, (currentpos - 1) - firstpos, this.charset), this.charset));
                                        this.currentAttribute.setValue("");
                                        addHttpData(this.currentAttribute);
                                        this.currentAttribute = null;
                                        firstpos = currentpos;
                                        contRead = true;
                                        continue;
                                    }
                                } else {
                                    this.currentStatus = MultiPartStatus.FIELD;
                                    this.currentAttribute = this.factory.createAttribute(this.request, decodeAttribute(this.undecodedChunk.toString(firstpos, (currentpos - 1) - firstpos, this.charset), this.charset));
                                    firstpos = currentpos;
                                    break;
                                }
                            case FIELD:
                                if (read != '&') {
                                    if (read != 13) {
                                        if (read != 10) {
                                            break;
                                        } else {
                                            this.currentStatus = MultiPartStatus.PREEPILOGUE;
                                            sao.setReadPosition(0);
                                            setFinalBuffer(this.undecodedChunk.slice(firstpos, (currentpos - 1) - firstpos));
                                            firstpos = currentpos;
                                            contRead = false;
                                            break;
                                        }
                                    } else if (sao.pos >= sao.limit) {
                                        if (sao.limit <= 0) {
                                            break;
                                        } else {
                                            currentpos--;
                                            break;
                                        }
                                    } else {
                                        byte[] bArr2 = sao.bytes;
                                        int i4 = sao.pos;
                                        sao.pos = i4 + 1;
                                        currentpos++;
                                        if (((char) (bArr2[i4] & NavSatStatus.STATUS_NO_FIX)) == 10) {
                                            this.currentStatus = MultiPartStatus.PREEPILOGUE;
                                            sao.setReadPosition(0);
                                            setFinalBuffer(this.undecodedChunk.slice(firstpos, (currentpos - 2) - firstpos));
                                            firstpos = currentpos;
                                            contRead = false;
                                            break;
                                        } else {
                                            sao.setReadPosition(0);
                                            throw new ErrorDataDecoderException("Bad end of line");
                                        }
                                    }
                                } else {
                                    this.currentStatus = MultiPartStatus.DISPOSITION;
                                    setFinalBuffer(this.undecodedChunk.slice(firstpos, (currentpos - 1) - firstpos));
                                    firstpos = currentpos;
                                    contRead = true;
                                    continue;
                                }
                            default:
                                sao.setReadPosition(0);
                                contRead = false;
                                break;
                        }
                    }
                } catch (ErrorDataDecoderException e) {
                    this.undecodedChunk.readerIndex(firstpos);
                    throw e;
                } catch (IOException e2) {
                    this.undecodedChunk.readerIndex(firstpos);
                    throw new ErrorDataDecoderException((Throwable) e2);
                }
            }
            if (this.isLastChunk && this.currentAttribute != null) {
                int ampersandpos = currentpos;
                if (ampersandpos > firstpos) {
                    setFinalBuffer(this.undecodedChunk.slice(firstpos, ampersandpos - firstpos));
                } else if (!this.currentAttribute.isCompleted()) {
                    setFinalBuffer(ChannelBuffers.EMPTY_BUFFER);
                }
                int firstpos2 = currentpos;
                this.currentStatus = MultiPartStatus.EPILOGUE;
            } else if (contRead && this.currentAttribute != null) {
                if (this.currentStatus == MultiPartStatus.FIELD) {
                    this.currentAttribute.addContent(this.undecodedChunk.slice(firstpos, currentpos - firstpos), false);
                    firstpos = currentpos;
                }
                this.undecodedChunk.readerIndex(firstpos);
            }
        } catch (HttpPostBodyUtil.SeekAheadNoBackArrayException e3) {
            parseBodyAttributesStandard();
        }
    }

    private void setFinalBuffer(ChannelBuffer buffer) throws ErrorDataDecoderException, IOException {
        this.currentAttribute.addContent(buffer, true);
        this.currentAttribute.setValue(decodeAttribute(this.currentAttribute.getChannelBuffer().toString(this.charset), this.charset));
        addHttpData(this.currentAttribute);
        this.currentAttribute = null;
    }

    private static String decodeAttribute(String s, Charset charset2) throws ErrorDataDecoderException {
        if (s == null) {
            return "";
        }
        try {
            return URLDecoder.decode(s, charset2.name());
        } catch (UnsupportedEncodingException e) {
            throw new ErrorDataDecoderException(charset2.toString(), e);
        }
    }

    private void parseBodyMultipart() throws ErrorDataDecoderException {
        if (this.undecodedChunk != null && this.undecodedChunk.readableBytes() != 0) {
            InterfaceHttpData data = decodeMultipart(this.currentStatus);
            while (data != null) {
                addHttpData(data);
                if (this.currentStatus != MultiPartStatus.PREEPILOGUE && this.currentStatus != MultiPartStatus.EPILOGUE) {
                    data = decodeMultipart(this.currentStatus);
                } else {
                    return;
                }
            }
        }
    }

    private InterfaceHttpData decodeMultipart(MultiPartStatus state) throws ErrorDataDecoderException {
        switch (state) {
            case DISPOSITION:
                return findMultipartDisposition();
            case FIELD:
                Charset localCharset = null;
                Attribute charsetAttribute = this.currentFieldAttributes.get("charset");
                if (charsetAttribute != null) {
                    try {
                        localCharset = Charset.forName(charsetAttribute.getValue());
                    } catch (IOException e) {
                        throw new ErrorDataDecoderException((Throwable) e);
                    }
                }
                Attribute nameAttribute = this.currentFieldAttributes.get("name");
                if (this.currentAttribute == null) {
                    try {
                        this.currentAttribute = this.factory.createAttribute(this.request, nameAttribute.getValue());
                        if (localCharset != null) {
                            this.currentAttribute.setCharset(localCharset);
                        }
                    } catch (NullPointerException e2) {
                        throw new ErrorDataDecoderException((Throwable) e2);
                    } catch (IllegalArgumentException e3) {
                        throw new ErrorDataDecoderException((Throwable) e3);
                    } catch (IOException e4) {
                        throw new ErrorDataDecoderException((Throwable) e4);
                    }
                }
                try {
                    loadFieldMultipart(this.multipartDataBoundary);
                    Attribute finalAttribute = this.currentAttribute;
                    this.currentAttribute = null;
                    this.currentFieldAttributes = null;
                    this.currentStatus = MultiPartStatus.HEADERDELIMITER;
                    return finalAttribute;
                } catch (NotEnoughDataDecoderException e5) {
                    return null;
                }
            case NOTSTARTED:
                throw new ErrorDataDecoderException("Should not be called with the current status");
            case PREAMBLE:
                throw new ErrorDataDecoderException("Should not be called with the current status");
            case HEADERDELIMITER:
                return findMultipartDelimiter(this.multipartDataBoundary, MultiPartStatus.DISPOSITION, MultiPartStatus.PREEPILOGUE);
            case FILEUPLOAD:
                return getFileUpload(this.multipartDataBoundary);
            case MIXEDDELIMITER:
                return findMultipartDelimiter(this.multipartMixedBoundary, MultiPartStatus.MIXEDDISPOSITION, MultiPartStatus.HEADERDELIMITER);
            case MIXEDDISPOSITION:
                return findMultipartDisposition();
            case MIXEDFILEUPLOAD:
                return getFileUpload(this.multipartMixedBoundary);
            case PREEPILOGUE:
                return null;
            case EPILOGUE:
                return null;
            default:
                throw new ErrorDataDecoderException("Shouldn't reach here.");
        }
    }

    /* access modifiers changed from: package-private */
    public void skipControlCharacters() {
        try {
            HttpPostBodyUtil.SeekAheadOptimize sao = new HttpPostBodyUtil.SeekAheadOptimize(this.undecodedChunk);
            while (sao.pos < sao.limit) {
                byte[] bArr = sao.bytes;
                int i = sao.pos;
                sao.pos = i + 1;
                char c = (char) (bArr[i] & NavSatStatus.STATUS_NO_FIX);
                if (!Character.isISOControl(c) && !Character.isWhitespace(c)) {
                    sao.setReadPosition(1);
                    return;
                }
            }
            sao.setReadPosition(0);
        } catch (HttpPostBodyUtil.SeekAheadNoBackArrayException e) {
            skipControlCharactersStandard(this.undecodedChunk);
        }
    }

    static void skipControlCharactersStandard(ChannelBuffer buffer) {
        while (true) {
            char c = (char) buffer.readUnsignedByte();
            if (!Character.isISOControl(c) && !Character.isWhitespace(c)) {
                buffer.readerIndex(buffer.readerIndex() - 1);
                return;
            }
        }
    }

    private InterfaceHttpData findMultipartDelimiter(String delimiter, MultiPartStatus dispositionStatus, MultiPartStatus closeDelimiterStatus) throws ErrorDataDecoderException {
        int readerIndex = this.undecodedChunk.readerIndex();
        skipControlCharacters();
        skipOneLine();
        try {
            String newline = readDelimiter(delimiter);
            if (newline.equals(delimiter)) {
                this.currentStatus = dispositionStatus;
                return decodeMultipart(dispositionStatus);
            }
            if (newline.equals(delimiter + "--")) {
                this.currentStatus = closeDelimiterStatus;
                if (this.currentStatus != MultiPartStatus.HEADERDELIMITER) {
                    return null;
                }
                this.currentFieldAttributes = null;
                return decodeMultipart(MultiPartStatus.HEADERDELIMITER);
            }
            this.undecodedChunk.readerIndex(readerIndex);
            throw new ErrorDataDecoderException("No Multipart delimiter found");
        } catch (NotEnoughDataDecoderException e) {
            this.undecodedChunk.readerIndex(readerIndex);
            return null;
        }
    }

    private InterfaceHttpData findMultipartDisposition() throws ErrorDataDecoderException {
        Attribute attribute;
        boolean checkSecondArg;
        int readerIndex = this.undecodedChunk.readerIndex();
        if (this.currentStatus == MultiPartStatus.DISPOSITION) {
            this.currentFieldAttributes = new TreeMap(CaseIgnoringComparator.INSTANCE);
        }
        Attribute attribute2 = null;
        String[] attribute3 = null;
        while (!skipOneLine()) {
            skipControlCharacters();
            try {
                String newline = readLine();
                String[] contents = splitMultipartHeader(newline);
                int i = 2;
                char c = 1;
                if (contents[0].equalsIgnoreCase("Content-Disposition")) {
                    if (this.currentStatus == MultiPartStatus.DISPOSITION) {
                        checkSecondArg = contents[1].equalsIgnoreCase(HttpPostBodyUtil.FORM_DATA);
                    } else {
                        checkSecondArg = contents[1].equalsIgnoreCase(HttpPostBodyUtil.ATTACHMENT) || contents[1].equalsIgnoreCase(HttpPostBodyUtil.FILE);
                    }
                    if (checkSecondArg) {
                        String[] strArr = attribute3;
                        Attribute attribute4 = attribute2;
                        String[] strArr2 = strArr;
                        while (i < contents.length) {
                            String[] values = contents[i].split("=");
                            try {
                                attribute4 = this.factory.createAttribute(this.request, values[0].trim(), decodeAttribute(cleanString(values[c]), this.charset));
                                this.currentFieldAttributes.put(attribute4.getName(), attribute4);
                                i++;
                                strArr2 = values;
                                c = 1;
                            } catch (NullPointerException e) {
                                throw new ErrorDataDecoderException((Throwable) e);
                            } catch (IllegalArgumentException e2) {
                                throw new ErrorDataDecoderException((Throwable) e2);
                            }
                        }
                        Attribute attribute5 = attribute4;
                        attribute3 = strArr2;
                        attribute2 = attribute5;
                    }
                } else {
                    if (contents[0].equalsIgnoreCase(HttpHeaders.Names.CONTENT_TRANSFER_ENCODING)) {
                        try {
                            attribute = this.factory.createAttribute(this.request, HttpHeaders.Names.CONTENT_TRANSFER_ENCODING, cleanString(contents[1]));
                            this.currentFieldAttributes.put(HttpHeaders.Names.CONTENT_TRANSFER_ENCODING, attribute);
                        } catch (NullPointerException e3) {
                            throw new ErrorDataDecoderException((Throwable) e3);
                        } catch (IllegalArgumentException e4) {
                            throw new ErrorDataDecoderException((Throwable) e4);
                        }
                    } else if (contents[0].equalsIgnoreCase("Content-Length")) {
                        try {
                            attribute = this.factory.createAttribute(this.request, "Content-Length", cleanString(contents[1]));
                            this.currentFieldAttributes.put("Content-Length", attribute);
                        } catch (NullPointerException e5) {
                            throw new ErrorDataDecoderException((Throwable) e5);
                        } catch (IllegalArgumentException e6) {
                            throw new ErrorDataDecoderException((Throwable) e6);
                        }
                    } else if (!contents[0].equalsIgnoreCase("Content-Type")) {
                        throw new ErrorDataDecoderException("Unknown Params: " + newline);
                    } else if (!contents[1].equalsIgnoreCase(HttpPostBodyUtil.MULTIPART_MIXED)) {
                        for (int i2 = 1; i2 < contents.length; i2++) {
                            if (contents[i2].toLowerCase().startsWith("charset")) {
                                try {
                                    attribute3 = this.factory.createAttribute(this.request, "charset", cleanString(contents[i2].split("=")[1]));
                                    this.currentFieldAttributes.put("charset", attribute3);
                                } catch (NullPointerException e7) {
                                    throw new ErrorDataDecoderException((Throwable) e7);
                                } catch (IllegalArgumentException e8) {
                                    throw new ErrorDataDecoderException((Throwable) e8);
                                }
                            } else {
                                try {
                                    Attribute attribute6 = this.factory.createAttribute(this.request, contents[0].trim(), decodeAttribute(cleanString(contents[i2]), this.charset));
                                    this.currentFieldAttributes.put(attribute6.getName(), attribute6);
                                } catch (NullPointerException e9) {
                                    throw new ErrorDataDecoderException((Throwable) e9);
                                } catch (IllegalArgumentException e10) {
                                    throw new ErrorDataDecoderException((Throwable) e10);
                                }
                            }
                        }
                    } else if (this.currentStatus == MultiPartStatus.DISPOSITION) {
                        String[] values2 = contents[2].split("=");
                        this.multipartMixedBoundary = "--" + values2[1];
                        this.currentStatus = MultiPartStatus.MIXEDDELIMITER;
                        return decodeMultipart(MultiPartStatus.MIXEDDELIMITER);
                    } else {
                        throw new ErrorDataDecoderException("Mixed Multipart found in a previous Mixed Multipart");
                    }
                }
            } catch (NotEnoughDataDecoderException e11) {
                NotEnoughDataDecoderException notEnoughDataDecoderException = e11;
                this.undecodedChunk.readerIndex(readerIndex);
                return null;
            }
        }
        Attribute filenameAttribute = this.currentFieldAttributes.get(HttpPostBodyUtil.FILENAME);
        if (this.currentStatus == MultiPartStatus.DISPOSITION) {
            if (filenameAttribute != null) {
                this.currentStatus = MultiPartStatus.FILEUPLOAD;
                return decodeMultipart(MultiPartStatus.FILEUPLOAD);
            }
            this.currentStatus = MultiPartStatus.FIELD;
            return decodeMultipart(MultiPartStatus.FIELD);
        } else if (filenameAttribute != null) {
            this.currentStatus = MultiPartStatus.MIXEDFILEUPLOAD;
            return decodeMultipart(MultiPartStatus.MIXEDFILEUPLOAD);
        } else {
            throw new ErrorDataDecoderException("Filename not found");
        }
    }

    private InterfaceHttpData getFileUpload(String delimiter) throws ErrorDataDecoderException {
        long size;
        Attribute encoding = this.currentFieldAttributes.get(HttpHeaders.Names.CONTENT_TRANSFER_ENCODING);
        Charset localCharset = this.charset;
        HttpPostBodyUtil.TransferEncodingMechanism mechanism = HttpPostBodyUtil.TransferEncodingMechanism.BIT7;
        if (encoding != null) {
            try {
                String code = encoding.getValue().toLowerCase();
                if (code.equals(HttpPostBodyUtil.TransferEncodingMechanism.BIT7.value)) {
                    localCharset = HttpPostBodyUtil.US_ASCII;
                } else if (code.equals(HttpPostBodyUtil.TransferEncodingMechanism.BIT8.value)) {
                    localCharset = HttpPostBodyUtil.ISO_8859_1;
                    mechanism = HttpPostBodyUtil.TransferEncodingMechanism.BIT8;
                } else if (code.equals(HttpPostBodyUtil.TransferEncodingMechanism.BINARY.value)) {
                    mechanism = HttpPostBodyUtil.TransferEncodingMechanism.BINARY;
                } else {
                    throw new ErrorDataDecoderException("TransferEncoding Unknown: " + code);
                }
            } catch (IOException e) {
                throw new ErrorDataDecoderException((Throwable) e);
            }
        }
        Attribute charsetAttribute = this.currentFieldAttributes.get("charset");
        if (charsetAttribute != null) {
            try {
                localCharset = Charset.forName(charsetAttribute.getValue());
            } catch (IOException e2) {
                throw new ErrorDataDecoderException((Throwable) e2);
            }
        }
        if (this.currentFileUpload == null) {
            Attribute filenameAttribute = this.currentFieldAttributes.get(HttpPostBodyUtil.FILENAME);
            Attribute nameAttribute = this.currentFieldAttributes.get("name");
            Attribute contentTypeAttribute = this.currentFieldAttributes.get("Content-Type");
            if (contentTypeAttribute != null) {
                Attribute lengthAttribute = this.currentFieldAttributes.get("Content-Length");
                if (lengthAttribute != null) {
                    try {
                        size = Long.parseLong(lengthAttribute.getValue());
                    } catch (IOException e3) {
                        throw new ErrorDataDecoderException((Throwable) e3);
                    } catch (NumberFormatException e4) {
                        size = 0;
                    }
                } else {
                    size = 0;
                }
                try {
                    this.currentFileUpload = this.factory.createFileUpload(this.request, nameAttribute.getValue(), filenameAttribute.getValue(), contentTypeAttribute.getValue(), mechanism.value, localCharset, size);
                } catch (NullPointerException e5) {
                    throw new ErrorDataDecoderException((Throwable) e5);
                } catch (IllegalArgumentException e6) {
                    throw new ErrorDataDecoderException((Throwable) e6);
                } catch (IOException e7) {
                    throw new ErrorDataDecoderException((Throwable) e7);
                }
            } else {
                throw new ErrorDataDecoderException("Content-Type is absent but required");
            }
        }
        try {
            readFileUploadByteMultipart(delimiter);
            if (!this.currentFileUpload.isCompleted()) {
                return null;
            }
            if (this.currentStatus == MultiPartStatus.FILEUPLOAD) {
                this.currentStatus = MultiPartStatus.HEADERDELIMITER;
                this.currentFieldAttributes = null;
            } else {
                this.currentStatus = MultiPartStatus.MIXEDDELIMITER;
                cleanMixedAttributes();
            }
            FileUpload fileUpload = this.currentFileUpload;
            this.currentFileUpload = null;
            return fileUpload;
        } catch (NotEnoughDataDecoderException e8) {
            NotEnoughDataDecoderException notEnoughDataDecoderException = e8;
            return null;
        }
    }

    public void cleanFiles() {
        this.factory.cleanRequestHttpDatas(this.request);
    }

    public void removeHttpDataFromClean(InterfaceHttpData data) {
        this.factory.removeHttpDataFromClean(this.request, data);
    }

    private void cleanMixedAttributes() {
        this.currentFieldAttributes.remove("charset");
        this.currentFieldAttributes.remove("Content-Length");
        this.currentFieldAttributes.remove(HttpHeaders.Names.CONTENT_TRANSFER_ENCODING);
        this.currentFieldAttributes.remove("Content-Type");
        this.currentFieldAttributes.remove(HttpPostBodyUtil.FILENAME);
    }

    private String readLineStandard() throws NotEnoughDataDecoderException {
        int readerIndex = this.undecodedChunk.readerIndex();
        try {
            StringBuilder sb = new StringBuilder(64);
            while (this.undecodedChunk.readable()) {
                byte nextByte = this.undecodedChunk.readByte();
                if (nextByte == 13) {
                    if (this.undecodedChunk.readByte() == 10) {
                        return sb.toString();
                    }
                } else if (nextByte == 10) {
                    return sb.toString();
                } else {
                    sb.append((char) nextByte);
                }
            }
            this.undecodedChunk.readerIndex(readerIndex);
            throw new NotEnoughDataDecoderException();
        } catch (IndexOutOfBoundsException e) {
            this.undecodedChunk.readerIndex(readerIndex);
            throw new NotEnoughDataDecoderException((Throwable) e);
        }
    }

    private String readLine() throws NotEnoughDataDecoderException {
        try {
            HttpPostBodyUtil.SeekAheadOptimize sao = new HttpPostBodyUtil.SeekAheadOptimize(this.undecodedChunk);
            int readerIndex = this.undecodedChunk.readerIndex();
            try {
                StringBuilder sb = new StringBuilder(64);
                while (sao.pos < sao.limit) {
                    byte[] bArr = sao.bytes;
                    int i = sao.pos;
                    sao.pos = i + 1;
                    byte nextByte = bArr[i];
                    if (nextByte == 13) {
                        if (sao.pos < sao.limit) {
                            byte[] bArr2 = sao.bytes;
                            int i2 = sao.pos;
                            sao.pos = i2 + 1;
                            if (bArr2[i2] == 10) {
                                sao.setReadPosition(0);
                                return sb.toString();
                            }
                        } else {
                            sb.append((char) nextByte);
                        }
                    } else if (nextByte == 10) {
                        sao.setReadPosition(0);
                        return sb.toString();
                    } else {
                        sb.append((char) nextByte);
                    }
                }
                this.undecodedChunk.readerIndex(readerIndex);
                throw new NotEnoughDataDecoderException();
            } catch (IndexOutOfBoundsException e) {
                this.undecodedChunk.readerIndex(readerIndex);
                throw new NotEnoughDataDecoderException((Throwable) e);
            }
        } catch (HttpPostBodyUtil.SeekAheadNoBackArrayException e2) {
            return readLineStandard();
        }
    }

    private String readDelimiterStandard(String delimiter) throws NotEnoughDataDecoderException {
        int readerIndex = this.undecodedChunk.readerIndex();
        try {
            StringBuilder sb = new StringBuilder(64);
            int delimiterPos = 0;
            int len = delimiter.length();
            while (this.undecodedChunk.readable() && delimiterPos < len) {
                byte nextByte = this.undecodedChunk.readByte();
                if (nextByte == delimiter.charAt(delimiterPos)) {
                    delimiterPos++;
                    sb.append((char) nextByte);
                } else {
                    this.undecodedChunk.readerIndex(readerIndex);
                    throw new NotEnoughDataDecoderException();
                }
            }
            if (this.undecodedChunk.readable()) {
                byte nextByte2 = this.undecodedChunk.readByte();
                if (nextByte2 == 13) {
                    if (this.undecodedChunk.readByte() == 10) {
                        return sb.toString();
                    }
                    this.undecodedChunk.readerIndex(readerIndex);
                    throw new NotEnoughDataDecoderException();
                } else if (nextByte2 == 10) {
                    return sb.toString();
                } else {
                    if (nextByte2 == 45) {
                        sb.append((char) nextByte2);
                        byte nextByte3 = this.undecodedChunk.readByte();
                        if (nextByte3 == 45) {
                            sb.append((char) nextByte3);
                            if (!this.undecodedChunk.readable()) {
                                return sb.toString();
                            }
                            byte nextByte4 = this.undecodedChunk.readByte();
                            if (nextByte4 == 13) {
                                if (this.undecodedChunk.readByte() == 10) {
                                    return sb.toString();
                                }
                                this.undecodedChunk.readerIndex(readerIndex);
                                throw new NotEnoughDataDecoderException();
                            } else if (nextByte4 == 10) {
                                return sb.toString();
                            } else {
                                this.undecodedChunk.readerIndex(this.undecodedChunk.readerIndex() - 1);
                                return sb.toString();
                            }
                        }
                    }
                }
            }
            this.undecodedChunk.readerIndex(readerIndex);
            throw new NotEnoughDataDecoderException();
        } catch (IndexOutOfBoundsException e) {
            this.undecodedChunk.readerIndex(readerIndex);
            throw new NotEnoughDataDecoderException((Throwable) e);
        }
    }

    private String readDelimiter(String delimiter) throws NotEnoughDataDecoderException {
        try {
            HttpPostBodyUtil.SeekAheadOptimize sao = new HttpPostBodyUtil.SeekAheadOptimize(this.undecodedChunk);
            int readerIndex = this.undecodedChunk.readerIndex();
            int delimiterPos = 0;
            int len = delimiter.length();
            try {
                StringBuilder sb = new StringBuilder(64);
                while (sao.pos < sao.limit && delimiterPos < len) {
                    byte[] bArr = sao.bytes;
                    int i = sao.pos;
                    sao.pos = i + 1;
                    byte nextByte = bArr[i];
                    if (nextByte == delimiter.charAt(delimiterPos)) {
                        delimiterPos++;
                        sb.append((char) nextByte);
                    } else {
                        this.undecodedChunk.readerIndex(readerIndex);
                        throw new NotEnoughDataDecoderException();
                    }
                }
                if (sao.pos < sao.limit) {
                    byte[] bArr2 = sao.bytes;
                    int i2 = sao.pos;
                    sao.pos = i2 + 1;
                    byte nextByte2 = bArr2[i2];
                    if (nextByte2 == 13) {
                        if (sao.pos < sao.limit) {
                            byte[] bArr3 = sao.bytes;
                            int i3 = sao.pos;
                            sao.pos = i3 + 1;
                            if (bArr3[i3] == 10) {
                                sao.setReadPosition(0);
                                return sb.toString();
                            }
                        } else {
                            this.undecodedChunk.readerIndex(readerIndex);
                            throw new NotEnoughDataDecoderException();
                        }
                    } else if (nextByte2 == 10) {
                        sao.setReadPosition(0);
                        return sb.toString();
                    } else if (nextByte2 == 45) {
                        sb.append((char) nextByte2);
                        if (sao.pos < sao.limit) {
                            byte[] bArr4 = sao.bytes;
                            int i4 = sao.pos;
                            sao.pos = i4 + 1;
                            byte nextByte3 = bArr4[i4];
                            if (nextByte3 == 45) {
                                sb.append((char) nextByte3);
                                if (sao.pos < sao.limit) {
                                    byte[] bArr5 = sao.bytes;
                                    int i5 = sao.pos;
                                    sao.pos = i5 + 1;
                                    byte nextByte4 = bArr5[i5];
                                    if (nextByte4 == 13) {
                                        if (sao.pos < sao.limit) {
                                            byte[] bArr6 = sao.bytes;
                                            int i6 = sao.pos;
                                            sao.pos = i6 + 1;
                                            if (bArr6[i6] == 10) {
                                                sao.setReadPosition(0);
                                                return sb.toString();
                                            }
                                        } else {
                                            this.undecodedChunk.readerIndex(readerIndex);
                                            throw new NotEnoughDataDecoderException();
                                        }
                                    } else if (nextByte4 == 10) {
                                        sao.setReadPosition(0);
                                        return sb.toString();
                                    } else {
                                        sao.setReadPosition(1);
                                        return sb.toString();
                                    }
                                }
                                sao.setReadPosition(0);
                                return sb.toString();
                            }
                        }
                    }
                }
                this.undecodedChunk.readerIndex(readerIndex);
                throw new NotEnoughDataDecoderException();
            } catch (IndexOutOfBoundsException e) {
                this.undecodedChunk.readerIndex(readerIndex);
                throw new NotEnoughDataDecoderException((Throwable) e);
            }
        } catch (HttpPostBodyUtil.SeekAheadNoBackArrayException e2) {
            return readDelimiterStandard(delimiter);
        }
    }

    private void readFileUploadByteMultipartStandard(String delimiter) throws NotEnoughDataDecoderException, ErrorDataDecoderException {
        int i;
        int i2;
        boolean newLine;
        int index;
        int readerIndex = this.undecodedChunk.readerIndex();
        int lastPosition = this.undecodedChunk.readerIndex();
        int index2 = 0;
        boolean newLine2 = true;
        boolean found = false;
        while (true) {
            if (!this.undecodedChunk.readable()) {
                break;
            }
            byte nextByte = this.undecodedChunk.readByte();
            if (newLine2) {
                if (nextByte == delimiter.codePointAt(index2)) {
                    index2++;
                    if (delimiter.length() == index2) {
                        found = true;
                        break;
                    }
                } else {
                    newLine2 = false;
                    index2 = 0;
                    if (nextByte == 13) {
                        if (this.undecodedChunk.readable() && this.undecodedChunk.readByte() == 10) {
                            newLine2 = true;
                            index2 = 0;
                            i2 = this.undecodedChunk.readerIndex();
                        }
                    } else if (nextByte == 10) {
                        newLine = true;
                        index = 0;
                        i = this.undecodedChunk.readerIndex();
                        lastPosition = i - 1;
                    } else {
                        lastPosition = this.undecodedChunk.readerIndex();
                    }
                }
            } else if (nextByte == 13) {
                if (this.undecodedChunk.readable() && this.undecodedChunk.readByte() == 10) {
                    newLine2 = true;
                    index2 = 0;
                    i2 = this.undecodedChunk.readerIndex();
                }
            } else if (nextByte == 10) {
                newLine = true;
                index = 0;
                i = this.undecodedChunk.readerIndex();
                lastPosition = i - 1;
            } else {
                lastPosition = this.undecodedChunk.readerIndex();
            }
            lastPosition = i2 - 2;
        }
        ChannelBuffer buffer = this.undecodedChunk.slice(readerIndex, lastPosition - readerIndex);
        if (found) {
            try {
                this.currentFileUpload.addContent(buffer, true);
                this.undecodedChunk.readerIndex(lastPosition);
            } catch (IOException e) {
                throw new ErrorDataDecoderException((Throwable) e);
            }
        } else {
            try {
                this.currentFileUpload.addContent(buffer, false);
                this.undecodedChunk.readerIndex(lastPosition);
                throw new NotEnoughDataDecoderException();
            } catch (IOException e2) {
                throw new ErrorDataDecoderException((Throwable) e2);
            }
        }
    }

    private void readFileUploadByteMultipart(String delimiter) throws NotEnoughDataDecoderException, ErrorDataDecoderException {
        int i;
        try {
            HttpPostBodyUtil.SeekAheadOptimize sao = new HttpPostBodyUtil.SeekAheadOptimize(this.undecodedChunk);
            int readerIndex = this.undecodedChunk.readerIndex();
            int setReadPosition = -1;
            int lastPosition = this.undecodedChunk.readerIndex();
            int index = 0;
            boolean newLine = true;
            boolean found = false;
            while (true) {
                if (sao.pos >= sao.limit) {
                    break;
                }
                byte[] bArr = sao.bytes;
                int i2 = sao.pos;
                sao.pos = i2 + 1;
                byte nextByte = bArr[i2];
                if (newLine) {
                    if (nextByte == delimiter.codePointAt(index)) {
                        index++;
                        if (delimiter.length() == index) {
                            found = true;
                            sao.setReadPosition(0);
                            break;
                        }
                    } else {
                        newLine = false;
                        index = 0;
                        if (nextByte == 13) {
                            if (sao.pos < sao.limit) {
                                byte[] bArr2 = sao.bytes;
                                int i3 = sao.pos;
                                sao.pos = i3 + 1;
                                if (bArr2[i3] == 10) {
                                    newLine = true;
                                    index = 0;
                                    setReadPosition = sao.pos;
                                    i = sao.pos - 2;
                                }
                            } else {
                                setReadPosition = sao.pos;
                                lastPosition = sao.pos;
                            }
                        } else if (nextByte == 10) {
                            newLine = true;
                            index = 0;
                            setReadPosition = sao.pos;
                            i = sao.pos - 1;
                        } else {
                            setReadPosition = sao.pos;
                            lastPosition = sao.pos;
                        }
                    }
                } else if (nextByte == 13) {
                    if (sao.pos < sao.limit) {
                        byte[] bArr3 = sao.bytes;
                        int i4 = sao.pos;
                        sao.pos = i4 + 1;
                        if (bArr3[i4] == 10) {
                            newLine = true;
                            index = 0;
                            setReadPosition = sao.pos;
                            i = sao.pos - 2;
                        }
                    } else {
                        setReadPosition = sao.pos;
                        lastPosition = sao.pos;
                    }
                } else if (nextByte == 10) {
                    newLine = true;
                    index = 0;
                    setReadPosition = sao.pos;
                    i = sao.pos - 1;
                } else {
                    setReadPosition = sao.pos;
                    lastPosition = sao.pos;
                }
                lastPosition = i;
            }
            if (setReadPosition > 0) {
                sao.pos = setReadPosition;
                sao.setReadPosition(0);
            }
            ChannelBuffer buffer = this.undecodedChunk.slice(readerIndex, lastPosition - readerIndex);
            if (found) {
                try {
                    this.currentFileUpload.addContent(buffer, true);
                    this.undecodedChunk.readerIndex(lastPosition);
                } catch (IOException e) {
                    throw new ErrorDataDecoderException((Throwable) e);
                }
            } else {
                try {
                    this.currentFileUpload.addContent(buffer, false);
                    this.undecodedChunk.readerIndex(lastPosition);
                    throw new NotEnoughDataDecoderException();
                } catch (IOException e2) {
                    throw new ErrorDataDecoderException((Throwable) e2);
                }
            }
        } catch (HttpPostBodyUtil.SeekAheadNoBackArrayException e3) {
            readFileUploadByteMultipartStandard(delimiter);
        }
    }

    private void loadFieldMultipartStandard(String delimiter) throws NotEnoughDataDecoderException, ErrorDataDecoderException {
        int i;
        int i2;
        boolean newLine;
        int index;
        int readerIndex = this.undecodedChunk.readerIndex();
        try {
            int lastPosition = this.undecodedChunk.readerIndex();
            int index2 = 0;
            boolean newLine2 = true;
            boolean found = false;
            while (true) {
                if (!this.undecodedChunk.readable()) {
                    break;
                }
                byte nextByte = this.undecodedChunk.readByte();
                if (newLine2) {
                    if (nextByte == delimiter.codePointAt(index2)) {
                        index2++;
                        if (delimiter.length() == index2) {
                            found = true;
                            break;
                        }
                    } else {
                        newLine2 = false;
                        index2 = 0;
                        if (nextByte == 13) {
                            if (this.undecodedChunk.readable() && this.undecodedChunk.readByte() == 10) {
                                newLine2 = true;
                                index2 = 0;
                                i2 = this.undecodedChunk.readerIndex() - 2;
                            }
                        } else if (nextByte == 10) {
                            newLine = true;
                            index = 0;
                            i = this.undecodedChunk.readerIndex() - 1;
                            lastPosition = i;
                        } else {
                            lastPosition = this.undecodedChunk.readerIndex();
                        }
                    }
                } else if (nextByte == 13) {
                    if (this.undecodedChunk.readable() && this.undecodedChunk.readByte() == 10) {
                        newLine2 = true;
                        index2 = 0;
                        i2 = this.undecodedChunk.readerIndex() - 2;
                    }
                } else if (nextByte == 10) {
                    newLine = true;
                    index = 0;
                    i = this.undecodedChunk.readerIndex() - 1;
                    lastPosition = i;
                } else {
                    lastPosition = this.undecodedChunk.readerIndex();
                }
                lastPosition = i2;
            }
            if (found) {
                this.currentAttribute.addContent(this.undecodedChunk.slice(readerIndex, lastPosition - readerIndex), true);
                this.undecodedChunk.readerIndex(lastPosition);
                return;
            }
            this.currentAttribute.addContent(this.undecodedChunk.slice(readerIndex, lastPosition - readerIndex), false);
            this.undecodedChunk.readerIndex(lastPosition);
            throw new NotEnoughDataDecoderException();
        } catch (IOException e) {
            throw new ErrorDataDecoderException((Throwable) e);
        } catch (IOException e2) {
            throw new ErrorDataDecoderException((Throwable) e2);
        } catch (IndexOutOfBoundsException e3) {
            this.undecodedChunk.readerIndex(readerIndex);
            throw new NotEnoughDataDecoderException((Throwable) e3);
        }
    }

    private void loadFieldMultipart(String delimiter) throws NotEnoughDataDecoderException, ErrorDataDecoderException {
        int lastPosition;
        int lastPosition2;
        try {
            HttpPostBodyUtil.SeekAheadOptimize sao = new HttpPostBodyUtil.SeekAheadOptimize(this.undecodedChunk);
            int readerIndex = this.undecodedChunk.readerIndex();
            try {
                int setReadPosition = -1;
                int lastPosition3 = this.undecodedChunk.readerIndex();
                int index = 0;
                boolean newLine = true;
                boolean found = false;
                while (true) {
                    if (sao.pos >= sao.limit) {
                        break;
                    }
                    byte[] bArr = sao.bytes;
                    int i = sao.pos;
                    sao.pos = i + 1;
                    byte nextByte = bArr[i];
                    if (newLine) {
                        if (nextByte == delimiter.codePointAt(index)) {
                            index++;
                            if (delimiter.length() == index) {
                                found = true;
                                sao.setReadPosition(0);
                                break;
                            }
                        } else {
                            newLine = false;
                            index = 0;
                            if (nextByte == 13) {
                                if (sao.pos < sao.limit) {
                                    byte[] bArr2 = sao.bytes;
                                    int i2 = sao.pos;
                                    sao.pos = i2 + 1;
                                    if (bArr2[i2] == 10) {
                                        newLine = true;
                                        index = 0;
                                        lastPosition = sao.pos - 2;
                                        lastPosition2 = sao.pos;
                                    }
                                } else {
                                    lastPosition3 = sao.pos;
                                    setReadPosition = sao.pos;
                                }
                            } else if (nextByte == 10) {
                                newLine = true;
                                index = 0;
                                lastPosition = sao.pos - 1;
                                lastPosition2 = sao.pos;
                            } else {
                                lastPosition3 = sao.pos;
                                setReadPosition = sao.pos;
                            }
                        }
                    } else if (nextByte == 13) {
                        if (sao.pos < sao.limit) {
                            byte[] bArr3 = sao.bytes;
                            int i3 = sao.pos;
                            sao.pos = i3 + 1;
                            if (bArr3[i3] == 10) {
                                newLine = true;
                                index = 0;
                                lastPosition = sao.pos - 2;
                                lastPosition2 = sao.pos;
                            }
                        } else {
                            lastPosition3 = sao.pos;
                            setReadPosition = sao.pos;
                        }
                    } else if (nextByte == 10) {
                        newLine = true;
                        index = 0;
                        lastPosition = sao.pos - 1;
                        lastPosition2 = sao.pos;
                    } else {
                        lastPosition3 = sao.pos;
                        setReadPosition = sao.pos;
                    }
                    setReadPosition = lastPosition2;
                    lastPosition3 = lastPosition;
                }
                if (setReadPosition > 0) {
                    sao.pos = setReadPosition;
                    sao.setReadPosition(0);
                }
                if (found) {
                    this.currentAttribute.addContent(this.undecodedChunk.slice(readerIndex, lastPosition3 - readerIndex), true);
                    this.undecodedChunk.readerIndex(lastPosition3);
                    return;
                }
                this.currentAttribute.addContent(this.undecodedChunk.slice(readerIndex, lastPosition3 - readerIndex), false);
                this.undecodedChunk.readerIndex(lastPosition3);
                throw new NotEnoughDataDecoderException();
            } catch (IOException e) {
                throw new ErrorDataDecoderException((Throwable) e);
            } catch (IOException e2) {
                throw new ErrorDataDecoderException((Throwable) e2);
            } catch (IndexOutOfBoundsException e3) {
                this.undecodedChunk.readerIndex(readerIndex);
                throw new NotEnoughDataDecoderException((Throwable) e3);
            }
        } catch (HttpPostBodyUtil.SeekAheadNoBackArrayException e4) {
            loadFieldMultipartStandard(delimiter);
        }
    }

    private static String cleanString(String field) {
        StringBuilder sb = new StringBuilder(field.length());
        for (int i = 0; i < field.length(); i++) {
            char nextChar = field.charAt(i);
            if (nextChar == ':') {
                sb.append(32);
            } else if (nextChar == ',') {
                sb.append(32);
            } else if (nextChar == '=') {
                sb.append(32);
            } else if (nextChar == ';') {
                sb.append(32);
            } else if (nextChar == 9) {
                sb.append(32);
            } else if (nextChar != '\"') {
                sb.append(nextChar);
            }
        }
        return sb.toString().trim();
    }

    private boolean skipOneLine() {
        if (!this.undecodedChunk.readable()) {
            return false;
        }
        byte nextByte = this.undecodedChunk.readByte();
        if (nextByte == 13) {
            if (!this.undecodedChunk.readable()) {
                this.undecodedChunk.readerIndex(this.undecodedChunk.readerIndex() - 1);
                return false;
            } else if (this.undecodedChunk.readByte() == 10) {
                return true;
            } else {
                this.undecodedChunk.readerIndex(this.undecodedChunk.readerIndex() - 2);
                return false;
            }
        } else if (nextByte == 10) {
            return true;
        } else {
            this.undecodedChunk.readerIndex(this.undecodedChunk.readerIndex() - 1);
            return false;
        }
    }

    private static String[] splitHeaderContentType(String sb) {
        int size = sb.length();
        int aStart = HttpPostBodyUtil.findNonWhitespace(sb, 0);
        int aEnd = HttpPostBodyUtil.findWhitespace(sb, aStart);
        if (aEnd >= size) {
            return new String[]{sb, ""};
        }
        if (sb.charAt(aEnd) == ';') {
            aEnd--;
        }
        return new String[]{sb.substring(aStart, aEnd), sb.substring(HttpPostBodyUtil.findNonWhitespace(sb, aEnd), HttpPostBodyUtil.findEndOfString(sb))};
    }

    private static String[] splitMultipartHeader(String sb) {
        String[] values;
        ArrayList<String> headers = new ArrayList<>(1);
        int nameStart = HttpPostBodyUtil.findNonWhitespace(sb, 0);
        int nameEnd = nameStart;
        while (nameEnd < sb.length() && (ch = sb.charAt(nameEnd)) != ':' && !Character.isWhitespace(ch)) {
            nameEnd++;
        }
        int colonEnd = nameEnd;
        while (true) {
            if (colonEnd >= sb.length()) {
                break;
            } else if (sb.charAt(colonEnd) == ':') {
                colonEnd++;
                break;
            } else {
                colonEnd++;
            }
        }
        int valueStart = HttpPostBodyUtil.findNonWhitespace(sb, colonEnd);
        int valueEnd = HttpPostBodyUtil.findEndOfString(sb);
        headers.add(sb.substring(nameStart, nameEnd));
        String svalue = sb.substring(valueStart, valueEnd);
        if (svalue.indexOf(";") >= 0) {
            values = svalue.split(";");
        } else {
            values = svalue.split(",");
        }
        for (String value : values) {
            headers.add(value.trim());
        }
        String[] array = new String[headers.size()];
        for (int i = 0; i < headers.size(); i++) {
            array[i] = headers.get(i);
        }
        return array;
    }

    public static class NotEnoughDataDecoderException extends Exception {
        private static final long serialVersionUID = -7846841864603865638L;

        public NotEnoughDataDecoderException() {
        }

        public NotEnoughDataDecoderException(String arg0) {
            super(arg0);
        }

        public NotEnoughDataDecoderException(Throwable arg0) {
            super(arg0);
        }

        public NotEnoughDataDecoderException(String arg0, Throwable arg1) {
            super(arg0, arg1);
        }
    }

    public static class ErrorDataDecoderException extends Exception {
        private static final long serialVersionUID = 5020247425493164465L;

        public ErrorDataDecoderException() {
        }

        public ErrorDataDecoderException(String arg0) {
            super(arg0);
        }

        public ErrorDataDecoderException(Throwable arg0) {
            super(arg0);
        }

        public ErrorDataDecoderException(String arg0, Throwable arg1) {
            super(arg0, arg1);
        }
    }

    public static class IncompatibleDataDecoderException extends Exception {
        private static final long serialVersionUID = -953268047926250267L;

        public IncompatibleDataDecoderException() {
        }

        public IncompatibleDataDecoderException(String arg0) {
            super(arg0);
        }

        public IncompatibleDataDecoderException(Throwable arg0) {
            super(arg0);
        }

        public IncompatibleDataDecoderException(String arg0, Throwable arg1) {
            super(arg0, arg1);
        }
    }
}
