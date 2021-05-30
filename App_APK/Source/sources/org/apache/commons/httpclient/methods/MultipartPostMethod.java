package org.apache.commons.httpclient.methods;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.httpclient.HttpConnection;
import org.apache.commons.httpclient.HttpException;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.httpclient.methods.multipart.FilePart;
import org.apache.commons.httpclient.methods.multipart.Part;
import org.apache.commons.httpclient.methods.multipart.StringPart;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

public class MultipartPostMethod extends ExpectContinueMethod {
    private static final Log LOG;
    public static final String MULTIPART_FORM_CONTENT_TYPE = "multipart/form-data";
    static /* synthetic */ Class class$org$apache$commons$httpclient$methods$MultipartPostMethod;
    private final List parameters = new ArrayList();

    static {
        Class cls;
        if (class$org$apache$commons$httpclient$methods$MultipartPostMethod == null) {
            cls = class$("org.apache.commons.httpclient.methods.MultipartPostMethod");
            class$org$apache$commons$httpclient$methods$MultipartPostMethod = cls;
        } else {
            cls = class$org$apache$commons$httpclient$methods$MultipartPostMethod;
        }
        LOG = LogFactory.getLog(cls);
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    public MultipartPostMethod() {
    }

    public MultipartPostMethod(String uri) {
        super(uri);
    }

    /* access modifiers changed from: protected */
    public boolean hasRequestContent() {
        return true;
    }

    public String getName() {
        return "POST";
    }

    public void addParameter(String parameterName, String parameterValue) {
        LOG.trace("enter addParameter(String parameterName, String parameterValue)");
        this.parameters.add(new StringPart(parameterName, parameterValue));
    }

    public void addParameter(String parameterName, File parameterFile) throws FileNotFoundException {
        LOG.trace("enter MultipartPostMethod.addParameter(String parameterName, File parameterFile)");
        this.parameters.add(new FilePart(parameterName, parameterFile));
    }

    public void addParameter(String parameterName, String fileName, File parameterFile) throws FileNotFoundException {
        LOG.trace("enter MultipartPostMethod.addParameter(String parameterName, String fileName, File parameterFile)");
        this.parameters.add(new FilePart(parameterName, fileName, parameterFile));
    }

    public void addPart(Part part) {
        LOG.trace("enter addPart(Part part)");
        this.parameters.add(part);
    }

    public Part[] getParts() {
        return (Part[]) this.parameters.toArray(new Part[this.parameters.size()]);
    }

    /* access modifiers changed from: protected */
    public void addContentLengthRequestHeader(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter EntityEnclosingMethod.addContentLengthRequestHeader(HttpState, HttpConnection)");
        if (getRequestHeader("Content-Length") == null) {
            addRequestHeader("Content-Length", String.valueOf(getRequestContentLength()));
        }
        removeRequestHeader("Transfer-Encoding");
    }

    /* access modifiers changed from: protected */
    public void addContentTypeRequestHeader(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter EntityEnclosingMethod.addContentTypeRequestHeader(HttpState, HttpConnection)");
        if (!this.parameters.isEmpty()) {
            StringBuffer buffer = new StringBuffer("multipart/form-data");
            if (Part.getBoundary() != null) {
                buffer.append("; boundary=");
                buffer.append(Part.getBoundary());
            }
            setRequestHeader("Content-Type", buffer.toString());
        }
    }

    /* access modifiers changed from: protected */
    public void addRequestHeaders(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter MultipartPostMethod.addRequestHeaders(HttpState state, HttpConnection conn)");
        super.addRequestHeaders(state, conn);
        addContentLengthRequestHeader(state, conn);
        addContentTypeRequestHeader(state, conn);
    }

    /* access modifiers changed from: protected */
    public boolean writeRequestBody(HttpState state, HttpConnection conn) throws IOException, HttpException {
        LOG.trace("enter MultipartPostMethod.writeRequestBody(HttpState state, HttpConnection conn)");
        Part.sendParts(conn.getRequestOutputStream(), getParts());
        return true;
    }

    /* access modifiers changed from: protected */
    public long getRequestContentLength() throws IOException {
        LOG.trace("enter MultipartPostMethod.getRequestContentLength()");
        return Part.getLengthOfParts(getParts());
    }

    public void recycle() {
        LOG.trace("enter MultipartPostMethod.recycle()");
        super.recycle();
        this.parameters.clear();
    }
}
