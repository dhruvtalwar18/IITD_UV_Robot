package org.apache.xmlrpc.client;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.zip.GZIPInputStream;
import java.util.zip.GZIPOutputStream;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.common.XmlRpcStreamConfig;
import org.apache.xmlrpc.common.XmlRpcStreamRequestConfig;
import org.apache.xmlrpc.parser.XmlRpcResponseParser;
import org.apache.xmlrpc.serializer.XmlRpcWriter;
import org.apache.xmlrpc.util.SAXParsers;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;
import org.xml.sax.XMLReader;

public abstract class XmlRpcStreamTransport extends XmlRpcTransportImpl {

    protected interface ReqWriter {
        void write(OutputStream outputStream) throws XmlRpcException, IOException, SAXException;
    }

    /* access modifiers changed from: protected */
    public abstract void close() throws XmlRpcClientException;

    /* access modifiers changed from: protected */
    public abstract InputStream getInputStream() throws XmlRpcException;

    /* access modifiers changed from: protected */
    public abstract boolean isResponseGzipCompressed(XmlRpcStreamRequestConfig xmlRpcStreamRequestConfig);

    /* access modifiers changed from: protected */
    public abstract void writeRequest(ReqWriter reqWriter) throws XmlRpcException, IOException, SAXException;

    protected class ReqWriterImpl implements ReqWriter {
        private final XmlRpcRequest request;

        protected ReqWriterImpl(XmlRpcRequest pRequest) {
            this.request = pRequest;
        }

        public void write(OutputStream pStream) throws XmlRpcException, IOException, SAXException {
            XmlRpcStreamConfig config = (XmlRpcStreamConfig) this.request.getConfig();
            try {
                new XmlRpcWriter(config, XmlRpcStreamTransport.this.getClient().getXmlWriterFactory().getXmlWriter(config, pStream), XmlRpcStreamTransport.this.getClient().getTypeFactory()).write(this.request);
                pStream.close();
                OutputStream pStream2 = null;
                if (pStream2 != null) {
                    try {
                        pStream2.close();
                        return;
                    } catch (Throwable th) {
                        return;
                    }
                } else {
                    return;
                }
            } catch (Throwable th2) {
            }
            throw th;
        }
    }

    protected class GzipReqWriter implements ReqWriter {
        private final ReqWriter reqWriter;

        protected GzipReqWriter(ReqWriter pReqWriter) {
            this.reqWriter = pReqWriter;
        }

        public void write(OutputStream pStream) throws XmlRpcException, IOException, SAXException {
            try {
                this.reqWriter.write(new GZIPOutputStream(pStream));
                pStream.close();
                OutputStream pStream2 = null;
                if (pStream2 != null) {
                    try {
                        pStream2.close();
                        return;
                    } catch (Throwable th) {
                        return;
                    }
                } else {
                    return;
                }
            } catch (IOException e) {
                throw new XmlRpcException("Failed to write request: " + e.getMessage(), (Throwable) e);
            } catch (Throwable th2) {
            }
            throw th;
        }
    }

    protected XmlRpcStreamTransport(XmlRpcClient pClient) {
        super(pClient);
    }

    /* access modifiers changed from: protected */
    public boolean isCompressingRequest(XmlRpcStreamRequestConfig pConfig) {
        return pConfig.isEnabledForExtensions() && pConfig.isGzipCompressing();
    }

    /* access modifiers changed from: protected */
    public ReqWriter newReqWriter(XmlRpcRequest pRequest) throws XmlRpcException, IOException, SAXException {
        ReqWriter reqWriter = new ReqWriterImpl(pRequest);
        if (isCompressingRequest((XmlRpcStreamRequestConfig) pRequest.getConfig())) {
            return new GzipReqWriter(reqWriter);
        }
        return reqWriter;
    }

    public Object sendRequest(XmlRpcRequest pRequest) throws XmlRpcException {
        XmlRpcStreamRequestConfig config = (XmlRpcStreamRequestConfig) pRequest.getConfig();
        try {
            writeRequest(newReqWriter(pRequest));
            InputStream istream = getInputStream();
            if (isResponseGzipCompressed(config)) {
                istream = new GZIPInputStream(istream);
            }
            Object result = readResponse(config, istream);
            close();
            if (1 == 0) {
                try {
                    close();
                } catch (Throwable th) {
                }
            }
            return result;
        } catch (IOException e) {
            throw new XmlRpcException("Failed to read server's response: " + e.getMessage(), (Throwable) e);
        } catch (SAXException e2) {
            Exception ex = e2.getException();
            if (ex == null || !(ex instanceof XmlRpcException)) {
                throw new XmlRpcException("Failed to generate request: " + e2.getMessage(), (Throwable) e2);
            }
            throw ((XmlRpcException) ex);
        } catch (Throwable th2) {
        }
        throw th;
    }

    /* access modifiers changed from: protected */
    public XMLReader newXMLReader() throws XmlRpcException {
        return SAXParsers.newXMLReader();
    }

    /* access modifiers changed from: protected */
    public Object readResponse(XmlRpcStreamRequestConfig pConfig, InputStream pStream) throws XmlRpcException {
        InputSource isource = new InputSource(pStream);
        XMLReader xr = newXMLReader();
        try {
            XmlRpcResponseParser xp = new XmlRpcResponseParser(pConfig, getClient().getTypeFactory());
            xr.setContentHandler(xp);
            xr.parse(isource);
            if (xp.isSuccess()) {
                return xp.getResult();
            }
            Throwable t = xp.getErrorCause();
            if (t == null) {
                throw new XmlRpcException(xp.getErrorCode(), xp.getErrorMessage());
            } else if (t instanceof XmlRpcException) {
                throw ((XmlRpcException) t);
            } else if (t instanceof RuntimeException) {
                throw ((RuntimeException) t);
            } else {
                throw new XmlRpcException(xp.getErrorCode(), xp.getErrorMessage(), t);
            }
        } catch (SAXException e) {
            throw new XmlRpcClientException("Failed to parse server's response: " + e.getMessage(), e);
        } catch (IOException e2) {
            throw new XmlRpcClientException("Failed to read server's response: " + e2.getMessage(), e2);
        }
    }
}
