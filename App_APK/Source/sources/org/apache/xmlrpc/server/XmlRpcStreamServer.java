package org.apache.xmlrpc.server;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.zip.GZIPInputStream;
import java.util.zip.GZIPOutputStream;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.XmlRpcRequestConfig;
import org.apache.xmlrpc.common.ServerStreamConnection;
import org.apache.xmlrpc.common.XmlRpcStreamRequestConfig;
import org.apache.xmlrpc.common.XmlRpcStreamRequestProcessor;
import org.apache.xmlrpc.parser.XmlRpcRequestParser;
import org.apache.xmlrpc.serializer.DefaultXMLWriterFactory;
import org.apache.xmlrpc.serializer.XmlRpcWriter;
import org.apache.xmlrpc.serializer.XmlWriterFactory;
import org.apache.xmlrpc.util.SAXParsers;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;
import org.xml.sax.XMLReader;

public abstract class XmlRpcStreamServer extends XmlRpcServer implements XmlRpcStreamRequestProcessor {
    private static final Log log = LogFactory.getLog(XmlRpcStreamServer.class);
    private static final XmlRpcErrorLogger theErrorLogger = new XmlRpcErrorLogger();
    private XmlRpcErrorLogger errorLogger = theErrorLogger;
    private XmlWriterFactory writerFactory = new DefaultXMLWriterFactory();

    /* access modifiers changed from: protected */
    public XmlRpcRequest getRequest(final XmlRpcStreamRequestConfig pConfig, InputStream pStream) throws XmlRpcException {
        final XmlRpcRequestParser parser = new XmlRpcRequestParser(pConfig, getTypeFactory());
        XMLReader xr = SAXParsers.newXMLReader();
        xr.setContentHandler(parser);
        try {
            xr.parse(new InputSource(pStream));
            final List params = parser.getParams();
            return new XmlRpcRequest() {
                public XmlRpcRequestConfig getConfig() {
                    return pConfig;
                }

                public String getMethodName() {
                    return parser.getMethodName();
                }

                public int getParameterCount() {
                    if (params == null) {
                        return 0;
                    }
                    return params.size();
                }

                public Object getParameter(int pIndex) {
                    return params.get(pIndex);
                }
            };
        } catch (SAXException e) {
            Exception ex = e.getException();
            if (ex == null || !(ex instanceof XmlRpcException)) {
                throw new XmlRpcException("Failed to parse XML-RPC request: " + e.getMessage(), (Throwable) e);
            }
            throw ((XmlRpcException) ex);
        } catch (IOException e2) {
            throw new XmlRpcException("Failed to read XML-RPC request: " + e2.getMessage(), (Throwable) e2);
        }
    }

    /* access modifiers changed from: protected */
    public XmlRpcWriter getXmlRpcWriter(XmlRpcStreamRequestConfig pConfig, OutputStream pStream) throws XmlRpcException {
        return new XmlRpcWriter(pConfig, getXMLWriterFactory().getXmlWriter(pConfig, pStream), getTypeFactory());
    }

    /* access modifiers changed from: protected */
    public void writeResponse(XmlRpcStreamRequestConfig pConfig, OutputStream pStream, Object pResult) throws XmlRpcException {
        try {
            getXmlRpcWriter(pConfig, pStream).write(pConfig, pResult);
        } catch (SAXException e) {
            throw new XmlRpcException("Failed to write XML-RPC response: " + e.getMessage(), (Throwable) e);
        }
    }

    /* access modifiers changed from: protected */
    public Throwable convertThrowable(Throwable pError) {
        return pError;
    }

    /* access modifiers changed from: protected */
    public void writeError(XmlRpcStreamRequestConfig pConfig, OutputStream pStream, Throwable pError) throws XmlRpcException {
        int code;
        Throwable error = convertThrowable(pError);
        if (error instanceof XmlRpcException) {
            code = ((XmlRpcException) error).code;
        } else {
            code = 0;
        }
        try {
            getXmlRpcWriter(pConfig, pStream).write(pConfig, code, error.getMessage(), error);
        } catch (SAXException e) {
            throw new XmlRpcException("Failed to write XML-RPC response: " + e.getMessage(), (Throwable) e);
        }
    }

    public void setXMLWriterFactory(XmlWriterFactory pFactory) {
        this.writerFactory = pFactory;
    }

    public XmlWriterFactory getXMLWriterFactory() {
        return this.writerFactory;
    }

    /* access modifiers changed from: protected */
    public InputStream getInputStream(XmlRpcStreamRequestConfig pConfig, ServerStreamConnection pConnection) throws IOException {
        InputStream istream = pConnection.newInputStream();
        if (!pConfig.isEnabledForExtensions() || !pConfig.isGzipCompressing()) {
            return istream;
        }
        return new GZIPInputStream(istream);
    }

    /* access modifiers changed from: protected */
    public OutputStream getOutputStream(ServerStreamConnection pConnection, XmlRpcStreamRequestConfig pConfig, OutputStream pStream) throws IOException {
        if (!pConfig.isEnabledForExtensions() || !pConfig.isGzipRequesting()) {
            return pStream;
        }
        return new GZIPOutputStream(pStream);
    }

    /* access modifiers changed from: protected */
    public OutputStream getOutputStream(XmlRpcStreamRequestConfig pConfig, ServerStreamConnection pConnection, int pSize) throws IOException {
        return pConnection.newOutputStream();
    }

    /* access modifiers changed from: protected */
    public boolean isContentLengthRequired(XmlRpcStreamRequestConfig pConfig) {
        return false;
    }

    /* JADX WARNING: Code restructure failed: missing block: B:16:0x0043, code lost:
        if (r0 == null) goto L_0x0046;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:7:0x0032, code lost:
        if (0 != 0) goto L_0x0034;
     */
    /* JADX WARNING: Code restructure failed: missing block: B:9:?, code lost:
        r0.close();
     */
    /* JADX WARNING: Removed duplicated region for block: B:38:0x0077 A[SYNTHETIC, Splitter:B:38:0x0077] */
    /* JADX WARNING: Removed duplicated region for block: B:57:0x009d A[SYNTHETIC, Splitter:B:57:0x009d] */
    /* JADX WARNING: Unknown top exception splitter block from list: {B:66:0x00b1=Splitter:B:66:0x00b1, B:52:0x0096=Splitter:B:52:0x0096, B:74:0x00bd=Splitter:B:74:0x00bd} */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public void execute(org.apache.xmlrpc.common.XmlRpcStreamRequestConfig r10, org.apache.xmlrpc.common.ServerStreamConnection r11) throws org.apache.xmlrpc.XmlRpcException {
        /*
            r9 = this;
            org.apache.commons.logging.Log r0 = log
            java.lang.String r1 = "execute: ->"
            r0.debug(r1)
            r0 = 0
            java.io.InputStream r1 = r9.getInputStream(r10, r11)     // Catch:{ Throwable -> 0x003d }
            r0 = r1
            org.apache.xmlrpc.XmlRpcRequest r1 = r9.getRequest(r10, r0)     // Catch:{ Throwable -> 0x003d }
            java.lang.String r2 = r1.getMethodName()     // Catch:{ Throwable -> 0x003d }
            java.lang.String r3 = "system.multicall"
            boolean r2 = r2.equals(r3)     // Catch:{ Throwable -> 0x003d }
            if (r2 == 0) goto L_0x0022
            java.lang.Object[] r2 = r9.executeMulticall(r1)     // Catch:{ Throwable -> 0x003d }
            goto L_0x0026
        L_0x0022:
            java.lang.Object r2 = r9.execute(r1)     // Catch:{ Throwable -> 0x003d }
        L_0x0026:
            r0.close()     // Catch:{ Throwable -> 0x003d }
            r0 = 0
            r3 = 0
            org.apache.commons.logging.Log r4 = log     // Catch:{ Throwable -> 0x003d }
            java.lang.String r5 = "execute: Request performed successfully"
            r4.debug(r5)     // Catch:{ Throwable -> 0x003d }
            if (r0 == 0) goto L_0x0046
        L_0x0034:
            r0.close()     // Catch:{ Throwable -> 0x0038 }
            goto L_0x0046
        L_0x0038:
            r1 = move-exception
            goto L_0x0046
        L_0x003a:
            r1 = move-exception
            goto L_0x00b2
        L_0x003d:
            r1 = move-exception
            r9.logError(r1)     // Catch:{ all -> 0x003a }
            r2 = 0
            r3 = r1
            if (r0 == 0) goto L_0x0046
            goto L_0x0034
        L_0x0046:
            r1 = r2
            r2 = r3
            boolean r3 = r9.isContentLengthRequired(r10)     // Catch:{ IOException -> 0x00ba }
            if (r3 == 0) goto L_0x0055
            java.io.ByteArrayOutputStream r4 = new java.io.ByteArrayOutputStream     // Catch:{ IOException -> 0x00ba }
            r4.<init>()     // Catch:{ IOException -> 0x00ba }
            r5 = r4
            goto L_0x005a
        L_0x0055:
            r4 = 0
            java.io.OutputStream r5 = r11.newOutputStream()     // Catch:{ IOException -> 0x00ba }
        L_0x005a:
            java.io.OutputStream r6 = r9.getOutputStream((org.apache.xmlrpc.common.ServerStreamConnection) r11, (org.apache.xmlrpc.common.XmlRpcStreamRequestConfig) r10, (java.io.OutputStream) r5)     // Catch:{ IOException -> 0x00ba }
            r5 = r6
            if (r2 != 0) goto L_0x0067
            r9.writeResponse(r10, r5, r1)     // Catch:{ all -> 0x0065 }
            goto L_0x006a
        L_0x0065:
            r6 = move-exception
            goto L_0x00aa
        L_0x0067:
            r9.writeError(r10, r5, r2)     // Catch:{ all -> 0x0065 }
        L_0x006a:
            r5.close()     // Catch:{ all -> 0x0065 }
            r5 = 0
            if (r5 == 0) goto L_0x0075
            r5.close()     // Catch:{ Throwable -> 0x0074 }
            goto L_0x0075
        L_0x0074:
            r6 = move-exception
        L_0x0075:
            if (r4 == 0) goto L_0x0097
            int r6 = r4.size()     // Catch:{ IOException -> 0x00ba }
            java.io.OutputStream r6 = r9.getOutputStream((org.apache.xmlrpc.common.XmlRpcStreamRequestConfig) r10, (org.apache.xmlrpc.common.ServerStreamConnection) r11, (int) r6)     // Catch:{ IOException -> 0x00ba }
            r4.writeTo(r6)     // Catch:{ all -> 0x008e }
            r6.close()     // Catch:{ all -> 0x008e }
            r6 = 0
            if (r6 == 0) goto L_0x0097
            r6.close()     // Catch:{ Throwable -> 0x008c }
            goto L_0x0097
        L_0x008c:
            r7 = move-exception
            goto L_0x0097
        L_0x008e:
            r7 = move-exception
            if (r6 == 0) goto L_0x0096
            r6.close()     // Catch:{ Throwable -> 0x0095 }
            goto L_0x0096
        L_0x0095:
            r8 = move-exception
        L_0x0096:
            throw r7     // Catch:{ IOException -> 0x00ba }
        L_0x0097:
            r11.close()     // Catch:{ IOException -> 0x00ba }
            r11 = 0
            if (r11 == 0) goto L_0x00a2
            r11.close()     // Catch:{ Throwable -> 0x00a1 }
            goto L_0x00a2
        L_0x00a1:
            r0 = move-exception
        L_0x00a2:
            org.apache.commons.logging.Log r0 = log
            java.lang.String r1 = "execute: <-"
            r0.debug(r1)
            return
        L_0x00aa:
            if (r5 == 0) goto L_0x00b1
            r5.close()     // Catch:{ Throwable -> 0x00b0 }
            goto L_0x00b1
        L_0x00b0:
            r7 = move-exception
        L_0x00b1:
            throw r6     // Catch:{ IOException -> 0x00ba }
        L_0x00b2:
            if (r0 == 0) goto L_0x00bd
            r0.close()     // Catch:{ Throwable -> 0x00bc }
            goto L_0x00bd
        L_0x00b8:
            r0 = move-exception
            goto L_0x00da
        L_0x00ba:
            r0 = move-exception
            goto L_0x00be
        L_0x00bc:
            r2 = move-exception
        L_0x00bd:
            throw r1     // Catch:{ IOException -> 0x00ba }
        L_0x00be:
            org.apache.xmlrpc.XmlRpcException r1 = new org.apache.xmlrpc.XmlRpcException     // Catch:{ all -> 0x00b8 }
            java.lang.StringBuilder r2 = new java.lang.StringBuilder     // Catch:{ all -> 0x00b8 }
            r2.<init>()     // Catch:{ all -> 0x00b8 }
            java.lang.String r3 = "I/O error while processing request: "
            r2.append(r3)     // Catch:{ all -> 0x00b8 }
            java.lang.String r3 = r0.getMessage()     // Catch:{ all -> 0x00b8 }
            r2.append(r3)     // Catch:{ all -> 0x00b8 }
            java.lang.String r2 = r2.toString()     // Catch:{ all -> 0x00b8 }
            r1.<init>((java.lang.String) r2, (java.lang.Throwable) r0)     // Catch:{ all -> 0x00b8 }
            throw r1     // Catch:{ all -> 0x00b8 }
        L_0x00da:
            if (r11 == 0) goto L_0x00e1
            r11.close()     // Catch:{ Throwable -> 0x00e0 }
            goto L_0x00e1
        L_0x00e0:
            r1 = move-exception
        L_0x00e1:
            throw r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.apache.xmlrpc.server.XmlRpcStreamServer.execute(org.apache.xmlrpc.common.XmlRpcStreamRequestConfig, org.apache.xmlrpc.common.ServerStreamConnection):void");
    }

    private Object[] executeMulticall(XmlRpcRequest pRequest) {
        Object result;
        if (pRequest.getParameterCount() != 1) {
            return null;
        }
        Object[] reqs = (Object[]) pRequest.getParameter(0);
        ArrayList<Object> results = new ArrayList<>();
        final XmlRpcRequestConfig pConfig = pRequest.getConfig();
        for (int i = 0; i < reqs.length; i++) {
            try {
                HashMap<String, Object> req = (HashMap) reqs[i];
                final String methodName = (String) req.get("methodName");
                final Object[] params = (Object[]) req.get("params");
                result = execute(new XmlRpcRequest() {
                    public XmlRpcRequestConfig getConfig() {
                        return pConfig;
                    }

                    public String getMethodName() {
                        return methodName;
                    }

                    public int getParameterCount() {
                        if (params == null) {
                            return 0;
                        }
                        return params.length;
                    }

                    public Object getParameter(int pIndex) {
                        return params[pIndex];
                    }
                });
            } catch (Throwable t) {
                logError(t);
                result = null;
            }
            results.add(result);
        }
        return new Object[]{results};
    }

    /* access modifiers changed from: protected */
    public void logError(Throwable t) {
        this.errorLogger.log(t.getMessage() == null ? t.getClass().getName() : t.getMessage(), t);
    }

    public XmlRpcErrorLogger getErrorLogger() {
        return this.errorLogger;
    }

    public void setErrorLogger(XmlRpcErrorLogger pErrorLogger) {
        this.errorLogger = pErrorLogger;
    }
}
