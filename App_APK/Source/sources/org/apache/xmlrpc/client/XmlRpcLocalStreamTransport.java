package org.apache.xmlrpc.client;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.client.XmlRpcStreamTransport;
import org.apache.xmlrpc.common.LocalStreamConnection;
import org.apache.xmlrpc.common.XmlRpcStreamRequestConfig;
import org.apache.xmlrpc.common.XmlRpcStreamRequestProcessor;
import org.xml.sax.SAXException;

public class XmlRpcLocalStreamTransport extends XmlRpcStreamTransport {
    private LocalStreamConnection conn;
    private final XmlRpcStreamRequestProcessor localServer;
    private XmlRpcRequest request;

    public XmlRpcLocalStreamTransport(XmlRpcClient pClient, XmlRpcStreamRequestProcessor pServer) {
        super(pClient);
        this.localServer = pServer;
    }

    /* access modifiers changed from: protected */
    public boolean isResponseGzipCompressed(XmlRpcStreamRequestConfig pConfig) {
        return pConfig.isGzipRequesting();
    }

    /* access modifiers changed from: protected */
    public void close() throws XmlRpcClientException {
    }

    /* access modifiers changed from: protected */
    public InputStream getInputStream() throws XmlRpcException {
        this.localServer.execute(this.conn.getConfig(), this.conn.getServerStreamConnection());
        return new ByteArrayInputStream(this.conn.getResponse().toByteArray());
    }

    /* access modifiers changed from: protected */
    public XmlRpcStreamTransport.ReqWriter newReqWriter(XmlRpcRequest pRequest) throws XmlRpcException, IOException, SAXException {
        this.request = pRequest;
        return super.newReqWriter(pRequest);
    }

    /* access modifiers changed from: protected */
    public void writeRequest(XmlRpcStreamTransport.ReqWriter pWriter) throws XmlRpcException, IOException, SAXException {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        pWriter.write(baos);
        this.conn = new LocalStreamConnection((XmlRpcStreamRequestConfig) this.request.getConfig(), new ByteArrayInputStream(baos.toByteArray()));
    }
}
