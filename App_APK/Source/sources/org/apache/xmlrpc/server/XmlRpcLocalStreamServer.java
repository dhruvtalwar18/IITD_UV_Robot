package org.apache.xmlrpc.server;

import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.common.XmlRpcRequestProcessorFactory;

public class XmlRpcLocalStreamServer extends XmlRpcStreamServer {
    public Object execute(XmlRpcRequest pRequest) throws XmlRpcException {
        return ((XmlRpcRequestProcessorFactory) pRequest.getConfig()).getXmlRpcServer().execute(pRequest);
    }
}
