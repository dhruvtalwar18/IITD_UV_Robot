package org.ros.internal.node.client;

import java.net.MalformedURLException;
import java.net.URI;
import org.apache.commons.lang.time.DateUtils;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.client.XmlRpcClientConfigImpl;
import org.apache.xmlrpc.client.XmlRpcCommonsTransportFactory;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.node.xmlrpc.XmlRpcClientFactory;
import org.ros.internal.node.xmlrpc.XmlRpcEndpoint;

abstract class Client<T extends XmlRpcEndpoint> {
    private static final int CONNECTION_TIMEOUT = 60000;
    private static final int REPLY_TIMEOUT = 60000;
    private static final int XMLRPC_TIMEOUT = 10000;
    private final URI uri;
    protected final T xmlRpcEndpoint;

    public Client(URI uri2, Class<T> interfaceClass) {
        this.uri = uri2;
        XmlRpcClientConfigImpl config = new XmlRpcClientConfigImpl();
        try {
            config.setServerURL(uri2.toURL());
            config.setConnectionTimeout(DateUtils.MILLIS_IN_MINUTE);
            config.setReplyTimeout(DateUtils.MILLIS_IN_MINUTE);
            XmlRpcClient client = new XmlRpcClient();
            client.setTransportFactory(new XmlRpcCommonsTransportFactory(client));
            client.setConfig(config);
            this.xmlRpcEndpoint = (XmlRpcEndpoint) interfaceClass.cast(new XmlRpcClientFactory<>(client).newInstance(getClass().getClassLoader(), interfaceClass, "", 10000));
        } catch (MalformedURLException e) {
            throw new RosRuntimeException((Throwable) e);
        }
    }

    public URI getRemoteUri() {
        return this.uri;
    }
}
