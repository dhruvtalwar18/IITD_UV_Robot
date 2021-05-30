package org.apache.xmlrpc.server;

import java.io.IOException;
import java.net.URL;
import java.util.Iterator;
import java.util.Map;
import java.util.Properties;
import org.apache.xmlrpc.XmlRpcException;

public class PropertyHandlerMapping extends AbstractReflectiveHandlerMapping {
    public void load(ClassLoader pClassLoader, String pResource) throws IOException, XmlRpcException {
        URL url = pClassLoader.getResource(pResource);
        if (url != null) {
            load(pClassLoader, url);
            return;
        }
        throw new IOException("Unable to locate resource " + pResource);
    }

    public void load(ClassLoader pClassLoader, URL pURL) throws IOException, XmlRpcException {
        Properties props = new Properties();
        props.load(pURL.openStream());
        load(pClassLoader, (Map) props);
    }

    public void load(ClassLoader pClassLoader, Map pMap) throws XmlRpcException {
        for (Map.Entry entry : pMap.entrySet()) {
            registerPublicMethods((String) entry.getKey(), newHandlerClass(pClassLoader, (String) entry.getValue()));
        }
    }

    /* access modifiers changed from: protected */
    public Class newHandlerClass(ClassLoader pClassLoader, String pClassName) throws XmlRpcException {
        try {
            Class c = pClassLoader.loadClass(pClassName);
            if (c != null) {
                return c;
            }
            throw new XmlRpcException(0, "Loading class " + pClassName + " returned null.");
        } catch (ClassNotFoundException e) {
            throw new XmlRpcException("Unable to load class: " + pClassName, (Throwable) e);
        }
    }

    public void addHandler(String pKey, Class pClass) throws XmlRpcException {
        registerPublicMethods(pKey, pClass);
    }

    public void removeHandler(String pKey) {
        Iterator i = this.handlerMap.keySet().iterator();
        while (i.hasNext()) {
            if (((String) i.next()).startsWith(pKey)) {
                i.remove();
            }
        }
    }
}
