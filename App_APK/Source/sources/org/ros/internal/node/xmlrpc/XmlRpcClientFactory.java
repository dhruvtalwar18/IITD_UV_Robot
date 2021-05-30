package org.ros.internal.node.xmlrpc;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.lang.reflect.UndeclaredThrowableException;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.client.AsyncCallback;
import org.apache.xmlrpc.client.TimingOutCallback;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.common.TypeConverterFactory;
import org.apache.xmlrpc.common.TypeConverterFactoryImpl;
import org.ros.internal.node.xmlrpc.XmlRpcEndpoint;

public class XmlRpcClientFactory<T extends XmlRpcEndpoint> {
    /* access modifiers changed from: private */
    public final XmlRpcClient client;
    private boolean objectMethodLocal;
    /* access modifiers changed from: private */
    public final TypeConverterFactory typeConverterFactory;

    public XmlRpcClientFactory(XmlRpcClient pClient, TypeConverterFactory pTypeConverterFactory) {
        this.typeConverterFactory = pTypeConverterFactory;
        this.client = pClient;
    }

    public XmlRpcClientFactory(XmlRpcClient pClient) {
        this(pClient, new TypeConverterFactoryImpl());
    }

    public XmlRpcClient getClient() {
        return this.client;
    }

    public boolean isObjectMethodLocal() {
        return this.objectMethodLocal;
    }

    public void setObjectMethodLocal(boolean pObjectMethodLocal) {
        this.objectMethodLocal = pObjectMethodLocal;
    }

    public Object newInstance(ClassLoader pClassLoader, Class<T> pClass, final String pRemoteName, final int timeout) {
        return Proxy.newProxyInstance(pClassLoader, new Class[]{pClass}, new InvocationHandler() {
            public Object invoke(Object pProxy, Method pMethod, Object[] pArgs) throws Throwable {
                String methodName;
                if (XmlRpcClientFactory.this.isObjectMethodLocal() && pMethod.getDeclaringClass().equals(Object.class)) {
                    return pMethod.invoke(pProxy, pArgs);
                }
                if (pRemoteName == null || pRemoteName.length() == 0) {
                    methodName = pMethod.getName();
                } else {
                    methodName = pRemoteName + "." + pMethod.getName();
                }
                try {
                    TimingOutCallback callback = new TimingOutCallback((long) timeout);
                    XmlRpcClientFactory.this.client.executeAsync(methodName, pArgs, (AsyncCallback) callback);
                    return XmlRpcClientFactory.this.typeConverterFactory.getTypeConverter(pMethod.getReturnType()).convert(callback.waitForResponse());
                } catch (TimingOutCallback.TimeoutException e) {
                    throw new XmlRpcTimeoutException(e);
                } catch (InterruptedException e2) {
                    throw new XmlRpcTimeoutException(e2);
                } catch (UndeclaredThrowableException e3) {
                    throw new RuntimeException(e3);
                } catch (XmlRpcException e4) {
                    Throwable linkedException = e4.linkedException;
                    if (linkedException != null) {
                        Class<?>[] exceptionTypes = pMethod.getExceptionTypes();
                        int i = 0;
                        while (i < exceptionTypes.length) {
                            if (!exceptionTypes[i].isAssignableFrom(linkedException.getClass())) {
                                i++;
                            } else {
                                throw linkedException;
                            }
                        }
                        throw new RuntimeException(linkedException);
                    }
                    throw new RuntimeException(e4);
                }
            }
        });
    }
}
