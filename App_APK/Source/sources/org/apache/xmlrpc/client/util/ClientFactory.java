package org.apache.xmlrpc.client.util;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.lang.reflect.UndeclaredThrowableException;
import org.apache.xmlrpc.client.XmlRpcClient;
import org.apache.xmlrpc.common.TypeConverterFactory;
import org.apache.xmlrpc.common.TypeConverterFactoryImpl;
import org.apache.xmlrpc.common.XmlRpcInvocationException;

public class ClientFactory {
    /* access modifiers changed from: private */
    public final XmlRpcClient client;
    private boolean objectMethodLocal;
    /* access modifiers changed from: private */
    public final TypeConverterFactory typeConverterFactory;

    public ClientFactory(XmlRpcClient pClient, TypeConverterFactory pTypeConverterFactory) {
        this.typeConverterFactory = pTypeConverterFactory;
        this.client = pClient;
    }

    public ClientFactory(XmlRpcClient pClient) {
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

    public Object newInstance(Class pClass) {
        return newInstance(Thread.currentThread().getContextClassLoader(), pClass);
    }

    public Object newInstance(ClassLoader pClassLoader, Class pClass) {
        return newInstance(pClassLoader, pClass, pClass.getName());
    }

    public Object newInstance(ClassLoader pClassLoader, Class pClass, final String pRemoteName) {
        return Proxy.newProxyInstance(pClassLoader, new Class[]{pClass}, new InvocationHandler() {
            public Object invoke(Object pProxy, Method pMethod, Object[] pArgs) throws Throwable {
                String methodName;
                if (ClientFactory.this.isObjectMethodLocal() && pMethod.getDeclaringClass().equals(Object.class)) {
                    return pMethod.invoke(pProxy, pArgs);
                }
                if (pRemoteName == null || pRemoteName.length() == 0) {
                    methodName = pMethod.getName();
                } else {
                    methodName = pRemoteName + "." + pMethod.getName();
                }
                try {
                    return ClientFactory.this.typeConverterFactory.getTypeConverter(pMethod.getReturnType()).convert(ClientFactory.this.client.execute(methodName, pArgs));
                } catch (XmlRpcInvocationException e) {
                    Throwable t = e.linkedException;
                    if (!(t instanceof RuntimeException)) {
                        Class[] exceptionTypes = pMethod.getExceptionTypes();
                        int i = 0;
                        while (i < exceptionTypes.length) {
                            if (!exceptionTypes[i].isAssignableFrom(t.getClass())) {
                                i++;
                            } else {
                                throw t;
                            }
                        }
                        throw new UndeclaredThrowableException(t);
                    }
                    throw t;
                }
            }
        });
    }
}
