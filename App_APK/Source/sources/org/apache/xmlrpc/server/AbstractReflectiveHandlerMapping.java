package org.apache.xmlrpc.server;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcHandler;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.common.TypeConverterFactory;
import org.apache.xmlrpc.common.TypeConverterFactoryImpl;
import org.apache.xmlrpc.metadata.ReflectiveXmlRpcMetaDataHandler;
import org.apache.xmlrpc.metadata.Util;
import org.apache.xmlrpc.metadata.XmlRpcListableHandlerMapping;
import org.apache.xmlrpc.metadata.XmlRpcMetaDataHandler;
import org.apache.xmlrpc.server.RequestProcessorFactoryFactory;

public abstract class AbstractReflectiveHandlerMapping implements XmlRpcListableHandlerMapping {
    private AuthenticationHandler authenticationHandler;
    protected Map handlerMap = new HashMap();
    private RequestProcessorFactoryFactory requestProcessorFactoryFactory = new RequestProcessorFactoryFactory.RequestSpecificProcessorFactoryFactory();
    private TypeConverterFactory typeConverterFactory = new TypeConverterFactoryImpl();
    private boolean voidMethodEnabled;

    public interface AuthenticationHandler {
        boolean isAuthorized(XmlRpcRequest xmlRpcRequest) throws XmlRpcException;
    }

    public void setTypeConverterFactory(TypeConverterFactory pFactory) {
        this.typeConverterFactory = pFactory;
    }

    public TypeConverterFactory getTypeConverterFactory() {
        return this.typeConverterFactory;
    }

    public void setRequestProcessorFactoryFactory(RequestProcessorFactoryFactory pFactory) {
        this.requestProcessorFactoryFactory = pFactory;
    }

    public RequestProcessorFactoryFactory getRequestProcessorFactoryFactory() {
        return this.requestProcessorFactoryFactory;
    }

    public AuthenticationHandler getAuthenticationHandler() {
        return this.authenticationHandler;
    }

    public void setAuthenticationHandler(AuthenticationHandler pAuthenticationHandler) {
        this.authenticationHandler = pAuthenticationHandler;
    }

    /* access modifiers changed from: protected */
    public boolean isHandlerMethod(Method pMethod) {
        if (!Modifier.isPublic(pMethod.getModifiers()) || Modifier.isStatic(pMethod.getModifiers())) {
            return false;
        }
        if ((isVoidMethodEnabled() || pMethod.getReturnType() != Void.TYPE) && pMethod.getDeclaringClass() != Object.class) {
            return true;
        }
        return false;
    }

    /* access modifiers changed from: protected */
    public void registerPublicMethods(String pKey, Class pType) throws XmlRpcException {
        String str;
        Method[] mArray;
        Map map = new HashMap();
        Method[] methods = pType.getMethods();
        for (Method method : methods) {
            if (isHandlerMethod(method)) {
                StringBuilder sb = new StringBuilder();
                if (pKey.length() > 0) {
                    str = pKey + ".";
                } else {
                    str = "";
                }
                sb.append(str);
                sb.append(method.getName());
                String name = sb.toString();
                Method[] oldMArray = (Method[]) map.get(name);
                if (oldMArray == null) {
                    mArray = new Method[]{method};
                } else {
                    mArray = new Method[(oldMArray.length + 1)];
                    System.arraycopy(oldMArray, 0, mArray, 0, oldMArray.length);
                    mArray[oldMArray.length] = method;
                }
                map.put(name, mArray);
            }
        }
        for (Map.Entry entry : map.entrySet()) {
            this.handlerMap.put((String) entry.getKey(), newXmlRpcHandler(pType, (Method[]) entry.getValue()));
        }
    }

    /* access modifiers changed from: protected */
    public XmlRpcHandler newXmlRpcHandler(Class pClass, Method[] pMethods) throws XmlRpcException {
        String[][] sig = getSignature(pMethods);
        String help = getMethodHelp(pClass, pMethods);
        RequestProcessorFactoryFactory.RequestProcessorFactory factory = this.requestProcessorFactoryFactory.getRequestProcessorFactory(pClass);
        if (sig == null || help == null) {
            return new ReflectiveXmlRpcHandler(this, this.typeConverterFactory, pClass, factory, pMethods);
        }
        return new ReflectiveXmlRpcMetaDataHandler(this, this.typeConverterFactory, pClass, factory, pMethods, sig, help);
    }

    /* access modifiers changed from: protected */
    public String[][] getSignature(Method[] pMethods) {
        return Util.getSignature(pMethods);
    }

    /* access modifiers changed from: protected */
    public String getMethodHelp(Class pClass, Method[] pMethods) {
        return Util.getMethodHelp(pClass, pMethods);
    }

    public XmlRpcHandler getHandler(String pHandlerName) throws XmlRpcNoSuchHandlerException, XmlRpcException {
        XmlRpcHandler result = (XmlRpcHandler) this.handlerMap.get(pHandlerName);
        if (result != null) {
            return result;
        }
        throw new XmlRpcNoSuchHandlerException("No such handler: " + pHandlerName);
    }

    public String[] getListMethods() throws XmlRpcException {
        List list = new ArrayList();
        for (Map.Entry entry : this.handlerMap.entrySet()) {
            if (entry.getValue() instanceof XmlRpcMetaDataHandler) {
                list.add(entry.getKey());
            }
        }
        return (String[]) list.toArray(new String[list.size()]);
    }

    public String getMethodHelp(String pHandlerName) throws XmlRpcException {
        XmlRpcHandler h = getHandler(pHandlerName);
        if (h instanceof XmlRpcMetaDataHandler) {
            return ((XmlRpcMetaDataHandler) h).getMethodHelp();
        }
        throw new XmlRpcNoSuchHandlerException("No help available for method: " + pHandlerName);
    }

    public String[][] getMethodSignature(String pHandlerName) throws XmlRpcException {
        XmlRpcHandler h = getHandler(pHandlerName);
        if (h instanceof XmlRpcMetaDataHandler) {
            return ((XmlRpcMetaDataHandler) h).getSignatures();
        }
        throw new XmlRpcNoSuchHandlerException("No metadata available for method: " + pHandlerName);
    }

    public boolean isVoidMethodEnabled() {
        return this.voidMethodEnabled;
    }

    public void setVoidMethodEnabled(boolean pVoidMethodEnabled) {
        this.voidMethodEnabled = pVoidMethodEnabled;
    }
}
