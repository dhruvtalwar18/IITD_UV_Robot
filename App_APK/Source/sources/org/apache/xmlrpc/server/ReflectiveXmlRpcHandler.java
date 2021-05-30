package org.apache.xmlrpc.server;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.XmlRpcHandler;
import org.apache.xmlrpc.XmlRpcRequest;
import org.apache.xmlrpc.common.TypeConverter;
import org.apache.xmlrpc.common.TypeConverterFactory;
import org.apache.xmlrpc.common.XmlRpcInvocationException;
import org.apache.xmlrpc.common.XmlRpcNotAuthorizedException;
import org.apache.xmlrpc.metadata.Util;
import org.apache.xmlrpc.server.AbstractReflectiveHandlerMapping;
import org.apache.xmlrpc.server.RequestProcessorFactoryFactory;

public class ReflectiveXmlRpcHandler implements XmlRpcHandler {
    private final Class clazz;
    private final AbstractReflectiveHandlerMapping mapping;
    private final MethodData[] methods;
    private final RequestProcessorFactoryFactory.RequestProcessorFactory requestProcessorFactory;

    private static class MethodData {
        final Method method;
        final TypeConverter[] typeConverters;

        MethodData(Method pMethod, TypeConverterFactory pTypeConverterFactory) {
            this.method = pMethod;
            Class[] paramClasses = this.method.getParameterTypes();
            this.typeConverters = new TypeConverter[paramClasses.length];
            for (int i = 0; i < paramClasses.length; i++) {
                this.typeConverters[i] = pTypeConverterFactory.getTypeConverter(paramClasses[i]);
            }
        }
    }

    public ReflectiveXmlRpcHandler(AbstractReflectiveHandlerMapping pMapping, TypeConverterFactory pTypeConverterFactory, Class pClass, RequestProcessorFactoryFactory.RequestProcessorFactory pFactory, Method[] pMethods) {
        this.mapping = pMapping;
        this.clazz = pClass;
        this.methods = new MethodData[pMethods.length];
        this.requestProcessorFactory = pFactory;
        for (int i = 0; i < this.methods.length; i++) {
            this.methods[i] = new MethodData(pMethods[i], pTypeConverterFactory);
        }
    }

    private Object getInstance(XmlRpcRequest pRequest) throws XmlRpcException {
        return this.requestProcessorFactory.getRequestProcessor(pRequest);
    }

    public Object execute(XmlRpcRequest pRequest) throws XmlRpcException {
        AbstractReflectiveHandlerMapping.AuthenticationHandler authHandler = this.mapping.getAuthenticationHandler();
        if (authHandler == null || authHandler.isAuthorized(pRequest)) {
            Object[] args = new Object[pRequest.getParameterCount()];
            for (int j = 0; j < args.length; j++) {
                args[j] = pRequest.getParameter(j);
            }
            Object instance = getInstance(pRequest);
            for (MethodData methodData : this.methods) {
                TypeConverter[] converters = methodData.typeConverters;
                if (args.length == converters.length) {
                    boolean matching = true;
                    int j2 = 0;
                    while (true) {
                        if (j2 >= args.length) {
                            break;
                        } else if (!converters[j2].isConvertable(args[j2])) {
                            matching = false;
                            break;
                        } else {
                            j2++;
                        }
                    }
                    if (matching) {
                        for (int j3 = 0; j3 < args.length; j3++) {
                            args[j3] = converters[j3].convert(args[j3]);
                        }
                        return invoke(instance, methodData.method, args);
                    }
                }
            }
            throw new XmlRpcException("No method " + pRequest.getMethodName() + " matching arguments: " + Util.getSignature(args));
        }
        throw new XmlRpcNotAuthorizedException("Not authorized");
    }

    private Object invoke(Object pInstance, Method pMethod, Object[] pArgs) throws XmlRpcException {
        try {
            return pMethod.invoke(pInstance, pArgs);
        } catch (IllegalAccessException e) {
            throw new XmlRpcException("Illegal access to method " + pMethod.getName() + " in class " + this.clazz.getName(), (Throwable) e);
        } catch (IllegalArgumentException e2) {
            throw new XmlRpcException("Illegal argument for method " + pMethod.getName() + " in class " + this.clazz.getName(), (Throwable) e2);
        } catch (InvocationTargetException e3) {
            Throwable t = e3.getTargetException();
            if (t instanceof XmlRpcException) {
                throw ((XmlRpcException) t);
            }
            throw new XmlRpcInvocationException("Failed to invoke method " + pMethod.getName() + " in class " + this.clazz.getName() + ": " + t.getMessage(), t);
        }
    }
}
