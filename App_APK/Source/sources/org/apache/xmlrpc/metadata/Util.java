package org.apache.xmlrpc.metadata;

import java.io.Serializable;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Map;
import org.apache.xmlrpc.XmlRpcException;
import org.apache.xmlrpc.serializer.BooleanSerializer;
import org.apache.xmlrpc.serializer.DoubleSerializer;
import org.apache.xmlrpc.serializer.FloatSerializer;
import org.apache.xmlrpc.serializer.I1Serializer;
import org.apache.xmlrpc.serializer.I2Serializer;
import org.apache.xmlrpc.serializer.I4Serializer;
import org.apache.xmlrpc.serializer.I8Serializer;
import org.apache.xmlrpc.serializer.MapSerializer;
import org.apache.xmlrpc.serializer.NullSerializer;
import org.apache.xmlrpc.serializer.ObjectArraySerializer;
import org.apache.xmlrpc.serializer.StringSerializer;
import org.w3c.dom.Node;

public class Util {
    private static final Class jaxbElementClass;

    static {
        Class c;
        try {
            c = Class.forName("javax.xml.bind.Element");
        } catch (ClassNotFoundException e) {
            c = null;
        }
        jaxbElementClass = c;
    }

    public static String getSignatureType(Class pType) {
        if (pType == Integer.TYPE || pType == Integer.class) {
            return I4Serializer.INT_TAG;
        }
        if (pType == Double.TYPE || pType == Double.class) {
            return DoubleSerializer.DOUBLE_TAG;
        }
        if (pType == Boolean.TYPE || pType == Boolean.class) {
            return BooleanSerializer.BOOLEAN_TAG;
        }
        if (pType == String.class) {
            return StringSerializer.STRING_TAG;
        }
        if (Object[].class.isAssignableFrom(pType) || List.class.isAssignableFrom(pType)) {
            return ObjectArraySerializer.ARRAY_TAG;
        }
        if (Map.class.isAssignableFrom(pType)) {
            return MapSerializer.STRUCT_TAG;
        }
        if (Date.class.isAssignableFrom(pType) || Calendar.class.isAssignableFrom(pType)) {
            return "dateTime.iso8601";
        }
        if (pType == byte[].class) {
            return "base64";
        }
        if (pType == Void.TYPE) {
            return NullSerializer.EX_NIL_TAG;
        }
        if (pType == Byte.TYPE || pType == Byte.class) {
            return I1Serializer.EX_I1_TAG;
        }
        if (pType == Short.TYPE || pType == Short.class) {
            return I2Serializer.EX_I2_TAG;
        }
        if (pType == Long.TYPE || pType == Long.class) {
            return I8Serializer.EX_I8_TAG;
        }
        if (pType == Float.TYPE || pType == Float.class) {
            return FloatSerializer.EX_FLOAT_TAG;
        }
        if (Node.class.isAssignableFrom(pType)) {
            return "ex:node";
        }
        if (jaxbElementClass != null && jaxbElementClass.isAssignableFrom(pType)) {
            return "ex:jaxbElement";
        }
        if (Serializable.class.isAssignableFrom(pType)) {
            return "base64";
        }
        return null;
    }

    public static String[][] getSignature(Method[] pMethods) {
        List result = new ArrayList();
        for (Method signature : pMethods) {
            String[] sig = getSignature(signature);
            if (sig != null) {
                result.add(sig);
            }
        }
        return (String[][]) result.toArray(new String[result.size()][]);
    }

    public static String[] getSignature(Method pMethod) {
        Class[] paramClasses = pMethod.getParameterTypes();
        String[] sig = new String[(paramClasses.length + 1)];
        String s = getSignatureType(pMethod.getReturnType());
        if (s == null) {
            return null;
        }
        sig[0] = s;
        for (int i = 0; i < paramClasses.length; i++) {
            String s2 = getSignatureType(paramClasses[i]);
            if (s2 == null) {
                return null;
            }
            sig[i + 1] = s2;
        }
        return sig;
    }

    public static String getMethodHelp(Class pClass, Method[] pMethods) {
        List result = new ArrayList();
        for (Method methodHelp : pMethods) {
            String help = getMethodHelp(pClass, methodHelp);
            if (help != null) {
                result.add(help);
            }
        }
        switch (result.size()) {
            case 0:
                return null;
            case 1:
                return (String) result.get(0);
            default:
                StringBuffer sb = new StringBuffer();
                for (int i = 0; i < result.size(); i++) {
                    sb.append(i + 1);
                    sb.append(": ");
                    sb.append(result.get(i));
                    sb.append("\n");
                }
                return sb.toString();
        }
    }

    public static String getMethodHelp(Class pClass, Method pMethod) {
        StringBuffer sb = new StringBuffer();
        sb.append("Invokes the method ");
        sb.append(pClass.getName());
        sb.append(".");
        sb.append(pMethod.getName());
        sb.append("(");
        Class[] paramClasses = pMethod.getParameterTypes();
        for (int i = 0; i < paramClasses.length; i++) {
            if (i > 0) {
                sb.append(", ");
            }
            sb.append(paramClasses[i].getName());
        }
        sb.append(").");
        return sb.toString();
    }

    public static String getSignature(Object[] args) {
        StringBuffer sb = new StringBuffer();
        for (int i = 0; i < args.length; i++) {
            if (i > 0) {
                sb.append(", ");
            }
            if (args[i] == null) {
                sb.append("null");
            } else {
                sb.append(args[i].getClass().getName());
            }
        }
        return sb.toString();
    }

    public static Object newInstance(Class pClass) throws XmlRpcException {
        try {
            return pClass.newInstance();
        } catch (InstantiationException e) {
            throw new XmlRpcException("Failed to instantiate class " + pClass.getName(), (Throwable) e);
        } catch (IllegalAccessException e2) {
            throw new XmlRpcException("Illegal access when instantiating class " + pClass.getName(), (Throwable) e2);
        }
    }
}
