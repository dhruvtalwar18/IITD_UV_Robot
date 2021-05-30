package org.apache.xmlrpc.util;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

public class ReflectionUtil {
    public static boolean setProperty(Object pObject, String pPropertyName, String pPropertyValue) throws IllegalAccessException, InvocationTargetException {
        Object param;
        String methodName = "set" + pPropertyName.substring(0, 1).toUpperCase() + pPropertyName.substring(1);
        Method[] methods = pObject.getClass().getMethods();
        for (Method method : methods) {
            if (method.getName().equals(methodName) && Modifier.isPublic(method.getModifiers())) {
                Class[] parameterTypes = method.getParameterTypes();
                if (parameterTypes.length == 1) {
                    Class parameterType = parameterTypes[0];
                    try {
                        if (!parameterType.equals(Boolean.TYPE)) {
                            if (!parameterType.equals(Boolean.class)) {
                                if (!parameterType.equals(Character.TYPE)) {
                                    if (!parameterType.equals(Character.class)) {
                                        if (!parameterType.equals(Byte.TYPE)) {
                                            if (!parameterType.equals(Byte.class)) {
                                                if (!parameterType.equals(Short.TYPE)) {
                                                    if (!parameterType.equals(Short.class)) {
                                                        if (!parameterType.equals(Integer.TYPE)) {
                                                            if (!parameterType.equals(Integer.class)) {
                                                                if (!parameterType.equals(Long.TYPE)) {
                                                                    if (!parameterType.equals(Long.class)) {
                                                                        if (!parameterType.equals(Float.TYPE)) {
                                                                            if (!parameterType.equals(Float.class)) {
                                                                                if (!parameterType.equals(Double.TYPE)) {
                                                                                    if (!parameterType.equals(Double.class)) {
                                                                                        if (parameterType.equals(String.class)) {
                                                                                            param = pPropertyValue;
                                                                                            method.invoke(pObject, new Object[]{param});
                                                                                            return true;
                                                                                        }
                                                                                        throw new IllegalStateException("The property " + pPropertyName + " has an unsupported type of " + parameterType.getName());
                                                                                    }
                                                                                }
                                                                                param = Double.valueOf(pPropertyValue);
                                                                                method.invoke(pObject, new Object[]{param});
                                                                                return true;
                                                                            }
                                                                        }
                                                                        param = Float.valueOf(pPropertyValue);
                                                                        method.invoke(pObject, new Object[]{param});
                                                                        return true;
                                                                    }
                                                                }
                                                                param = Long.valueOf(pPropertyValue);
                                                                method.invoke(pObject, new Object[]{param});
                                                                return true;
                                                            }
                                                        }
                                                        param = Integer.valueOf(pPropertyValue);
                                                        method.invoke(pObject, new Object[]{param});
                                                        return true;
                                                    }
                                                }
                                                param = Short.valueOf(pPropertyValue);
                                                method.invoke(pObject, new Object[]{param});
                                                return true;
                                            }
                                        }
                                        param = Byte.valueOf(pPropertyValue);
                                        method.invoke(pObject, new Object[]{param});
                                        return true;
                                    }
                                }
                                if (pPropertyValue.length() == 1) {
                                    param = new Character(pPropertyValue.charAt(0));
                                    method.invoke(pObject, new Object[]{param});
                                    return true;
                                }
                                throw new IllegalArgumentException("Invalid value for parameter " + pPropertyName + "(length != 1):" + pPropertyValue);
                            }
                        }
                        param = Boolean.valueOf(pPropertyValue);
                        method.invoke(pObject, new Object[]{param});
                        return true;
                    } catch (NumberFormatException e) {
                        throw new IllegalArgumentException("Invalid value for property " + pPropertyName + ": " + pPropertyValue);
                    }
                }
            }
        }
        return false;
    }
}
