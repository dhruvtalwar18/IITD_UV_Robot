package org.bytedeco.javacpp.tools;

import java.lang.annotation.Annotation;
import java.lang.reflect.Method;

public class MethodInformation {
    boolean allocator;
    Annotation[] annotations;
    boolean arrayAllocator;
    boolean bufferGetter;
    Class<?> cls;
    boolean criticalRegion;
    boolean deallocator;
    int dim;
    boolean memberGetter;
    String[] memberName;
    boolean memberSetter;
    Method method;
    int modifiers;
    String name;
    boolean noOffset;
    boolean noReturnGetter;
    boolean overloaded;
    Method pairedMethod;
    Annotation[][] parameterAnnotations;
    boolean[] parameterRaw;
    Class<?>[] parameterTypes;
    boolean returnRaw;
    Class<?> returnType;
    Class<?> throwsException;
    boolean valueGetter;
    boolean valueSetter;
    boolean withEnv;
}
