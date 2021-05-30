package org.yaml.snakeyaml.introspector;

import java.lang.reflect.Array;
import java.lang.reflect.GenericArrayType;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;

public abstract class GenericProperty extends Property {
    private Class<?>[] actualClasses;
    private boolean actualClassesChecked;
    private Type genType;

    public GenericProperty(String name, Class<?> aClass, Type aType) {
        super(name, aClass);
        this.genType = aType;
        this.actualClassesChecked = aType == null;
    }

    public Class<?>[] getActualTypeArguments() {
        if (!this.actualClassesChecked) {
            if (this.genType instanceof ParameterizedType) {
                Type[] actualTypeArguments = ((ParameterizedType) this.genType).getActualTypeArguments();
                if (actualTypeArguments.length > 0) {
                    this.actualClasses = new Class[actualTypeArguments.length];
                    int i = 0;
                    while (true) {
                        if (i >= actualTypeArguments.length) {
                            break;
                        }
                        if (!(actualTypeArguments[i] instanceof Class)) {
                            if (!(actualTypeArguments[i] instanceof ParameterizedType)) {
                                if (!(actualTypeArguments[i] instanceof GenericArrayType)) {
                                    this.actualClasses = null;
                                    break;
                                }
                                Type componentType = ((GenericArrayType) actualTypeArguments[i]).getGenericComponentType();
                                if (!(componentType instanceof Class)) {
                                    this.actualClasses = null;
                                    break;
                                }
                                this.actualClasses[i] = Array.newInstance((Class) componentType, 0).getClass();
                            } else {
                                this.actualClasses[i] = (Class) ((ParameterizedType) actualTypeArguments[i]).getRawType();
                            }
                        } else {
                            this.actualClasses[i] = (Class) actualTypeArguments[i];
                        }
                        i++;
                    }
                }
            } else if (this.genType instanceof GenericArrayType) {
                Type componentType2 = ((GenericArrayType) this.genType).getGenericComponentType();
                if (componentType2 instanceof Class) {
                    this.actualClasses = new Class[]{(Class) componentType2};
                }
            } else if ((this.genType instanceof Class) && ((Class) this.genType).isArray()) {
                this.actualClasses = new Class[1];
                this.actualClasses[0] = getType().getComponentType();
            }
            this.actualClassesChecked = true;
        }
        return this.actualClasses;
    }
}
