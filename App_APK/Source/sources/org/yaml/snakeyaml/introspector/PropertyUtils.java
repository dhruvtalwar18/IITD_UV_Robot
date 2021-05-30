package org.yaml.snakeyaml.introspector;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import org.yaml.snakeyaml.error.YAMLException;

public class PropertyUtils {
    private boolean allowReadOnlyProperties = false;
    private BeanAccess beanAccess = BeanAccess.DEFAULT;
    private final Map<Class<?>, Map<String, Property>> propertiesCache = new HashMap();
    private final Map<Class<?>, Set<Property>> readableProperties = new HashMap();

    /* access modifiers changed from: protected */
    public Map<String, Property> getPropertiesMap(Class<?> type, BeanAccess bAccess) {
        if (this.propertiesCache.containsKey(type)) {
            return this.propertiesCache.get(type);
        }
        Map<String, Property> properties = new LinkedHashMap<>();
        for (Class<?> c = type; c != null; c = c.getSuperclass()) {
            for (Field field : c.getDeclaredFields()) {
                int modifiers = field.getModifiers();
                if (!Modifier.isStatic(modifiers) && !Modifier.isTransient(modifiers) && !properties.containsKey(field.getName())) {
                    properties.put(field.getName(), new FieldProperty(field));
                }
            }
        }
        this.propertiesCache.put(type, properties);
        return properties;
    }

    public Set<Property> getProperties(Class<? extends Object> type) {
        return getProperties(type, this.beanAccess);
    }

    public Set<Property> getProperties(Class<? extends Object> type, BeanAccess bAccess) {
        if (this.readableProperties.containsKey(type)) {
            return this.readableProperties.get(type);
        }
        Set<Property> properties = createPropertySet(type, bAccess);
        this.readableProperties.put(type, properties);
        return properties;
    }

    /* access modifiers changed from: protected */
    public Set<Property> createPropertySet(Class<? extends Object> type, BeanAccess bAccess) {
        Set<Property> properties = new TreeSet<>();
        for (Property property : getPropertiesMap(type, bAccess).values()) {
            if (property.isReadable() && (this.allowReadOnlyProperties || property.isWritable())) {
                properties.add(property);
            }
        }
        return properties;
    }

    public Property getProperty(Class<? extends Object> type, String name) {
        return getProperty(type, name, this.beanAccess);
    }

    public Property getProperty(Class<? extends Object> type, String name, BeanAccess bAccess) {
        Property property = getPropertiesMap(type, bAccess).get(name);
        if (property != null && property.isWritable()) {
            return property;
        }
        throw new YAMLException("Unable to find property '" + name + "' on class: " + type.getName());
    }

    public void setBeanAccess(BeanAccess beanAccess2) {
        if (this.beanAccess != beanAccess2) {
            this.beanAccess = beanAccess2;
            this.propertiesCache.clear();
            this.readableProperties.clear();
        }
    }

    public void setAllowReadOnlyProperties(boolean allowReadOnlyProperties2) {
        if (this.allowReadOnlyProperties != allowReadOnlyProperties2) {
            this.allowReadOnlyProperties = allowReadOnlyProperties2;
            this.readableProperties.clear();
        }
    }
}
