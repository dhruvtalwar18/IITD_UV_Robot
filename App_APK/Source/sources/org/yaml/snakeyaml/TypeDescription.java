package org.yaml.snakeyaml;

import java.util.HashMap;
import java.util.Map;
import org.yaml.snakeyaml.nodes.Tag;

public final class TypeDescription {
    private Map<String, Class<? extends Object>> keyProperties;
    private Map<String, Class<? extends Object>> listProperties;
    private Tag tag;
    private final Class<? extends Object> type;
    private Map<String, Class<? extends Object>> valueProperties;

    public TypeDescription(Class<? extends Object> clazz, Tag tag2) {
        this.type = clazz;
        this.tag = tag2;
        this.listProperties = new HashMap();
        this.keyProperties = new HashMap();
        this.valueProperties = new HashMap();
    }

    public TypeDescription(Class<? extends Object> clazz, String tag2) {
        this(clazz, new Tag(tag2));
    }

    public TypeDescription(Class<? extends Object> clazz) {
        this(clazz, (Tag) null);
    }

    public Tag getTag() {
        return this.tag;
    }

    public void setTag(Tag tag2) {
        this.tag = tag2;
    }

    public void setTag(String tag2) {
        setTag(new Tag(tag2));
    }

    public Class<? extends Object> getType() {
        return this.type;
    }

    public void putListPropertyType(String property, Class<? extends Object> type2) {
        this.listProperties.put(property, type2);
    }

    public Class<? extends Object> getListPropertyType(String property) {
        return this.listProperties.get(property);
    }

    public void putMapPropertyType(String property, Class<? extends Object> key, Class<? extends Object> value) {
        this.keyProperties.put(property, key);
        this.valueProperties.put(property, value);
    }

    public Class<? extends Object> getMapKeyType(String property) {
        return this.keyProperties.get(property);
    }

    public Class<? extends Object> getMapValueType(String property) {
        return this.valueProperties.get(property);
    }

    public String toString() {
        return "TypeDescription for " + getType() + " (tag='" + getTag() + "')";
    }
}
