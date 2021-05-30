package org.bytedeco.javacpp.tools;

import java.util.LinkedHashMap;

class TemplateMap extends LinkedHashMap<String, Type> {
    Declarator declarator = null;
    TemplateMap parent = null;
    Type type = null;
    boolean variadic = false;

    TemplateMap(TemplateMap parent2) {
        this.parent = parent2;
    }

    /* access modifiers changed from: package-private */
    public String getName() {
        if (this.type != null) {
            return this.type.cppName;
        }
        if (this.declarator != null) {
            return this.declarator.cppName;
        }
        return null;
    }

    /* access modifiers changed from: package-private */
    public boolean empty() {
        for (Type t : values()) {
            if (t != null) {
                return false;
            }
        }
        return !isEmpty();
    }

    /* access modifiers changed from: package-private */
    public boolean full() {
        for (Type t : values()) {
            if (t == null) {
                return false;
            }
        }
        return true;
    }

    /* access modifiers changed from: package-private */
    public Type get(String key) {
        Type value = (Type) super.get(key);
        if (value != null || this.parent == null) {
            return value;
        }
        return this.parent.get(key);
    }
}
