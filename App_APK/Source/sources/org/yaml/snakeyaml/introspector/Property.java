package org.yaml.snakeyaml.introspector;

public abstract class Property implements Comparable<Property> {
    private final String name;
    private final Class<?> type;

    public abstract Object get(Object obj);

    public abstract Class<?>[] getActualTypeArguments();

    public abstract void set(Object obj, Object obj2) throws Exception;

    public Property(String name2, Class<?> type2) {
        this.name = name2;
        this.type = type2;
    }

    public Class<?> getType() {
        return this.type;
    }

    public String getName() {
        return this.name;
    }

    public String toString() {
        return getName() + " of " + getType();
    }

    public int compareTo(Property o) {
        return this.name.compareTo(o.name);
    }

    public boolean isWritable() {
        return true;
    }

    public boolean isReadable() {
        return true;
    }

    public int hashCode() {
        return this.name.hashCode() + this.type.hashCode();
    }

    public boolean equals(Object other) {
        if (!(other instanceof Property)) {
            return false;
        }
        Property p = (Property) other;
        if (!this.name.equals(p.getName()) || !this.type.equals(p.getType())) {
            return false;
        }
        return true;
    }
}
