package org.yaml.snakeyaml.nodes;

import java.math.BigDecimal;
import java.math.BigInteger;
import java.net.URI;
import java.sql.Timestamp;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.util.UriEncoder;

public final class Tag implements Comparable<Tag> {
    public static final Tag BINARY = new Tag("tag:yaml.org,2002:binary");
    public static final Tag BOOL = new Tag("tag:yaml.org,2002:bool");
    public static final Map<Tag, Set<Class<?>>> COMPATIBILITY_MAP = new HashMap();
    public static final Tag FLOAT = new Tag("tag:yaml.org,2002:float");
    public static final Tag INT = new Tag("tag:yaml.org,2002:int");
    public static final Tag MAP = new Tag("tag:yaml.org,2002:map");
    public static final Tag MERGE = new Tag("tag:yaml.org,2002:merge");
    public static final Tag NULL = new Tag("tag:yaml.org,2002:null");
    public static final Tag OMAP = new Tag("tag:yaml.org,2002:omap");
    public static final Tag PAIRS = new Tag("tag:yaml.org,2002:pairs");
    public static final String PREFIX = "tag:yaml.org,2002:";
    public static final Tag SEQ = new Tag("tag:yaml.org,2002:seq");
    public static final Tag SET = new Tag("tag:yaml.org,2002:set");
    public static final Tag STR = new Tag("tag:yaml.org,2002:str");
    public static final Tag TIMESTAMP = new Tag("tag:yaml.org,2002:timestamp");
    public static final Tag VALUE = new Tag("tag:yaml.org,2002:value");
    public static final Tag YAML = new Tag("tag:yaml.org,2002:yaml");
    private final String value;

    static {
        Set<Class<?>> floatSet = new HashSet<>();
        floatSet.add(Double.class);
        floatSet.add(Float.class);
        floatSet.add(BigDecimal.class);
        COMPATIBILITY_MAP.put(FLOAT, floatSet);
        Set<Class<?>> intSet = new HashSet<>();
        intSet.add(Integer.class);
        intSet.add(Long.class);
        intSet.add(BigInteger.class);
        COMPATIBILITY_MAP.put(INT, intSet);
        Set<Class<?>> timestampSet = new HashSet<>();
        timestampSet.add(Date.class);
        timestampSet.add(java.sql.Date.class);
        timestampSet.add(Timestamp.class);
        COMPATIBILITY_MAP.put(TIMESTAMP, timestampSet);
    }

    public Tag(String tag) {
        if (tag == null) {
            throw new NullPointerException("Tag must be provided.");
        } else if (tag.length() == 0) {
            throw new IllegalArgumentException("Tag must not be empty.");
        } else if (tag.trim().length() == tag.length()) {
            this.value = UriEncoder.encode(tag);
        } else {
            throw new IllegalArgumentException("Tag must not contain leading or trailing spaces.");
        }
    }

    public Tag(Class<? extends Object> clazz) {
        if (clazz != null) {
            this.value = PREFIX + UriEncoder.encode(clazz.getName());
            return;
        }
        throw new NullPointerException("Class for tag must be provided.");
    }

    public Tag(URI uri) {
        if (uri != null) {
            this.value = uri.toASCIIString();
            return;
        }
        throw new NullPointerException("URI for tag must be provided.");
    }

    public String getValue() {
        return this.value;
    }

    public boolean startsWith(String prefix) {
        return this.value.startsWith(prefix);
    }

    public String getClassName() {
        if (this.value.startsWith(PREFIX)) {
            return UriEncoder.decode(this.value.substring(PREFIX.length()));
        }
        throw new YAMLException("Invalid tag: " + this.value);
    }

    public int getLength() {
        return this.value.length();
    }

    public String toString() {
        return this.value;
    }

    public boolean equals(Object obj) {
        if (obj instanceof Tag) {
            return this.value.equals(((Tag) obj).getValue());
        }
        if (!(obj instanceof String) || !this.value.equals(obj.toString())) {
            return false;
        }
        System.err.println("Comparing Tag and String is deprecated.");
        return true;
    }

    public int hashCode() {
        return this.value.hashCode();
    }

    public boolean isCompatible(Class<?> clazz) {
        Set<Class<?>> set = COMPATIBILITY_MAP.get(this);
        if (set != null) {
            return set.contains(clazz);
        }
        return false;
    }

    public boolean matches(Class<? extends Object> clazz) {
        String str = this.value;
        return str.equals(PREFIX + clazz.getName());
    }

    public int compareTo(Tag o) {
        return this.value.compareTo(o.getValue());
    }
}
