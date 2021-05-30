package org.yaml.snakeyaml.extensions.compactnotation;

import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.yaml.snakeyaml.constructor.AbstractConstruct;
import org.yaml.snakeyaml.constructor.Construct;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.introspector.Property;
import org.yaml.snakeyaml.nodes.MappingNode;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.NodeTuple;
import org.yaml.snakeyaml.nodes.ScalarNode;

public class CompactConstructor extends Constructor {
    private static final Pattern FIRST_PATTERN = Pattern.compile("(\\p{Alpha}.*)(\\s*)\\((.*?)\\)");
    private static final Pattern PROPERTY_NAME_PATTERN = Pattern.compile("\\s*(\\p{Alpha}\\w*)\\s*=(.+)");

    /* access modifiers changed from: protected */
    public Object constructScalar(ScalarNode node) {
        CompactData data = getCompactData(node.getValue());
        if (data != null) {
            return constructCompactFormat(node, data);
        }
        return super.constructScalar(node);
    }

    /* access modifiers changed from: protected */
    public Object constructCompactFormat(ScalarNode node, CompactData data) {
        try {
            Object obj = createInstance(node, data);
            setProperties(obj, new HashMap<>(data.getProperties()));
            return obj;
        } catch (Exception e) {
            throw new YAMLException((Throwable) e);
        }
    }

    /* access modifiers changed from: protected */
    public Object createInstance(ScalarNode node, CompactData data) throws Exception {
        Class<?> clazz = getClassForName(data.getPrefix());
        Class[] clsArr = new Class[data.getArguments().size()];
        for (int i = 0; i < clsArr.length; i++) {
            clsArr[i] = String.class;
        }
        java.lang.reflect.Constructor<?> c = clazz.getDeclaredConstructor(clsArr);
        c.setAccessible(true);
        return c.newInstance(data.getArguments().toArray());
    }

    /* access modifiers changed from: protected */
    public void setProperties(Object bean, Map<String, Object> data) throws Exception {
        if (data != null) {
            for (Map.Entry<String, Object> entry : data.entrySet()) {
                String key = entry.getKey();
                try {
                    getPropertyUtils().getProperty(bean.getClass(), key).set(bean, entry.getValue());
                } catch (IllegalArgumentException e) {
                    throw new YAMLException("Cannot set property='" + key + "' with value='" + data.get(key) + "' (" + data.get(key).getClass() + ") in " + bean);
                }
            }
            return;
        }
        throw new NullPointerException("Data for Compact Object Notation cannot be null.");
    }

    public CompactData getCompactData(String scalar) {
        if (!scalar.endsWith(")") || scalar.indexOf(40) < 0) {
            return null;
        }
        Matcher m = FIRST_PATTERN.matcher(scalar);
        if (!m.matches()) {
            return null;
        }
        String tag = m.group(1).trim();
        String content = m.group(3);
        CompactData data = new CompactData(tag);
        if (content.length() == 0) {
            return data;
        }
        String[] names = content.split("\\s*,\\s*");
        for (String section : names) {
            if (section.indexOf(61) < 0) {
                data.getArguments().add(section);
            } else {
                Matcher sm = PROPERTY_NAME_PATTERN.matcher(section);
                if (!sm.matches()) {
                    return null;
                }
                data.getProperties().put(sm.group(1), sm.group(2).trim());
            }
        }
        return data;
    }

    /* access modifiers changed from: protected */
    public Construct getConstructor(Node node) {
        if (node instanceof MappingNode) {
            List<NodeTuple> list = ((MappingNode) node).getValue();
            if (list.size() == 1) {
                Node key = list.get(0).getKeyNode();
                if ((key instanceof ScalarNode) && getCompactData(((ScalarNode) key).getValue()) != null) {
                    return new ConstructCompactObject();
                }
            }
        }
        return super.getConstructor(node);
    }

    public class ConstructCompactObject extends AbstractConstruct {
        public ConstructCompactObject() {
        }

        public Object construct(Node node) {
            Map.Entry<Object, Object> entry = CompactConstructor.this.constructMapping((MappingNode) node).entrySet().iterator().next();
            Object result = entry.getKey();
            Object value = entry.getValue();
            if (value instanceof Map) {
                try {
                    CompactConstructor.this.setProperties(result, (Map) value);
                } catch (Exception e) {
                    throw new YAMLException((Throwable) e);
                }
            } else {
                CompactConstructor.this.applySequence(result, (List) value);
            }
            return result;
        }
    }

    /* access modifiers changed from: protected */
    public void applySequence(Object bean, List<?> value) {
        try {
            getPropertyUtils().getProperty(bean.getClass(), getSequencePropertyName(bean.getClass())).set(bean, value);
        } catch (Exception e) {
            throw new YAMLException((Throwable) e);
        }
    }

    /* access modifiers changed from: protected */
    public String getSequencePropertyName(Class<?> bean) {
        Set<Property> properties = getPropertyUtils().getProperties(bean);
        Iterator<Property> iterator = properties.iterator();
        while (iterator.hasNext()) {
            if (!List.class.isAssignableFrom(iterator.next().getType())) {
                iterator.remove();
            }
        }
        if (properties.size() == 0) {
            throw new YAMLException("No list property found in " + bean);
        } else if (properties.size() <= 1) {
            return properties.iterator().next().getName();
        } else {
            throw new YAMLException("Many list properties found in " + bean + "; Please override getSequencePropertyName() to specify which property to use.");
        }
    }
}
