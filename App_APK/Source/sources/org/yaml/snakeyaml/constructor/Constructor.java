package org.yaml.snakeyaml.constructor;

import java.math.BigDecimal;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collection;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import java.util.SortedMap;
import java.util.SortedSet;
import java.util.TreeMap;
import java.util.TreeSet;
import org.yaml.snakeyaml.TypeDescription;
import org.yaml.snakeyaml.constructor.SafeConstructor;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.introspector.Property;
import org.yaml.snakeyaml.nodes.MappingNode;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.NodeId;
import org.yaml.snakeyaml.nodes.NodeTuple;
import org.yaml.snakeyaml.nodes.ScalarNode;
import org.yaml.snakeyaml.nodes.SequenceNode;
import org.yaml.snakeyaml.nodes.Tag;

public class Constructor extends SafeConstructor {
    /* access modifiers changed from: private */
    public final Map<Class<? extends Object>, TypeDescription> typeDefinitions;
    private final Map<Tag, Class<? extends Object>> typeTags;

    public Constructor() {
        this((Class<? extends Object>) Object.class);
    }

    public Constructor(Class<? extends Object> theRoot) {
        this(new TypeDescription(checkRoot(theRoot)));
    }

    private static Class<? extends Object> checkRoot(Class<? extends Object> theRoot) {
        if (theRoot != null) {
            return theRoot;
        }
        throw new NullPointerException("Root class must be provided.");
    }

    public Constructor(TypeDescription theRoot) {
        if (theRoot != null) {
            this.yamlConstructors.put((Object) null, new ConstructYamlObject());
            if (!Object.class.equals(theRoot.getType())) {
                this.rootTag = new Tag(theRoot.getType());
            }
            this.typeTags = new HashMap();
            this.typeDefinitions = new HashMap();
            this.yamlClassConstructors.put(NodeId.scalar, new ConstructScalar());
            this.yamlClassConstructors.put(NodeId.mapping, new ConstructMapping());
            this.yamlClassConstructors.put(NodeId.sequence, new ConstructSequence());
            addTypeDescription(theRoot);
            return;
        }
        throw new NullPointerException("Root type must be provided.");
    }

    public Constructor(String theRoot) throws ClassNotFoundException {
        this((Class<? extends Object>) Class.forName(check(theRoot)));
    }

    private static final String check(String s) {
        if (s == null) {
            throw new NullPointerException("Root type must be provided.");
        } else if (s.trim().length() != 0) {
            return s;
        } else {
            throw new YAMLException("Root type must be provided.");
        }
    }

    public TypeDescription addTypeDescription(TypeDescription definition) {
        if (definition != null) {
            this.typeTags.put(definition.getTag(), definition.getType());
            return this.typeDefinitions.put(definition.getType(), definition);
        }
        throw new NullPointerException("TypeDescription is required.");
    }

    protected class ConstructMapping implements Construct {
        protected ConstructMapping() {
        }

        public Object construct(Node node) {
            MappingNode mnode = (MappingNode) node;
            if (Properties.class.isAssignableFrom(node.getType())) {
                Properties properties = new Properties();
                if (!node.isTwoStepsConstruction()) {
                    Constructor.this.constructMapping2ndStep(mnode, properties);
                    return properties;
                }
                throw new YAMLException("Properties must not be recursive.");
            } else if (SortedMap.class.isAssignableFrom(node.getType())) {
                SortedMap<Object, Object> map = new TreeMap<>();
                if (!node.isTwoStepsConstruction()) {
                    Constructor.this.constructMapping2ndStep(mnode, map);
                }
                return map;
            } else if (Map.class.isAssignableFrom(node.getType())) {
                if (node.isTwoStepsConstruction()) {
                    return Constructor.this.createDefaultMap();
                }
                return Constructor.this.constructMapping(mnode);
            } else if (SortedSet.class.isAssignableFrom(node.getType())) {
                SortedSet<Object> set = new TreeSet<>();
                Constructor.this.constructSet2ndStep(mnode, set);
                return set;
            } else if (Collection.class.isAssignableFrom(node.getType())) {
                if (node.isTwoStepsConstruction()) {
                    return Constructor.this.createDefaultSet();
                }
                return Constructor.this.constructSet(mnode);
            } else if (node.isTwoStepsConstruction()) {
                return createEmptyJavaBean(mnode);
            } else {
                return constructJavaBean2ndStep(mnode, createEmptyJavaBean(mnode));
            }
        }

        public void construct2ndStep(Node node, Object object) {
            if (Map.class.isAssignableFrom(node.getType())) {
                Constructor.this.constructMapping2ndStep((MappingNode) node, (Map) object);
            } else if (Set.class.isAssignableFrom(node.getType())) {
                Constructor.this.constructSet2ndStep((MappingNode) node, (Set) object);
            } else {
                constructJavaBean2ndStep((MappingNode) node, object);
            }
        }

        /* access modifiers changed from: protected */
        public Object createEmptyJavaBean(MappingNode node) {
            try {
                java.lang.reflect.Constructor<? extends Object> declaredConstructor = node.getType().getDeclaredConstructor(new Class[0]);
                declaredConstructor.setAccessible(true);
                return declaredConstructor.newInstance(new Object[0]);
            } catch (Exception e) {
                throw new YAMLException((Throwable) e);
            }
        }

        /* access modifiers changed from: protected */
        public Object constructJavaBean2ndStep(MappingNode node, Object object) {
            Class<? extends Object> beanType;
            Class<?>[] arguments;
            Class<?> t;
            Object obj = object;
            Constructor.this.flattenMapping(node);
            Class<? extends Object> beanType2 = node.getType();
            Iterator i$ = node.getValue().iterator();
            while (true) {
                Iterator i$2 = i$;
                if (i$2.hasNext()) {
                    NodeTuple tuple = i$2.next();
                    if (tuple.getKeyNode() instanceof ScalarNode) {
                        ScalarNode keyNode = (ScalarNode) tuple.getKeyNode();
                        Node valueNode = tuple.getValueNode();
                        keyNode.setType(String.class);
                        String key = (String) Constructor.this.constructObject(keyNode);
                        try {
                            Property property = getProperty(beanType2, key);
                            valueNode.setType(property.getType());
                            TypeDescription memberDescription = (TypeDescription) Constructor.this.typeDefinitions.get(beanType2);
                            boolean typeDetected = false;
                            if (memberDescription != null) {
                                switch (valueNode.getNodeId()) {
                                    case sequence:
                                        beanType = beanType2;
                                        SequenceNode snode = (SequenceNode) valueNode;
                                        Class<? extends Object> memberType = memberDescription.getListPropertyType(key);
                                        if (memberType == null) {
                                            if (property.getType().isArray()) {
                                                snode.setListType(property.getType().getComponentType());
                                                typeDetected = true;
                                                break;
                                            }
                                        } else {
                                            snode.setListType(memberType);
                                            typeDetected = true;
                                            break;
                                        }
                                        break;
                                    case mapping:
                                        MappingNode mnode = (MappingNode) valueNode;
                                        Class<? extends Object> keyType = memberDescription.getMapKeyType(key);
                                        if (keyType != null) {
                                            beanType = beanType2;
                                            try {
                                                mnode.setTypes(keyType, memberDescription.getMapValueType(key));
                                                typeDetected = true;
                                                break;
                                            } catch (Exception e) {
                                                e = e;
                                                throw new YAMLException("Cannot create property=" + key + " for JavaBean=" + obj + "; " + e.getMessage(), e);
                                            }
                                        }
                                        break;
                                }
                            }
                            beanType = beanType2;
                            if (!(typeDetected || valueNode.getNodeId() == NodeId.scalar || (arguments = property.getActualTypeArguments()) == null)) {
                                if (valueNode.getNodeId() == NodeId.sequence) {
                                    t = arguments[0];
                                    ((SequenceNode) valueNode).setListType(t);
                                } else if (valueNode.getTag().equals(Tag.SET)) {
                                    t = arguments[0];
                                    MappingNode mnode2 = (MappingNode) valueNode;
                                    mnode2.setOnlyKeyType(t);
                                    mnode2.setUseClassConstructor(true);
                                } else if (property.getType().isAssignableFrom(Map.class)) {
                                    t = arguments[0];
                                    MappingNode mnode3 = (MappingNode) valueNode;
                                    mnode3.setTypes(t, arguments[1]);
                                    mnode3.setUseClassConstructor(true);
                                }
                                Class<?> memberType2 = t;
                            }
                            property.set(obj, Constructor.this.constructObject(valueNode));
                            i$ = i$2;
                            beanType2 = beanType;
                            MappingNode mappingNode = node;
                        } catch (Exception e2) {
                            e = e2;
                            Class<? extends Object> cls = beanType2;
                            throw new YAMLException("Cannot create property=" + key + " for JavaBean=" + obj + "; " + e.getMessage(), e);
                        }
                    } else {
                        throw new YAMLException("Keys must be scalars but found: " + tuple.getKeyNode());
                    }
                } else {
                    return obj;
                }
            }
        }

        /* access modifiers changed from: protected */
        public Property getProperty(Class<? extends Object> type, String name) {
            return Constructor.this.getPropertyUtils().getProperty(type, name);
        }
    }

    protected class ConstructYamlObject implements Construct {
        protected ConstructYamlObject() {
        }

        private Construct getConstructor(Node node) {
            node.setType(Constructor.this.getClassForNode(node));
            return (Construct) Constructor.this.yamlClassConstructors.get(node.getNodeId());
        }

        public Object construct(Node node) {
            try {
                return getConstructor(node).construct(node);
            } catch (Exception e) {
                Exception e2 = e;
                throw new ConstructorException((String) null, (Mark) null, "Can't construct a java object for " + node.getTag() + "; exception=" + e2.getMessage(), node.getStartMark(), e2);
            }
        }

        public void construct2ndStep(Node node, Object object) {
            try {
                getConstructor(node).construct2ndStep(node, object);
            } catch (Exception e) {
                Exception e2 = e;
                throw new ConstructorException((String) null, (Mark) null, "Can't construct a second step for a java object for " + node.getTag() + "; exception=" + e2.getMessage(), node.getStartMark(), e2);
            }
        }
    }

    protected class ConstructScalar extends AbstractConstruct {
        protected ConstructScalar() {
        }

        public Object construct(Node nnode) {
            Object argument;
            ScalarNode node = (ScalarNode) nnode;
            Class<? extends Object> type = node.getType();
            if (type.isPrimitive() || type == String.class || Number.class.isAssignableFrom(type) || type == Boolean.class || Date.class.isAssignableFrom(type) || type == Character.class || type == BigInteger.class || type == BigDecimal.class || Enum.class.isAssignableFrom(type) || Tag.BINARY.equals(node.getTag()) || Calendar.class.isAssignableFrom(type)) {
                return constructStandardJavaInstance(type, node);
            }
            java.lang.reflect.Constructor<?> javaConstructor = null;
            int oneArgCount = 0;
            for (java.lang.reflect.Constructor<?> c : type.getConstructors()) {
                if (c.getParameterTypes().length == 1) {
                    oneArgCount++;
                    javaConstructor = c;
                }
            }
            if (javaConstructor != null) {
                if (oneArgCount == 1) {
                    argument = constructStandardJavaInstance(javaConstructor.getParameterTypes()[0], node);
                } else {
                    argument = Constructor.this.constructScalar(node);
                    try {
                        javaConstructor = type.getConstructor(new Class[]{String.class});
                    } catch (Exception e) {
                        Exception e2 = e;
                        throw new ConstructorException((String) null, (Mark) null, "Can't construct a java object for scalar " + node.getTag() + "; No String constructor found. Exception=" + e2.getMessage(), node.getStartMark(), e2);
                    }
                }
                try {
                    return javaConstructor.newInstance(new Object[]{argument});
                } catch (Exception e3) {
                    Exception e4 = e3;
                    throw new ConstructorException((String) null, (Mark) null, "Can't construct a java object for scalar " + node.getTag() + "; exception=" + e4.getMessage(), node.getStartMark(), e4);
                }
            } else {
                throw new YAMLException("No single argument constructor found for " + type);
            }
        }

        private Object constructStandardJavaInstance(Class type, ScalarNode node) {
            Object result;
            Object result2;
            Object result3;
            if (type == String.class) {
                return ((Construct) Constructor.this.yamlConstructors.get(Tag.STR)).construct(node);
            }
            if (type == Boolean.class || type == Boolean.TYPE) {
                return ((Construct) Constructor.this.yamlConstructors.get(Tag.BOOL)).construct(node);
            }
            if (type == Character.class || type == Character.TYPE) {
                String ch = (String) ((Construct) Constructor.this.yamlConstructors.get(Tag.STR)).construct(node);
                if (ch.length() == 0) {
                    result = null;
                } else if (ch.length() == 1) {
                    result = Character.valueOf(ch.charAt(0));
                } else {
                    throw new YAMLException("Invalid node Character: '" + ch + "'; length: " + ch.length());
                }
                return result;
            } else if (Date.class.isAssignableFrom(type)) {
                Date date = (Date) ((Construct) Constructor.this.yamlConstructors.get(Tag.TIMESTAMP)).construct(node);
                if (type == Date.class) {
                    result3 = date;
                } else {
                    try {
                        result3 = type.getConstructor(new Class[]{Long.TYPE}).newInstance(new Object[]{Long.valueOf(date.getTime())});
                    } catch (Exception e) {
                        throw new YAMLException("Cannot construct: '" + type + "'");
                    }
                }
                return result3;
            } else if (type == Float.class || type == Double.class || type == Float.TYPE || type == Double.TYPE || type == BigDecimal.class) {
                if (type == BigDecimal.class) {
                    return new BigDecimal(node.getValue());
                }
                Object result4 = ((Construct) Constructor.this.yamlConstructors.get(Tag.FLOAT)).construct(node);
                if (type == Float.class || type == Float.TYPE) {
                    return new Float(((Double) result4).doubleValue());
                }
                return result4;
            } else if (type == Byte.class || type == Short.class || type == Integer.class || type == Long.class || type == BigInteger.class || type == Byte.TYPE || type == Short.TYPE || type == Integer.TYPE || type == Long.TYPE) {
                Object result5 = ((Construct) Constructor.this.yamlConstructors.get(Tag.INT)).construct(node);
                if (type == Byte.class || type == Byte.TYPE) {
                    result2 = new Byte(result5.toString());
                } else if (type == Short.class || type == Short.TYPE) {
                    result2 = new Short(result5.toString());
                } else if (type == Integer.class || type == Integer.TYPE) {
                    result2 = Integer.valueOf(Integer.parseInt(result5.toString()));
                } else if (type != Long.class && type != Long.TYPE) {
                    return new BigInteger(result5.toString());
                } else {
                    result2 = new Long(result5.toString());
                }
                return result2;
            } else if (Enum.class.isAssignableFrom(type)) {
                String enumValueName = node.getValue();
                try {
                    return Enum.valueOf(type, enumValueName);
                } catch (Exception e2) {
                    throw new YAMLException("Unable to find enum value '" + enumValueName + "' for enum class: " + type.getName());
                }
            } else if (Calendar.class.isAssignableFrom(type)) {
                SafeConstructor.ConstructYamlTimestamp contr = new SafeConstructor.ConstructYamlTimestamp();
                contr.construct(node);
                return contr.getCalendar();
            } else {
                throw new YAMLException("Unsupported class: " + type);
            }
        }
    }

    protected class ConstructSequence implements Construct {
        protected ConstructSequence() {
        }

        public Object construct(Node node) {
            SequenceNode snode = (SequenceNode) node;
            if (Set.class.isAssignableFrom(node.getType())) {
                if (!node.isTwoStepsConstruction()) {
                    return Constructor.this.constructSet(snode);
                }
                throw new YAMLException("Set cannot be recursive.");
            } else if (Collection.class.isAssignableFrom(node.getType())) {
                if (node.isTwoStepsConstruction()) {
                    return Constructor.this.createDefaultList(snode.getValue().size());
                }
                return Constructor.this.constructSequence(snode);
            } else if (!node.getType().isArray()) {
                List<java.lang.reflect.Constructor<?>> possibleConstructors = new ArrayList<>(snode.getValue().size());
                for (java.lang.reflect.Constructor<?> constructor : node.getType().getConstructors()) {
                    if (snode.getValue().size() == constructor.getParameterTypes().length) {
                        possibleConstructors.add(constructor);
                    }
                }
                if (!possibleConstructors.isEmpty()) {
                    if (possibleConstructors.size() == 1) {
                        Object[] argumentList = new Object[snode.getValue().size()];
                        java.lang.reflect.Constructor<?> c = possibleConstructors.get(0);
                        int index = 0;
                        for (Node argumentNode : snode.getValue()) {
                            argumentNode.setType(c.getParameterTypes()[index]);
                            argumentList[index] = Constructor.this.constructObject(argumentNode);
                            index++;
                        }
                        try {
                            return c.newInstance(argumentList);
                        } catch (Exception e) {
                            throw new YAMLException((Throwable) e);
                        }
                    } else {
                        List<? extends Object> constructSequence = Constructor.this.constructSequence(snode);
                        Class<?>[] parameterTypes = new Class[constructSequence.size()];
                        int index2 = 0;
                        for (Object parameter : constructSequence) {
                            parameterTypes[index2] = parameter.getClass();
                            index2++;
                        }
                        for (java.lang.reflect.Constructor<?> c2 : possibleConstructors) {
                            Class<?>[] argTypes = c2.getParameterTypes();
                            boolean foundConstructor = true;
                            int i = 0;
                            while (true) {
                                if (i >= argTypes.length) {
                                    break;
                                } else if (!wrapIfPrimitive(argTypes[i]).isAssignableFrom(parameterTypes[i])) {
                                    foundConstructor = false;
                                    continue;
                                    break;
                                } else {
                                    i++;
                                }
                            }
                            if (foundConstructor) {
                                try {
                                    return c2.newInstance(constructSequence.toArray());
                                } catch (Exception e2) {
                                    throw new YAMLException((Throwable) e2);
                                }
                            }
                        }
                    }
                }
                throw new YAMLException("No suitable constructor with " + String.valueOf(snode.getValue().size()) + " arguments found for " + node.getType());
            } else if (node.isTwoStepsConstruction()) {
                return Constructor.this.createArray(node.getType(), snode.getValue().size());
            } else {
                return Constructor.this.constructArray(snode);
            }
        }

        private final Class<? extends Object> wrapIfPrimitive(Class<?> clazz) {
            if (!clazz.isPrimitive()) {
                return clazz;
            }
            if (clazz == Integer.TYPE) {
                return Integer.class;
            }
            if (clazz == Float.TYPE) {
                return Float.class;
            }
            if (clazz == Double.TYPE) {
                return Double.class;
            }
            if (clazz == Boolean.TYPE) {
                return Boolean.class;
            }
            if (clazz == Long.TYPE) {
                return Long.class;
            }
            if (clazz == Character.TYPE) {
                return Character.class;
            }
            if (clazz == Short.TYPE) {
                return Short.class;
            }
            if (clazz == Byte.TYPE) {
                return Byte.class;
            }
            throw new YAMLException("Unexpected primitive " + clazz);
        }

        public void construct2ndStep(Node node, Object object) {
            SequenceNode snode = (SequenceNode) node;
            if (List.class.isAssignableFrom(node.getType())) {
                Constructor.this.constructSequenceStep2(snode, (List) object);
            } else if (node.getType().isArray()) {
                Constructor.this.constructArrayStep2(snode, object);
            } else {
                throw new YAMLException("Immutable objects cannot be recursive.");
            }
        }
    }

    /* access modifiers changed from: protected */
    public Class<?> getClassForNode(Node node) {
        Class<? extends Object> classForTag = this.typeTags.get(node.getTag());
        if (classForTag != null) {
            return classForTag;
        }
        String name = node.getTag().getClassName();
        try {
            Class<?> cl = getClassForName(name);
            this.typeTags.put(node.getTag(), cl);
            return cl;
        } catch (ClassNotFoundException e) {
            throw new YAMLException("Class not found: " + name);
        }
    }

    /* access modifiers changed from: protected */
    public Class<?> getClassForName(String name) throws ClassNotFoundException {
        return Class.forName(name);
    }
}
