package org.yaml.snakeyaml.constructor;

import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TimeZone;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.apache.commons.httpclient.HttpState;
import org.bytedeco.javacpp.opencv_stitching;
import org.yaml.snakeyaml.error.Mark;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.external.biz.base64Coder.Base64Coder;
import org.yaml.snakeyaml.nodes.MappingNode;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.NodeId;
import org.yaml.snakeyaml.nodes.NodeTuple;
import org.yaml.snakeyaml.nodes.ScalarNode;
import org.yaml.snakeyaml.nodes.SequenceNode;
import org.yaml.snakeyaml.nodes.Tag;

public class SafeConstructor extends BaseConstructor {
    /* access modifiers changed from: private */
    public static final Map<String, Boolean> BOOL_VALUES = new HashMap();
    /* access modifiers changed from: private */
    public static final Pattern TIMESTAMP_REGEXP = Pattern.compile("^([0-9][0-9][0-9][0-9])-([0-9][0-9]?)-([0-9][0-9]?)(?:(?:[Tt]|[ \t]+)([0-9][0-9]?):([0-9][0-9]):([0-9][0-9])(?:\\.([0-9]*))?(?:[ \t]*(?:Z|([-+][0-9][0-9]?)(?::([0-9][0-9])?)?))?)?$");
    /* access modifiers changed from: private */
    public static final Pattern YMD_REGEXP = Pattern.compile("^([0-9][0-9][0-9][0-9])-([0-9][0-9]?)-([0-9][0-9]?)$");
    public static final ConstructUndefined undefinedConstructor = new ConstructUndefined();

    static {
        BOOL_VALUES.put("yes", Boolean.TRUE);
        BOOL_VALUES.put("no", Boolean.FALSE);
        BOOL_VALUES.put("true", Boolean.TRUE);
        BOOL_VALUES.put(HttpState.PREEMPTIVE_DEFAULT, Boolean.FALSE);
        BOOL_VALUES.put("on", Boolean.TRUE);
        BOOL_VALUES.put("off", Boolean.FALSE);
    }

    public SafeConstructor() {
        this.yamlConstructors.put(Tag.NULL, new ConstructYamlNull());
        this.yamlConstructors.put(Tag.BOOL, new ConstructYamlBool());
        this.yamlConstructors.put(Tag.INT, new ConstructYamlInt());
        this.yamlConstructors.put(Tag.FLOAT, new ConstructYamlFloat());
        this.yamlConstructors.put(Tag.BINARY, new ConstructYamlBinary());
        this.yamlConstructors.put(Tag.TIMESTAMP, new ConstructYamlTimestamp());
        this.yamlConstructors.put(Tag.OMAP, new ConstructYamlOmap());
        this.yamlConstructors.put(Tag.PAIRS, new ConstructYamlPairs());
        this.yamlConstructors.put(Tag.SET, new ConstructYamlSet());
        this.yamlConstructors.put(Tag.STR, new ConstructYamlStr());
        this.yamlConstructors.put(Tag.SEQ, new ConstructYamlSeq());
        this.yamlConstructors.put(Tag.MAP, new ConstructYamlMap());
        this.yamlConstructors.put((Object) null, undefinedConstructor);
        this.yamlClassConstructors.put(NodeId.scalar, undefinedConstructor);
        this.yamlClassConstructors.put(NodeId.sequence, undefinedConstructor);
        this.yamlClassConstructors.put(NodeId.mapping, undefinedConstructor);
    }

    /* access modifiers changed from: protected */
    public void flattenMapping(MappingNode node) {
        if (node.isMerged()) {
            node.setValue(mergeNode(node, true, new HashMap(), new ArrayList()));
        }
    }

    private List<NodeTuple> mergeNode(MappingNode node, boolean isPreffered, Map<Object, Integer> key2index, List<NodeTuple> values) {
        Iterator<NodeTuple> iter;
        List<NodeTuple> nodeValue;
        Map<Object, Integer> map = key2index;
        List<NodeTuple> list = values;
        List<NodeTuple> nodeValue2 = node.getValue();
        Collections.reverse(nodeValue2);
        Iterator<NodeTuple> iter2 = nodeValue2.iterator();
        while (iter2.hasNext()) {
            NodeTuple nodeTuple = iter2.next();
            Node keyNode = nodeTuple.getKeyNode();
            Node valueNode = nodeTuple.getValueNode();
            if (keyNode.getTag().equals(Tag.MERGE)) {
                iter2.remove();
                switch (valueNode.getNodeId()) {
                    case mapping:
                        nodeValue = nodeValue2;
                        iter = iter2;
                        mergeNode((MappingNode) valueNode, false, map, list);
                        break;
                    case sequence:
                        for (Node subnode : ((SequenceNode) valueNode).getValue()) {
                            if (subnode instanceof MappingNode) {
                                mergeNode((MappingNode) subnode, false, map, list);
                            } else {
                                Mark startMark = node.getStartMark();
                                StringBuilder sb = new StringBuilder();
                                List<NodeTuple> list2 = nodeValue2;
                                sb.append("expected a mapping for merging, but found ");
                                sb.append(subnode.getNodeId());
                                Iterator<NodeTuple> it = iter2;
                                throw new ConstructorException("while constructing a mapping", startMark, sb.toString(), subnode.getStartMark());
                            }
                        }
                        nodeValue = nodeValue2;
                        iter = iter2;
                        break;
                    default:
                        List<NodeTuple> list3 = nodeValue2;
                        Iterator<NodeTuple> it2 = iter2;
                        Mark startMark2 = node.getStartMark();
                        throw new ConstructorException("while constructing a mapping", startMark2, "expected a mapping or list of mappings for merging, but found " + valueNode.getNodeId(), valueNode.getStartMark());
                }
            } else {
                nodeValue = nodeValue2;
                iter = iter2;
                Object key = constructObject(keyNode);
                if (!map.containsKey(key)) {
                    list.add(nodeTuple);
                    map.put(key, Integer.valueOf(values.size() - 1));
                } else if (isPreffered) {
                    list.set(map.get(key).intValue(), nodeTuple);
                }
                Object obj = key;
            }
            nodeValue2 = nodeValue;
            iter2 = iter;
        }
        return list;
    }

    /* access modifiers changed from: protected */
    public void constructMapping2ndStep(MappingNode node, Map<Object, Object> mapping) {
        flattenMapping(node);
        super.constructMapping2ndStep(node, mapping);
    }

    /* access modifiers changed from: protected */
    public void constructSet2ndStep(MappingNode node, Set<Object> set) {
        flattenMapping(node);
        super.constructSet2ndStep(node, set);
    }

    public class ConstructYamlNull extends AbstractConstruct {
        public ConstructYamlNull() {
        }

        public Object construct(Node node) {
            SafeConstructor.this.constructScalar((ScalarNode) node);
            return null;
        }
    }

    public class ConstructYamlBool extends AbstractConstruct {
        public ConstructYamlBool() {
        }

        public Object construct(Node node) {
            return SafeConstructor.BOOL_VALUES.get(((String) SafeConstructor.this.constructScalar((ScalarNode) node)).toLowerCase());
        }
    }

    public class ConstructYamlInt extends AbstractConstruct {
        public ConstructYamlInt() {
        }

        public Object construct(Node node) {
            int base;
            String value;
            String value2 = SafeConstructor.this.constructScalar((ScalarNode) node).toString().replaceAll("_", "");
            int sign = 1;
            char first = value2.charAt(0);
            int i = 1;
            if (first == '-') {
                sign = -1;
                value2 = value2.substring(1);
            } else if (first == '+') {
                value2 = value2.substring(1);
            }
            int base2 = 10;
            if ("0".equals(value2)) {
                return 0;
            }
            if (value2.startsWith("0b")) {
                value = value2.substring(2);
                base = 2;
            } else if (value2.startsWith("0x")) {
                value = value2.substring(2);
                base = 16;
            } else if (value2.startsWith("0")) {
                value = value2.substring(1);
                base = 8;
            } else if (value2.indexOf(58) == -1) {
                return SafeConstructor.this.createNumber(sign, value2, 10);
            } else {
                String[] digits = value2.split(":");
                int bes = 1;
                int val = 0;
                int i2 = 0;
                int j = digits.length;
                while (i2 < j) {
                    val = (int) (((long) val) + (Long.parseLong(digits[(j - i2) - i]) * ((long) bes)));
                    bes *= 60;
                    i2++;
                    base2 = base2;
                    i = 1;
                }
                return SafeConstructor.this.createNumber(sign, String.valueOf(val), 10);
            }
            return SafeConstructor.this.createNumber(sign, value, base);
        }
    }

    /* access modifiers changed from: private */
    public Number createNumber(int sign, String number, int radix) {
        if (sign < 0) {
            number = "-" + number;
        }
        try {
            return Integer.valueOf(number, radix);
        } catch (NumberFormatException e) {
            try {
                return Long.valueOf(number, radix);
            } catch (NumberFormatException e2) {
                return new BigInteger(number, radix);
            }
        }
    }

    public class ConstructYamlFloat extends AbstractConstruct {
        public ConstructYamlFloat() {
        }

        public Object construct(Node node) {
            String value = SafeConstructor.this.constructScalar((ScalarNode) node).toString().replaceAll("_", "");
            int sign = 1;
            char first = value.charAt(0);
            int i = 1;
            if (first == '-') {
                sign = -1;
                value = value.substring(1);
            } else if (first == '+') {
                value = value.substring(1);
            }
            String valLower = value.toLowerCase();
            if (".inf".equals(valLower)) {
                return new Double(sign == -1 ? Double.NEGATIVE_INFINITY : Double.POSITIVE_INFINITY);
            } else if (".nan".equals(valLower)) {
                return new Double(Double.NaN);
            } else {
                if (value.indexOf(58) != -1) {
                    String[] digits = value.split(":");
                    int bes = 1;
                    double val = opencv_stitching.Stitcher.ORIG_RESOL;
                    int i2 = 0;
                    int j = digits.length;
                    while (i2 < j) {
                        double parseDouble = Double.parseDouble(digits[(j - i2) - i]);
                        double d = (double) bes;
                        Double.isNaN(d);
                        val += parseDouble * d;
                        bes *= 60;
                        i2++;
                        valLower = valLower;
                        i = 1;
                    }
                    double d2 = (double) sign;
                    Double.isNaN(d2);
                    return new Double(d2 * val);
                }
                double doubleValue = Double.valueOf(value).doubleValue();
                double d3 = (double) sign;
                Double.isNaN(d3);
                return new Double(doubleValue * d3);
            }
        }
    }

    public class ConstructYamlBinary extends AbstractConstruct {
        public ConstructYamlBinary() {
        }

        public Object construct(Node node) {
            return Base64Coder.decode(SafeConstructor.this.constructScalar((ScalarNode) node).toString().toCharArray());
        }
    }

    public static class ConstructYamlTimestamp extends AbstractConstruct {
        private Calendar calendar;

        public Calendar getCalendar() {
            return this.calendar;
        }

        public Object construct(Node node) {
            TimeZone timeZone;
            String time;
            ScalarNode scalar = (ScalarNode) node;
            String nodeValue = scalar.getValue();
            Matcher match = SafeConstructor.YMD_REGEXP.matcher(nodeValue);
            if (match.matches()) {
                String year_s = match.group(1);
                String month_s = match.group(2);
                String day_s = match.group(3);
                this.calendar = Calendar.getInstance(TimeZone.getTimeZone("UTC"));
                this.calendar.clear();
                this.calendar.set(1, Integer.parseInt(year_s));
                this.calendar.set(2, Integer.parseInt(month_s) - 1);
                this.calendar.set(5, Integer.parseInt(day_s));
                return this.calendar.getTime();
            }
            Matcher match2 = SafeConstructor.TIMESTAMP_REGEXP.matcher(nodeValue);
            if (match2.matches()) {
                String year_s2 = match2.group(1);
                String month_s2 = match2.group(2);
                String day_s2 = match2.group(3);
                String hour_s = match2.group(4);
                String min_s = match2.group(5);
                String seconds = match2.group(6);
                String millis = match2.group(7);
                if (millis != null) {
                    seconds = seconds + "." + millis;
                }
                double fractions = Double.parseDouble(seconds);
                int sec_s = (int) Math.round(Math.floor(fractions));
                double d = (double) sec_s;
                Double.isNaN(d);
                int usec = (int) Math.round((fractions - d) * 1000.0d);
                String timezoneh_s = match2.group(8);
                String timezonem_s = match2.group(9);
                if (timezoneh_s != null) {
                    if (timezonem_s != null) {
                        StringBuilder sb = new StringBuilder();
                        ScalarNode scalarNode = scalar;
                        sb.append(":");
                        sb.append(timezonem_s);
                        time = sb.toString();
                    } else {
                        time = "00";
                    }
                    StringBuilder sb2 = new StringBuilder();
                    Matcher matcher = match2;
                    sb2.append("GMT");
                    sb2.append(timezoneh_s);
                    sb2.append(time);
                    timeZone = TimeZone.getTimeZone(sb2.toString());
                } else {
                    Matcher matcher2 = match2;
                    timeZone = TimeZone.getTimeZone("UTC");
                }
                this.calendar = Calendar.getInstance(timeZone);
                TimeZone timeZone2 = timeZone;
                this.calendar.set(1, Integer.parseInt(year_s2));
                this.calendar.set(2, Integer.parseInt(month_s2) - 1);
                this.calendar.set(5, Integer.parseInt(day_s2));
                this.calendar.set(11, Integer.parseInt(hour_s));
                this.calendar.set(12, Integer.parseInt(min_s));
                this.calendar.set(13, sec_s);
                this.calendar.set(14, usec);
                return this.calendar.getTime();
            }
            Matcher matcher3 = match2;
            throw new YAMLException("Unexpected timestamp: " + nodeValue);
        }
    }

    public class ConstructYamlOmap extends AbstractConstruct {
        public ConstructYamlOmap() {
        }

        public Object construct(Node node) {
            Map<Object, Object> omap = new LinkedHashMap<>();
            if (node instanceof SequenceNode) {
                for (Node subnode : ((SequenceNode) node).getValue()) {
                    if (subnode instanceof MappingNode) {
                        MappingNode mnode = (MappingNode) subnode;
                        if (mnode.getValue().size() == 1) {
                            omap.put(SafeConstructor.this.constructObject(mnode.getValue().get(0).getKeyNode()), SafeConstructor.this.constructObject(mnode.getValue().get(0).getValueNode()));
                        } else {
                            Mark startMark = node.getStartMark();
                            throw new ConstructorException("while constructing an ordered map", startMark, "expected a single mapping item, but found " + mnode.getValue().size() + " items", mnode.getStartMark());
                        }
                    } else {
                        Mark startMark2 = node.getStartMark();
                        throw new ConstructorException("while constructing an ordered map", startMark2, "expected a mapping of length 1, but found " + subnode.getNodeId(), subnode.getStartMark());
                    }
                }
                return omap;
            }
            Mark startMark3 = node.getStartMark();
            throw new ConstructorException("while constructing an ordered map", startMark3, "expected a sequence, but found " + node.getNodeId(), node.getStartMark());
        }
    }

    public class ConstructYamlPairs extends AbstractConstruct {
        public ConstructYamlPairs() {
        }

        public Object construct(Node node) {
            if (node instanceof SequenceNode) {
                SequenceNode snode = (SequenceNode) node;
                List<Object[]> pairs = new ArrayList<>(snode.getValue().size());
                for (Node subnode : snode.getValue()) {
                    if (subnode instanceof MappingNode) {
                        MappingNode mnode = (MappingNode) subnode;
                        if (mnode.getValue().size() == 1) {
                            Node keyNode = mnode.getValue().get(0).getKeyNode();
                            Node valueNode = mnode.getValue().get(0).getValueNode();
                            pairs.add(new Object[]{SafeConstructor.this.constructObject(keyNode), SafeConstructor.this.constructObject(valueNode)});
                        } else {
                            Mark startMark = node.getStartMark();
                            throw new ConstructorException("while constructing pairs", startMark, "expected a single mapping item, but found " + mnode.getValue().size() + " items", mnode.getStartMark());
                        }
                    } else {
                        Mark startMark2 = node.getStartMark();
                        throw new ConstructorException("while constructingpairs", startMark2, "expected a mapping of length 1, but found " + subnode.getNodeId(), subnode.getStartMark());
                    }
                }
                return pairs;
            }
            Mark startMark3 = node.getStartMark();
            throw new ConstructorException("while constructing pairs", startMark3, "expected a sequence, but found " + node.getNodeId(), node.getStartMark());
        }
    }

    public class ConstructYamlSet implements Construct {
        public ConstructYamlSet() {
        }

        public Object construct(Node node) {
            if (node.isTwoStepsConstruction()) {
                return SafeConstructor.this.createDefaultSet();
            }
            return SafeConstructor.this.constructSet((MappingNode) node);
        }

        public void construct2ndStep(Node node, Object object) {
            if (node.isTwoStepsConstruction()) {
                SafeConstructor.this.constructSet2ndStep((MappingNode) node, (Set) object);
                return;
            }
            throw new YAMLException("Unexpected recursive set structure. Node: " + node);
        }
    }

    public class ConstructYamlStr extends AbstractConstruct {
        public ConstructYamlStr() {
        }

        public Object construct(Node node) {
            return SafeConstructor.this.constructScalar((ScalarNode) node);
        }
    }

    public class ConstructYamlSeq implements Construct {
        public ConstructYamlSeq() {
        }

        public Object construct(Node node) {
            SequenceNode seqNode = (SequenceNode) node;
            if (node.isTwoStepsConstruction()) {
                return SafeConstructor.this.createDefaultList(seqNode.getValue().size());
            }
            return SafeConstructor.this.constructSequence(seqNode);
        }

        public void construct2ndStep(Node node, Object data) {
            if (node.isTwoStepsConstruction()) {
                SafeConstructor.this.constructSequenceStep2((SequenceNode) node, (List) data);
                return;
            }
            throw new YAMLException("Unexpected recursive sequence structure. Node: " + node);
        }
    }

    public class ConstructYamlMap implements Construct {
        public ConstructYamlMap() {
        }

        public Object construct(Node node) {
            if (node.isTwoStepsConstruction()) {
                return SafeConstructor.this.createDefaultMap();
            }
            return SafeConstructor.this.constructMapping((MappingNode) node);
        }

        public void construct2ndStep(Node node, Object object) {
            if (node.isTwoStepsConstruction()) {
                SafeConstructor.this.constructMapping2ndStep((MappingNode) node, (Map) object);
                return;
            }
            throw new YAMLException("Unexpected recursive mapping structure. Node: " + node);
        }
    }

    public static final class ConstructUndefined extends AbstractConstruct {
        public Object construct(Node node) {
            throw new ConstructorException((String) null, (Mark) null, "could not determine a constructor for the tag " + node.getTag(), node.getStartMark());
        }
    }
}
