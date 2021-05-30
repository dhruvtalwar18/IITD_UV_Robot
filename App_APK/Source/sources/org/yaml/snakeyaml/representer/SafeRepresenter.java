package org.yaml.snakeyaml.representer;

import java.io.UnsupportedEncodingException;
import java.math.BigInteger;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TimeZone;
import java.util.regex.Pattern;
import org.apache.commons.httpclient.HttpState;
import org.apache.commons.lang.time.DateUtils;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.external.biz.base64Coder.Base64Coder;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.Tag;

class SafeRepresenter extends BaseRepresenter {
    public static Pattern BINARY_PATTERN = Pattern.compile("[\\x00-\\x08\\x0B\\x0C\\x0E-\\x1F]");
    public static Pattern MULTILINE_PATTERN = Pattern.compile("\n|| | ");
    protected Map<Class<? extends Object>, Tag> classTags;
    protected TimeZone timeZone = null;

    public SafeRepresenter() {
        this.nullRepresenter = new RepresentNull();
        this.representers.put(String.class, new RepresentString());
        this.representers.put(Boolean.class, new RepresentBoolean());
        this.representers.put(Character.class, new RepresentString());
        this.representers.put(byte[].class, new RepresentByteArray());
        this.multiRepresenters.put(Number.class, new RepresentNumber());
        this.multiRepresenters.put(List.class, new RepresentList());
        this.multiRepresenters.put(Map.class, new RepresentMap());
        this.multiRepresenters.put(Set.class, new RepresentSet());
        this.multiRepresenters.put(Iterator.class, new RepresentIterator());
        this.multiRepresenters.put(new Object[0].getClass(), new RepresentArray());
        this.multiRepresenters.put(Date.class, new RepresentDate());
        this.multiRepresenters.put(Enum.class, new RepresentEnum());
        this.multiRepresenters.put(Calendar.class, new RepresentDate());
        this.classTags = new HashMap();
    }

    /* access modifiers changed from: protected */
    public Tag getTag(Class<?> clazz, Tag defaultTag) {
        if (this.classTags.containsKey(clazz)) {
            return this.classTags.get(clazz);
        }
        return defaultTag;
    }

    public Tag addClassTag(Class<? extends Object> clazz, String tag) {
        return addClassTag(clazz, new Tag(tag));
    }

    public Tag addClassTag(Class<? extends Object> clazz, Tag tag) {
        if (tag != null) {
            return this.classTags.put(clazz, tag);
        }
        throw new NullPointerException("Tag must be provided.");
    }

    protected class RepresentNull implements Represent {
        protected RepresentNull() {
        }

        public Node representData(Object data) {
            return SafeRepresenter.this.representScalar(Tag.NULL, "null");
        }
    }

    protected class RepresentString implements Represent {
        protected RepresentString() {
        }

        public Node representData(Object data) {
            Tag tag = Tag.STR;
            char style = null;
            String value = data.toString();
            if (SafeRepresenter.BINARY_PATTERN.matcher(value).find()) {
                tag = Tag.BINARY;
                try {
                    value = String.valueOf(Base64Coder.encode(value.getBytes("UTF-8")));
                    style = '|';
                } catch (UnsupportedEncodingException e) {
                    throw new YAMLException((Throwable) e);
                }
            }
            if (SafeRepresenter.this.defaultScalarStyle == null && SafeRepresenter.MULTILINE_PATTERN.matcher(value).find()) {
                style = '|';
            }
            return SafeRepresenter.this.representScalar(tag, value, style);
        }
    }

    protected class RepresentBoolean implements Represent {
        protected RepresentBoolean() {
        }

        public Node representData(Object data) {
            String value;
            if (Boolean.TRUE.equals(data)) {
                value = "true";
            } else {
                value = HttpState.PREEMPTIVE_DEFAULT;
            }
            return SafeRepresenter.this.representScalar(Tag.BOOL, value);
        }
    }

    protected class RepresentNumber implements Represent {
        protected RepresentNumber() {
        }

        public Node representData(Object data) {
            String value;
            Tag tag;
            if ((data instanceof Byte) || (data instanceof Short) || (data instanceof Integer) || (data instanceof Long) || (data instanceof BigInteger)) {
                tag = Tag.INT;
                value = data.toString();
            } else {
                Number number = (Number) data;
                tag = Tag.FLOAT;
                if (number.equals(Double.valueOf(Double.NaN))) {
                    value = ".NaN";
                } else if (number.equals(Double.valueOf(Double.POSITIVE_INFINITY))) {
                    value = ".inf";
                } else if (number.equals(Double.valueOf(Double.NEGATIVE_INFINITY))) {
                    value = "-.inf";
                } else {
                    value = number.toString();
                }
            }
            return SafeRepresenter.this.representScalar(SafeRepresenter.this.getTag(data.getClass(), tag), value);
        }
    }

    protected class RepresentList implements Represent {
        protected RepresentList() {
        }

        public Node representData(Object data) {
            return SafeRepresenter.this.representSequence(SafeRepresenter.this.getTag(data.getClass(), Tag.SEQ), (List) data, (Boolean) null);
        }
    }

    protected class RepresentIterator implements Represent {
        protected RepresentIterator() {
        }

        public Node representData(Object data) {
            return SafeRepresenter.this.representSequence(SafeRepresenter.this.getTag(data.getClass(), Tag.SEQ), new IteratorWrapper((Iterator) data), (Boolean) null);
        }
    }

    private static class IteratorWrapper implements Iterable<Object> {
        private Iterator<Object> iter;

        public IteratorWrapper(Iterator<Object> iter2) {
            this.iter = iter2;
        }

        public Iterator<Object> iterator() {
            return this.iter;
        }
    }

    protected class RepresentArray implements Represent {
        protected RepresentArray() {
        }

        public Node representData(Object data) {
            return SafeRepresenter.this.representSequence(Tag.SEQ, Arrays.asList((Object[]) data), (Boolean) null);
        }
    }

    protected class RepresentMap implements Represent {
        protected RepresentMap() {
        }

        public Node representData(Object data) {
            return SafeRepresenter.this.representMapping(SafeRepresenter.this.getTag(data.getClass(), Tag.MAP), (Map) data, (Boolean) null);
        }
    }

    protected class RepresentSet implements Represent {
        protected RepresentSet() {
        }

        public Node representData(Object data) {
            Map<Object, Object> value = new LinkedHashMap<>();
            for (Object key : (Set) data) {
                value.put(key, (Object) null);
            }
            return SafeRepresenter.this.representMapping(SafeRepresenter.this.getTag(data.getClass(), Tag.SET), value, (Boolean) null);
        }
    }

    protected class RepresentDate implements Represent {
        protected RepresentDate() {
        }

        public Node representData(Object data) {
            Calendar calendar;
            Object obj;
            Object obj2 = data;
            if (obj2 instanceof Calendar) {
                calendar = (Calendar) obj2;
            } else {
                calendar = Calendar.getInstance(SafeRepresenter.this.getTimeZone() == null ? TimeZone.getTimeZone("UTC") : SafeRepresenter.this.timeZone);
                calendar.setTime((Date) obj2);
            }
            int years = calendar.get(1);
            int months = calendar.get(2) + 1;
            int days = calendar.get(5);
            int hour24 = calendar.get(11);
            int minutes = calendar.get(12);
            int seconds = calendar.get(13);
            int millis = calendar.get(14);
            StringBuilder buffer = new StringBuilder(String.valueOf(years));
            while (buffer.length() < 4) {
                buffer.insert(0, "0");
            }
            buffer.append("-");
            if (months < 10) {
                buffer.append("0");
            }
            buffer.append(String.valueOf(months));
            buffer.append("-");
            if (days < 10) {
                buffer.append("0");
            }
            buffer.append(String.valueOf(days));
            buffer.append("T");
            if (hour24 < 10) {
                buffer.append("0");
            }
            buffer.append(String.valueOf(hour24));
            buffer.append(":");
            if (minutes < 10) {
                buffer.append("0");
            }
            buffer.append(String.valueOf(minutes));
            buffer.append(":");
            if (seconds < 10) {
                buffer.append("0");
            }
            buffer.append(String.valueOf(seconds));
            if (millis > 0) {
                if (millis < 10) {
                    buffer.append(".00");
                } else if (millis < 100) {
                    buffer.append(".0");
                } else {
                    buffer.append(".");
                }
                buffer.append(String.valueOf(millis));
            }
            if (TimeZone.getTimeZone("UTC").equals(calendar.getTimeZone())) {
                buffer.append("Z");
                Calendar calendar2 = calendar;
            } else {
                int minutesOffset = calendar.getTimeZone().getOffset(calendar.get(0), calendar.get(1), calendar.get(2), calendar.get(5), calendar.get(7), calendar.get(14)) / DateUtils.MILLIS_IN_MINUTE;
                int hoursOffset = minutesOffset / 60;
                int partOfHour = minutesOffset % 60;
                StringBuilder sb = new StringBuilder();
                sb.append(hoursOffset > 0 ? "+" : "");
                sb.append(hoursOffset);
                sb.append(":");
                if (partOfHour < 10) {
                    StringBuilder sb2 = new StringBuilder();
                    Calendar calendar3 = calendar;
                    sb2.append("0");
                    sb2.append(partOfHour);
                    obj = sb2.toString();
                } else {
                    obj = Integer.valueOf(partOfHour);
                }
                sb.append(obj);
                buffer.append(sb.toString());
            }
            return SafeRepresenter.this.representScalar(SafeRepresenter.this.getTag(data.getClass(), Tag.TIMESTAMP), buffer.toString(), (Character) null);
        }
    }

    protected class RepresentEnum implements Represent {
        protected RepresentEnum() {
        }

        public Node representData(Object data) {
            return SafeRepresenter.this.representScalar(SafeRepresenter.this.getTag(data.getClass(), new Tag((Class<? extends Object>) data.getClass())), ((Enum) data).name());
        }
    }

    protected class RepresentByteArray implements Represent {
        protected RepresentByteArray() {
        }

        public Node representData(Object data) {
            return SafeRepresenter.this.representScalar(Tag.BINARY, String.valueOf(Base64Coder.encode((byte[]) data)), '|');
        }
    }

    public TimeZone getTimeZone() {
        return this.timeZone;
    }

    public void setTimeZone(TimeZone timeZone2) {
        this.timeZone = timeZone2;
    }
}
