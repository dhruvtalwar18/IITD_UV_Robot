package org.apache.commons.lang.enums;

import java.io.Serializable;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.WeakHashMap;
import org.apache.commons.lang.ClassUtils;
import org.apache.commons.lang.StringUtils;

public abstract class Enum implements Comparable, Serializable {
    private static final Map EMPTY_MAP = Collections.unmodifiableMap(new HashMap(0));
    private static Map cEnumClasses = new WeakHashMap();
    static /* synthetic */ Class class$org$apache$commons$lang$enums$Enum = null;
    static /* synthetic */ Class class$org$apache$commons$lang$enums$ValuedEnum = null;
    private static final long serialVersionUID = -487045951170455942L;
    private final transient int iHashCode;
    private final String iName;
    protected transient String iToString = null;

    private static class Entry {
        final List list = new ArrayList(25);
        final Map map = new HashMap();
        final List unmodifiableList = Collections.unmodifiableList(this.list);
        final Map unmodifiableMap = Collections.unmodifiableMap(this.map);

        protected Entry() {
        }
    }

    protected Enum(String name) {
        init(name);
        this.iName = name;
        this.iHashCode = getEnumClass().hashCode() + 7 + (name.hashCode() * 3);
    }

    private void init(String name) {
        Class cls;
        Entry entry;
        Class cls2;
        Class cls3;
        if (!StringUtils.isEmpty(name)) {
            Class enumClass = getEnumClass();
            if (enumClass != null) {
                Class cls4 = getClass();
                boolean ok = false;
                while (true) {
                    if (cls4 == null) {
                        break;
                    }
                    if (class$org$apache$commons$lang$enums$Enum == null) {
                        cls2 = class$("org.apache.commons.lang.enums.Enum");
                        class$org$apache$commons$lang$enums$Enum = cls2;
                    } else {
                        cls2 = class$org$apache$commons$lang$enums$Enum;
                    }
                    if (cls4 == cls2) {
                        break;
                    }
                    if (class$org$apache$commons$lang$enums$ValuedEnum == null) {
                        cls3 = class$("org.apache.commons.lang.enums.ValuedEnum");
                        class$org$apache$commons$lang$enums$ValuedEnum = cls3;
                    } else {
                        cls3 = class$org$apache$commons$lang$enums$ValuedEnum;
                    }
                    if (cls4 == cls3) {
                        break;
                    } else if (cls4 == enumClass) {
                        ok = true;
                        break;
                    } else {
                        cls4 = cls4.getSuperclass();
                    }
                }
                if (ok) {
                    if (class$org$apache$commons$lang$enums$Enum == null) {
                        cls = class$("org.apache.commons.lang.enums.Enum");
                        class$org$apache$commons$lang$enums$Enum = cls;
                    } else {
                        cls = class$org$apache$commons$lang$enums$Enum;
                    }
                    synchronized (cls) {
                        entry = (Entry) cEnumClasses.get(enumClass);
                        if (entry == null) {
                            entry = createEntry(enumClass);
                            Map myMap = new WeakHashMap();
                            myMap.putAll(cEnumClasses);
                            myMap.put(enumClass, entry);
                            cEnumClasses = myMap;
                        }
                    }
                    Entry entry2 = entry;
                    if (!entry2.map.containsKey(name)) {
                        entry2.map.put(name, this);
                        entry2.list.add(this);
                        return;
                    }
                    StringBuffer stringBuffer = new StringBuffer();
                    stringBuffer.append("The Enum name must be unique, '");
                    stringBuffer.append(name);
                    stringBuffer.append("' has already been added");
                    throw new IllegalArgumentException(stringBuffer.toString());
                }
                throw new IllegalArgumentException("getEnumClass() must return a superclass of this class");
            }
            throw new IllegalArgumentException("getEnumClass() must not be null");
        }
        throw new IllegalArgumentException("The Enum name must not be empty or null");
    }

    static /* synthetic */ Class class$(String x0) {
        try {
            return Class.forName(x0);
        } catch (ClassNotFoundException x1) {
            throw new NoClassDefFoundError(x1.getMessage());
        }
    }

    /* access modifiers changed from: protected */
    public Object readResolve() {
        Entry entry = (Entry) cEnumClasses.get(getEnumClass());
        if (entry == null) {
            return null;
        }
        return entry.map.get(getName());
    }

    protected static Enum getEnum(Class enumClass, String name) {
        Entry entry = getEntry(enumClass);
        if (entry == null) {
            return null;
        }
        return (Enum) entry.map.get(name);
    }

    protected static Map getEnumMap(Class enumClass) {
        Entry entry = getEntry(enumClass);
        if (entry == null) {
            return EMPTY_MAP;
        }
        return entry.unmodifiableMap;
    }

    protected static List getEnumList(Class enumClass) {
        Entry entry = getEntry(enumClass);
        if (entry == null) {
            return Collections.EMPTY_LIST;
        }
        return entry.unmodifiableList;
    }

    protected static Iterator iterator(Class enumClass) {
        return getEnumList(enumClass).iterator();
    }

    private static Entry getEntry(Class enumClass) {
        Class cls;
        if (enumClass != null) {
            if (class$org$apache$commons$lang$enums$Enum == null) {
                cls = class$("org.apache.commons.lang.enums.Enum");
                class$org$apache$commons$lang$enums$Enum = cls;
            } else {
                cls = class$org$apache$commons$lang$enums$Enum;
            }
            if (cls.isAssignableFrom(enumClass)) {
                return (Entry) cEnumClasses.get(enumClass);
            }
            throw new IllegalArgumentException("The Class must be a subclass of Enum");
        }
        throw new IllegalArgumentException("The Enum Class must not be null");
    }

    private static Entry createEntry(Class enumClass) {
        Class cls;
        Class cls2;
        Entry entry = new Entry();
        Class cls3 = enumClass.getSuperclass();
        while (true) {
            if (cls3 == null) {
                break;
            }
            if (class$org$apache$commons$lang$enums$Enum == null) {
                cls = class$("org.apache.commons.lang.enums.Enum");
                class$org$apache$commons$lang$enums$Enum = cls;
            } else {
                cls = class$org$apache$commons$lang$enums$Enum;
            }
            if (cls3 == cls) {
                break;
            }
            if (class$org$apache$commons$lang$enums$ValuedEnum == null) {
                cls2 = class$("org.apache.commons.lang.enums.ValuedEnum");
                class$org$apache$commons$lang$enums$ValuedEnum = cls2;
            } else {
                cls2 = class$org$apache$commons$lang$enums$ValuedEnum;
            }
            if (cls3 == cls2) {
                break;
            }
            Entry loopEntry = (Entry) cEnumClasses.get(cls3);
            if (loopEntry != null) {
                entry.list.addAll(loopEntry.list);
                entry.map.putAll(loopEntry.map);
                break;
            }
            cls3 = cls3.getSuperclass();
        }
        return entry;
    }

    public final String getName() {
        return this.iName;
    }

    public Class getEnumClass() {
        return getClass();
    }

    public final boolean equals(Object other) {
        if (other == this) {
            return true;
        }
        if (other == null) {
            return false;
        }
        if (other.getClass() == getClass()) {
            return this.iName.equals(((Enum) other).iName);
        }
        if (!other.getClass().getName().equals(getClass().getName())) {
            return false;
        }
        return this.iName.equals(getNameInOtherClassLoader(other));
    }

    public final int hashCode() {
        return this.iHashCode;
    }

    public int compareTo(Object other) {
        if (other == this) {
            return 0;
        }
        if (other.getClass() == getClass()) {
            return this.iName.compareTo(((Enum) other).iName);
        }
        if (other.getClass().getName().equals(getClass().getName())) {
            return this.iName.compareTo(getNameInOtherClassLoader(other));
        }
        StringBuffer stringBuffer = new StringBuffer();
        stringBuffer.append("Different enum class '");
        stringBuffer.append(ClassUtils.getShortClassName((Class) other.getClass()));
        stringBuffer.append("'");
        throw new ClassCastException(stringBuffer.toString());
    }

    private String getNameInOtherClassLoader(Object other) {
        try {
            return (String) other.getClass().getMethod("getName", (Class[]) null).invoke(other, (Object[]) null);
        } catch (IllegalAccessException | NoSuchMethodException | InvocationTargetException e) {
            throw new IllegalStateException("This should not happen");
        }
    }

    public String toString() {
        if (this.iToString == null) {
            String shortName = ClassUtils.getShortClassName(getEnumClass());
            StringBuffer stringBuffer = new StringBuffer();
            stringBuffer.append(shortName);
            stringBuffer.append("[");
            stringBuffer.append(getName());
            stringBuffer.append("]");
            this.iToString = stringBuffer.toString();
        }
        return this.iToString;
    }
}
