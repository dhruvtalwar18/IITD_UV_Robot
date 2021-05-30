package org.jboss.netty.handler.codec.spdy;

import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import org.jboss.netty.util.internal.StringUtil;

public class DefaultSpdySettingsFrame implements SpdySettingsFrame {
    private boolean clear;
    private final Map<Integer, Setting> settingsMap = new TreeMap();

    public Set<Integer> getIDs() {
        return getIds();
    }

    public Set<Integer> getIds() {
        return this.settingsMap.keySet();
    }

    public boolean isSet(int ID) {
        return this.settingsMap.containsKey(new Integer(ID));
    }

    public int getValue(int ID) {
        Integer key = new Integer(ID);
        if (this.settingsMap.containsKey(key)) {
            return this.settingsMap.get(key).getValue();
        }
        return -1;
    }

    public void setValue(int ID, int value) {
        setValue(ID, value, false, false);
    }

    public void setValue(int ID, int value, boolean persistValue, boolean persisted) {
        if (ID <= 0 || ID > 16777215) {
            throw new IllegalArgumentException("Setting ID is not valid: " + ID);
        }
        Integer key = new Integer(ID);
        if (this.settingsMap.containsKey(key)) {
            Setting setting = this.settingsMap.get(key);
            setting.setValue(value);
            setting.setPersist(persistValue);
            setting.setPersisted(persisted);
            return;
        }
        this.settingsMap.put(key, new Setting(value, persistValue, persisted));
    }

    public void removeValue(int ID) {
        Integer key = new Integer(ID);
        if (this.settingsMap.containsKey(key)) {
            this.settingsMap.remove(key);
        }
    }

    public boolean persistValue(int ID) {
        return isPersistValue(ID);
    }

    public boolean isPersistValue(int ID) {
        Integer key = new Integer(ID);
        if (this.settingsMap.containsKey(key)) {
            return this.settingsMap.get(key).isPersist();
        }
        return false;
    }

    public void setPersistValue(int ID, boolean persistValue) {
        Integer key = new Integer(ID);
        if (this.settingsMap.containsKey(key)) {
            this.settingsMap.get(key).setPersist(persistValue);
        }
    }

    public boolean isPersisted(int ID) {
        Integer key = new Integer(ID);
        if (this.settingsMap.containsKey(key)) {
            return this.settingsMap.get(key).isPersisted();
        }
        return false;
    }

    public void setPersisted(int ID, boolean persisted) {
        Integer key = new Integer(ID);
        if (this.settingsMap.containsKey(key)) {
            this.settingsMap.get(key).setPersisted(persisted);
        }
    }

    public boolean clearPreviouslyPersistedSettings() {
        return this.clear;
    }

    public void setClearPreviouslyPersistedSettings(boolean clear2) {
        this.clear = clear2;
    }

    private Set<Map.Entry<Integer, Setting>> getSettings() {
        return this.settingsMap.entrySet();
    }

    private void appendSettings(StringBuilder buf) {
        for (Map.Entry<Integer, Setting> e : getSettings()) {
            Setting setting = e.getValue();
            buf.append("--> ");
            buf.append(e.getKey().toString());
            buf.append(":");
            buf.append(setting.getValue());
            buf.append(" (persist value: ");
            buf.append(setting.isPersist());
            buf.append("; persisted: ");
            buf.append(setting.isPersisted());
            buf.append(')');
            buf.append(StringUtil.NEWLINE);
        }
    }

    public String toString() {
        StringBuilder buf = new StringBuilder();
        buf.append(getClass().getSimpleName());
        buf.append(StringUtil.NEWLINE);
        appendSettings(buf);
        buf.setLength(buf.length() - StringUtil.NEWLINE.length());
        return buf.toString();
    }

    private static final class Setting {
        private boolean persist;
        private boolean persisted;
        private int value;

        Setting(int value2, boolean persist2, boolean persisted2) {
            this.value = value2;
            this.persist = persist2;
            this.persisted = persisted2;
        }

        /* access modifiers changed from: package-private */
        public int getValue() {
            return this.value;
        }

        /* access modifiers changed from: package-private */
        public void setValue(int value2) {
            this.value = value2;
        }

        /* access modifiers changed from: package-private */
        public boolean isPersist() {
            return this.persist;
        }

        /* access modifiers changed from: package-private */
        public void setPersist(boolean persist2) {
            this.persist = persist2;
        }

        /* access modifiers changed from: package-private */
        public boolean isPersisted() {
            return this.persisted;
        }

        /* access modifiers changed from: package-private */
        public void setPersisted(boolean persisted2) {
            this.persisted = persisted2;
        }
    }
}
