package org.yaml.snakeyaml.introspector;

import java.lang.reflect.Field;
import org.yaml.snakeyaml.error.YAMLException;

public class FieldProperty extends GenericProperty {
    private final Field field;

    public FieldProperty(Field field2) {
        super(field2.getName(), field2.getType(), field2.getGenericType());
        this.field = field2;
        field2.setAccessible(true);
    }

    public void set(Object object, Object value) throws Exception {
        this.field.set(object, value);
    }

    public Object get(Object object) {
        try {
            return this.field.get(object);
        } catch (Exception e) {
            throw new YAMLException("Unable to access field " + this.field.getName() + " on object " + object + " : " + e);
        }
    }
}
