package org.yaml.snakeyaml;

import java.io.InputStream;
import java.io.Reader;
import java.io.StringReader;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.introspector.BeanAccess;
import org.yaml.snakeyaml.reader.UnicodeReader;
import org.yaml.snakeyaml.representer.Representer;
import org.yaml.snakeyaml.resolver.Resolver;

public class JavaBeanLoader<T> {
    private Yaml loader;

    public JavaBeanLoader(TypeDescription typeDescription) {
        this(typeDescription, BeanAccess.DEFAULT);
    }

    public JavaBeanLoader(TypeDescription typeDescription, BeanAccess beanAccess) {
        this(new LoaderOptions(typeDescription), beanAccess);
    }

    public JavaBeanLoader(LoaderOptions options, BeanAccess beanAccess) {
        if (options == null) {
            throw new NullPointerException("LoaderOptions must be provided.");
        } else if (options.getRootTypeDescription() != null) {
            this.loader = new Yaml(new Constructor(options.getRootTypeDescription()), options, new Representer(), new DumperOptions(), new Resolver());
            this.loader.setBeanAccess(beanAccess);
        } else {
            throw new NullPointerException("TypeDescription must be provided.");
        }
    }

    public <S extends T> JavaBeanLoader(Class<S> clazz, BeanAccess beanAccess) {
        this(new TypeDescription(clazz), beanAccess);
    }

    public <S extends T> JavaBeanLoader(Class<S> clazz) {
        this(clazz, BeanAccess.DEFAULT);
    }

    public T load(String yaml) {
        return this.loader.load((Reader) new StringReader(yaml));
    }

    public T load(InputStream io) {
        return this.loader.load((Reader) new UnicodeReader(io));
    }

    public T load(Reader io) {
        return this.loader.load(io);
    }
}
