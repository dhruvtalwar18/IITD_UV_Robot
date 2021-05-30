package org.yaml.snakeyaml;

import java.io.IOException;
import java.io.InputStream;
import java.io.Reader;
import java.io.StringReader;
import java.io.StringWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.regex.Pattern;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.composer.Composer;
import org.yaml.snakeyaml.constructor.BaseConstructor;
import org.yaml.snakeyaml.constructor.Constructor;
import org.yaml.snakeyaml.emitter.Emitable;
import org.yaml.snakeyaml.emitter.Emitter;
import org.yaml.snakeyaml.error.YAMLException;
import org.yaml.snakeyaml.events.Event;
import org.yaml.snakeyaml.introspector.BeanAccess;
import org.yaml.snakeyaml.nodes.Node;
import org.yaml.snakeyaml.nodes.Tag;
import org.yaml.snakeyaml.parser.Parser;
import org.yaml.snakeyaml.parser.ParserImpl;
import org.yaml.snakeyaml.reader.StreamReader;
import org.yaml.snakeyaml.reader.UnicodeReader;
import org.yaml.snakeyaml.representer.Representer;
import org.yaml.snakeyaml.resolver.Resolver;
import org.yaml.snakeyaml.serializer.Serializer;

public class Yaml {
    protected BaseConstructor constructor;
    protected DumperOptions dumperOptions;
    private String name;
    protected Representer representer;
    protected final Resolver resolver;

    public Yaml() {
        this(new Constructor(), new Representer(), new DumperOptions(), new Resolver());
    }

    public Yaml(LoaderOptions loaderOptions) {
        this(new Constructor(), new Representer(), new DumperOptions(), new Resolver());
    }

    public Yaml(DumperOptions dumperOptions2) {
        this((BaseConstructor) new Constructor(), new Representer(), dumperOptions2);
    }

    public Yaml(Representer representer2) {
        this((BaseConstructor) new Constructor(), representer2);
    }

    public Yaml(BaseConstructor constructor2) {
        this(constructor2, new Representer());
    }

    public Yaml(BaseConstructor constructor2, Representer representer2) {
        this(constructor2, representer2, new DumperOptions());
    }

    public Yaml(Representer representer2, DumperOptions dumperOptions2) {
        this(new Constructor(), representer2, dumperOptions2, new Resolver());
    }

    public Yaml(BaseConstructor constructor2, Representer representer2, DumperOptions dumperOptions2) {
        this(constructor2, representer2, dumperOptions2, new Resolver());
    }

    public Yaml(BaseConstructor constructor2, Representer representer2, DumperOptions dumperOptions2, Resolver resolver2) {
        if (!constructor2.isExplicitPropertyUtils()) {
            constructor2.setPropertyUtils(representer2.getPropertyUtils());
        } else if (!representer2.isExplicitPropertyUtils()) {
            representer2.setPropertyUtils(constructor2.getPropertyUtils());
        }
        this.constructor = constructor2;
        representer2.setDefaultFlowStyle(dumperOptions2.getDefaultFlowStyle());
        representer2.setDefaultScalarStyle(dumperOptions2.getDefaultScalarStyle());
        representer2.getPropertyUtils().setAllowReadOnlyProperties(dumperOptions2.isAllowReadOnlyProperties());
        representer2.setTimeZone(dumperOptions2.getTimeZone());
        this.representer = representer2;
        this.dumperOptions = dumperOptions2;
        this.resolver = resolver2;
        this.name = "Yaml:" + System.identityHashCode(this);
    }

    public Yaml(BaseConstructor constructor2, LoaderOptions loaderOptions, Representer representer2, DumperOptions dumperOptions2, Resolver resolver2) {
        this(constructor2, representer2, dumperOptions2, resolver2);
    }

    public String dump(Object data) {
        List<Object> list = new ArrayList<>(1);
        list.add(data);
        return dumpAll(list.iterator());
    }

    public Node represent(Object data) {
        return this.representer.represent(data);
    }

    public String dumpAll(Iterator<? extends Object> data) {
        StringWriter buffer = new StringWriter();
        dumpAll(data, buffer);
        return buffer.toString();
    }

    public void dump(Object data, Writer output) {
        List<Object> list = new ArrayList<>(1);
        list.add(data);
        dumpAll(list.iterator(), output);
    }

    public void dumpAll(Iterator<? extends Object> data, Writer output) {
        dumpAll(data, output, this.dumperOptions.getExplicitRoot());
    }

    private void dumpAll(Iterator<? extends Object> data, Writer output, Tag rootTag) {
        Serializer serializer = new Serializer(new Emitter(output, this.dumperOptions), this.resolver, this.dumperOptions, rootTag);
        try {
            serializer.open();
            while (data.hasNext()) {
                serializer.serialize(this.representer.represent(data.next()));
            }
            serializer.close();
        } catch (IOException e) {
            throw new YAMLException((Throwable) e);
        }
    }

    public String dumpAs(Object data, Tag rootTag, DumperOptions.FlowStyle flowStyle) {
        DumperOptions.FlowStyle oldStyle = this.representer.getDefaultFlowStyle();
        if (flowStyle != null) {
            this.representer.setDefaultFlowStyle(flowStyle);
        }
        List<Object> list = new ArrayList<>(1);
        list.add(data);
        StringWriter buffer = new StringWriter();
        dumpAll(list.iterator(), buffer, rootTag);
        this.representer.setDefaultFlowStyle(oldStyle);
        return buffer.toString();
    }

    public String dumpAsMap(Object data) {
        return dumpAs(data, Tag.MAP, DumperOptions.FlowStyle.BLOCK);
    }

    public List<Event> serialize(Node data) {
        SilentEmitter emitter = new SilentEmitter();
        Serializer serializer = new Serializer(emitter, this.resolver, this.dumperOptions, this.dumperOptions.getExplicitRoot());
        try {
            serializer.open();
            serializer.serialize(data);
            serializer.close();
            return emitter.getEvents();
        } catch (IOException e) {
            throw new YAMLException((Throwable) e);
        }
    }

    private static class SilentEmitter implements Emitable {
        private List<Event> events;

        private SilentEmitter() {
            this.events = new ArrayList(100);
        }

        public List<Event> getEvents() {
            return this.events;
        }

        public void emit(Event event) throws IOException {
            this.events.add(event);
        }
    }

    public Object load(String yaml) {
        return loadFromReader(new StreamReader(yaml), Object.class);
    }

    public Object load(InputStream io) {
        return loadFromReader(new StreamReader((Reader) new UnicodeReader(io)), Object.class);
    }

    public Object load(Reader io) {
        return loadFromReader(new StreamReader(io), Object.class);
    }

    public <T> T loadAs(Reader io, Class<T> type) {
        return loadFromReader(new StreamReader(io), type);
    }

    public <T> T loadAs(String yaml, Class<T> type) {
        return loadFromReader(new StreamReader(yaml), type);
    }

    public <T> T loadAs(InputStream input, Class<T> type) {
        return loadFromReader(new StreamReader((Reader) new UnicodeReader(input)), type);
    }

    private Object loadFromReader(StreamReader sreader, Class<?> type) {
        this.constructor.setComposer(new Composer(new ParserImpl(sreader), this.resolver));
        return this.constructor.getSingleData(type);
    }

    public Iterable<Object> loadAll(Reader yaml) {
        this.constructor.setComposer(new Composer(new ParserImpl(new StreamReader(yaml)), this.resolver));
        return new YamlIterable(new Iterator<Object>() {
            public boolean hasNext() {
                return Yaml.this.constructor.checkData();
            }

            public Object next() {
                return Yaml.this.constructor.getData();
            }

            public void remove() {
                throw new UnsupportedOperationException();
            }
        });
    }

    private static class YamlIterable implements Iterable<Object> {
        private Iterator<Object> iterator;

        public YamlIterable(Iterator<Object> iterator2) {
            this.iterator = iterator2;
        }

        public Iterator<Object> iterator() {
            return this.iterator;
        }
    }

    public Iterable<Object> loadAll(String yaml) {
        return loadAll((Reader) new StringReader(yaml));
    }

    public Iterable<Object> loadAll(InputStream yaml) {
        return loadAll((Reader) new UnicodeReader(yaml));
    }

    public Node compose(Reader yaml) {
        Composer composer = new Composer(new ParserImpl(new StreamReader(yaml)), this.resolver);
        this.constructor.setComposer(composer);
        return composer.getSingleNode();
    }

    public Iterable<Node> composeAll(Reader yaml) {
        final Composer composer = new Composer(new ParserImpl(new StreamReader(yaml)), this.resolver);
        this.constructor.setComposer(composer);
        return new NodeIterable(new Iterator<Node>() {
            public boolean hasNext() {
                return composer.checkNode();
            }

            public Node next() {
                return composer.getNode();
            }

            public void remove() {
                throw new UnsupportedOperationException();
            }
        });
    }

    private static class NodeIterable implements Iterable<Node> {
        private Iterator<Node> iterator;

        public NodeIterable(Iterator<Node> iterator2) {
            this.iterator = iterator2;
        }

        public Iterator<Node> iterator() {
            return this.iterator;
        }
    }

    public void addImplicitResolver(String tag, Pattern regexp, String first) {
        addImplicitResolver(new Tag(tag), regexp, first);
    }

    public void addImplicitResolver(Tag tag, Pattern regexp, String first) {
        this.resolver.addImplicitResolver(tag, regexp, first);
    }

    public String toString() {
        return this.name;
    }

    public String getName() {
        return this.name;
    }

    public void setName(String name2) {
        this.name = name2;
    }

    public Iterable<Event> parse(Reader yaml) {
        final Parser parser = new ParserImpl(new StreamReader(yaml));
        return new EventIterable(new Iterator<Event>() {
            public boolean hasNext() {
                return parser.peekEvent() != null;
            }

            public Event next() {
                return parser.getEvent();
            }

            public void remove() {
                throw new UnsupportedOperationException();
            }
        });
    }

    private static class EventIterable implements Iterable<Event> {
        private Iterator<Event> iterator;

        public EventIterable(Iterator<Event> iterator2) {
            this.iterator = iterator2;
        }

        public Iterator<Event> iterator() {
            return this.iterator;
        }
    }

    public void setBeanAccess(BeanAccess beanAccess) {
        this.constructor.getPropertyUtils().setBeanAccess(beanAccess);
        this.representer.getPropertyUtils().setBeanAccess(beanAccess);
    }

    public Yaml(Loader loader) {
        this(loader, new Dumper(new DumperOptions()));
    }

    public Yaml(Loader loader, Dumper dumper) {
        this(loader, dumper, new Resolver());
    }

    public Yaml(Loader loader, Dumper dumper, Resolver resolver2) {
        this(loader.constructor, dumper.representer, dumper.options, resolver2);
    }

    public Yaml(Dumper dumper) {
        this((BaseConstructor) new Constructor(), dumper.representer, dumper.options);
    }
}
