package org.yaml.snakeyaml;

import java.io.StringWriter;
import java.io.Writer;
import org.yaml.snakeyaml.DumperOptions;
import org.yaml.snakeyaml.introspector.BeanAccess;
import org.yaml.snakeyaml.nodes.Tag;
import org.yaml.snakeyaml.representer.Representer;

public class JavaBeanDumper {
    private final BeanAccess beanAccess;
    private DumperOptions.FlowStyle flowStyle;
    private DumperOptions options;
    private Representer representer;
    private boolean useGlobalTag;

    public JavaBeanDumper(boolean useGlobalTag2, BeanAccess beanAccess2) {
        this.useGlobalTag = useGlobalTag2;
        this.beanAccess = beanAccess2;
        this.flowStyle = DumperOptions.FlowStyle.BLOCK;
    }

    public JavaBeanDumper(boolean useGlobalTag2) {
        this(useGlobalTag2, BeanAccess.DEFAULT);
    }

    public JavaBeanDumper(BeanAccess beanAccess2) {
        this(false, beanAccess2);
    }

    public JavaBeanDumper() {
        this(BeanAccess.DEFAULT);
    }

    public JavaBeanDumper(Representer representer2, DumperOptions options2) {
        if (representer2 == null) {
            throw new NullPointerException("Representer must be provided.");
        } else if (options2 != null) {
            this.options = options2;
            this.representer = representer2;
            this.beanAccess = null;
        } else {
            throw new NullPointerException("DumperOptions must be provided.");
        }
    }

    public void dump(Object data, Writer output) {
        DumperOptions doptions;
        Representer repr;
        if (this.options == null) {
            doptions = new DumperOptions();
            if (!this.useGlobalTag) {
                doptions.setExplicitRoot(Tag.MAP);
            }
            doptions.setDefaultFlowStyle(this.flowStyle);
        } else {
            doptions = this.options;
        }
        if (this.representer == null) {
            repr = new Representer();
            repr.getPropertyUtils().setBeanAccess(this.beanAccess);
        } else {
            repr = this.representer;
        }
        new Yaml(repr, doptions).dump(data, output);
    }

    public String dump(Object data) {
        StringWriter buffer = new StringWriter();
        dump(data, buffer);
        return buffer.toString();
    }

    public boolean isUseGlobalTag() {
        return this.useGlobalTag;
    }

    public void setUseGlobalTag(boolean useGlobalTag2) {
        this.useGlobalTag = useGlobalTag2;
    }

    public DumperOptions.FlowStyle getFlowStyle() {
        return this.flowStyle;
    }

    public void setFlowStyle(DumperOptions.FlowStyle flowStyle2) {
        this.flowStyle = flowStyle2;
    }
}
