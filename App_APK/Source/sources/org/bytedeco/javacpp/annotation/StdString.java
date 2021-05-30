package org.bytedeco.javacpp.annotation;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Documented
@Target({ElementType.METHOD, ElementType.PARAMETER})
@Cast({"std::basic_string", "&"})
@Adapter("StringAdapter")
@Retention(RetentionPolicy.RUNTIME)
public @interface StdString {
    String value() default "char";
}
