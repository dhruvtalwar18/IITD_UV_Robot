package org.bytedeco.javacpp.annotation;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Documented
@Target({ElementType.TYPE})
@Retention(RetentionPolicy.RUNTIME)
public @interface Properties {
    String global() default "";

    String helper() default "";

    Class[] inherit() default {};

    String[] names() default {};

    String target() default "";

    Platform[] value() default {};
}
