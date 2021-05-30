package com.google.common.reflect;

import com.google.common.base.Joiner;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.google.common.reflect.Types;
import java.lang.reflect.GenericArrayType;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.lang.reflect.TypeVariable;
import java.lang.reflect.WildcardType;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import javax.annotation.Nullable;

class TypeResolver {
    private final ImmutableMap<TypeVariable<?>, Type> typeTable;

    static TypeResolver accordingTo(Type type) {
        return new TypeResolver().where(TypeMappingIntrospector.getTypeMappings(type));
    }

    TypeResolver() {
        this.typeTable = ImmutableMap.of();
    }

    private TypeResolver(ImmutableMap<TypeVariable<?>, Type> typeTable2) {
        this.typeTable = typeTable2;
    }

    /* access modifiers changed from: package-private */
    public final TypeResolver where(Map<? extends TypeVariable<?>, ? extends Type> mappings) {
        ImmutableMap.Builder<TypeVariable<?>, Type> builder = ImmutableMap.builder();
        builder.putAll(this.typeTable);
        for (Map.Entry<? extends TypeVariable<?>, ? extends Type> mapping : mappings.entrySet()) {
            TypeVariable<?> variable = (TypeVariable) mapping.getKey();
            Type type = (Type) mapping.getValue();
            Preconditions.checkArgument(!variable.equals(type), "Type variable %s bound to itself", variable);
            builder.put(variable, type);
        }
        return new TypeResolver(builder.build());
    }

    /* access modifiers changed from: package-private */
    public final TypeResolver where(Type mapFrom, Type mapTo) {
        Map<TypeVariable<?>, Type> mappings = Maps.newHashMap();
        populateTypeMappings(mappings, mapFrom, mapTo);
        return where(mappings);
    }

    private static void populateTypeMappings(Map<TypeVariable<?>, Type> mappings, Type from, Type to) {
        if (from instanceof TypeVariable) {
            mappings.put((TypeVariable) from, to);
        } else if (from instanceof GenericArrayType) {
            populateTypeMappings(mappings, ((GenericArrayType) from).getGenericComponentType(), Types.getComponentType(to));
        } else if (from instanceof ParameterizedType) {
            Type[] fromArgs = ((ParameterizedType) from).getActualTypeArguments();
            Type[] toArgs = ((ParameterizedType) to).getActualTypeArguments();
            int i = 0;
            Preconditions.checkArgument(fromArgs.length == toArgs.length);
            while (true) {
                int i2 = i;
                if (i2 < fromArgs.length) {
                    populateTypeMappings(mappings, fromArgs[i2], toArgs[i2]);
                    i = i2 + 1;
                } else {
                    return;
                }
            }
        }
    }

    /* access modifiers changed from: package-private */
    public final Type resolve(Type type) {
        if (type instanceof TypeVariable) {
            return resolveTypeVariable((TypeVariable) type);
        }
        if (type instanceof ParameterizedType) {
            return resolveParameterizedType((ParameterizedType) type);
        }
        if (type instanceof GenericArrayType) {
            return resolveGenericArrayType((GenericArrayType) type);
        }
        if (!(type instanceof WildcardType)) {
            return type;
        }
        WildcardType wildcardType = (WildcardType) type;
        return new Types.WildcardTypeImpl(resolve(wildcardType.getLowerBounds()), resolve(wildcardType.getUpperBounds()));
    }

    private Type[] resolve(Type[] types) {
        Type[] result = new Type[types.length];
        for (int i = 0; i < types.length; i++) {
            result[i] = resolve(types[i]);
        }
        return result;
    }

    private Type resolveGenericArrayType(GenericArrayType type) {
        return Types.newArrayType(resolve(type.getGenericComponentType()));
    }

    private Type resolveTypeVariable(final TypeVariable<?> var) {
        return resolveTypeVariable(var, new TypeResolver(this.typeTable) {
            /* access modifiers changed from: package-private */
            public Type resolveTypeVariable(TypeVariable<?> intermediateVar, TypeResolver guardedResolver) {
                if (intermediateVar.getGenericDeclaration().equals(var.getGenericDeclaration())) {
                    return intermediateVar;
                }
                return this.resolveTypeVariable(intermediateVar, guardedResolver);
            }
        });
    }

    /* JADX WARNING: type inference failed for: r6v0, types: [java.lang.reflect.Type, java.lang.reflect.TypeVariable, java.lang.reflect.TypeVariable<?>, java.lang.Object] */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Unknown variable types count: 1 */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.lang.reflect.Type resolveTypeVariable(java.lang.reflect.TypeVariable<?> r6, com.google.common.reflect.TypeResolver r7) {
        /*
            r5 = this;
            com.google.common.collect.ImmutableMap<java.lang.reflect.TypeVariable<?>, java.lang.reflect.Type> r0 = r5.typeTable
            java.lang.Object r0 = r0.get(r6)
            java.lang.reflect.Type r0 = (java.lang.reflect.Type) r0
            if (r0 != 0) goto L_0x0023
            java.lang.reflect.Type[] r1 = r6.getBounds()
            int r2 = r1.length
            if (r2 != 0) goto L_0x0012
            return r6
        L_0x0012:
            java.lang.reflect.GenericDeclaration r2 = r6.getGenericDeclaration()
            java.lang.String r3 = r6.getName()
            java.lang.reflect.Type[] r4 = r7.resolve((java.lang.reflect.Type[]) r1)
            java.lang.reflect.TypeVariable r2 = com.google.common.reflect.Types.newTypeVariable(r2, r3, r4)
            return r2
        L_0x0023:
            java.lang.reflect.Type r1 = r7.resolve((java.lang.reflect.Type) r0)
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.google.common.reflect.TypeResolver.resolveTypeVariable(java.lang.reflect.TypeVariable, com.google.common.reflect.TypeResolver):java.lang.reflect.Type");
    }

    private ParameterizedType resolveParameterizedType(ParameterizedType type) {
        Type owner = type.getOwnerType();
        Type resolvedOwner = owner == null ? null : resolve(owner);
        Type resolvedRawType = resolve(type.getRawType());
        Type[] vars = type.getActualTypeArguments();
        Type[] resolvedArgs = new Type[vars.length];
        for (int i = 0; i < vars.length; i++) {
            resolvedArgs[i] = resolve(vars[i]);
        }
        return Types.newParameterizedTypeWithOwner(resolvedOwner, (Class) resolvedRawType, resolvedArgs);
    }

    private static final class TypeMappingIntrospector {
        private static final WildcardCapturer wildcardCapturer = new WildcardCapturer();
        private final Set<Type> introspectedTypes = Sets.newHashSet();
        private final Map<TypeVariable<?>, Type> mappings = Maps.newHashMap();

        private TypeMappingIntrospector() {
        }

        static ImmutableMap<TypeVariable<?>, Type> getTypeMappings(Type contextType) {
            TypeMappingIntrospector introspector = new TypeMappingIntrospector();
            introspector.introspect(wildcardCapturer.capture(contextType));
            return ImmutableMap.copyOf(introspector.mappings);
        }

        private void introspect(Type type) {
            if (this.introspectedTypes.add(type)) {
                if (type instanceof ParameterizedType) {
                    introspectParameterizedType((ParameterizedType) type);
                } else if (type instanceof Class) {
                    introspectClass((Class) type);
                } else {
                    int i$ = 0;
                    if (type instanceof TypeVariable) {
                        Type[] arr$ = ((TypeVariable) type).getBounds();
                        int len$ = arr$.length;
                        while (i$ < len$) {
                            introspect(arr$[i$]);
                            i$++;
                        }
                    } else if (type instanceof WildcardType) {
                        Type[] arr$2 = ((WildcardType) type).getUpperBounds();
                        int len$2 = arr$2.length;
                        while (i$ < len$2) {
                            introspect(arr$2[i$]);
                            i$++;
                        }
                    }
                }
            }
        }

        private void introspectClass(Class<?> clazz) {
            introspect(clazz.getGenericSuperclass());
            for (Type interfaceType : clazz.getGenericInterfaces()) {
                introspect(interfaceType);
            }
        }

        private void introspectParameterizedType(ParameterizedType parameterizedType) {
            Class<?> rawClass = (Class) parameterizedType.getRawType();
            TypeVariable<?>[] vars = rawClass.getTypeParameters();
            Type[] typeArgs = parameterizedType.getActualTypeArguments();
            int i = 0;
            Preconditions.checkState(vars.length == typeArgs.length);
            while (true) {
                int i2 = i;
                if (i2 < vars.length) {
                    map(vars[i2], typeArgs[i2]);
                    i = i2 + 1;
                } else {
                    introspectClass(rawClass);
                    introspect(parameterizedType.getOwnerType());
                    return;
                }
            }
        }

        private void map(TypeVariable<?> var, Type arg) {
            if (!this.mappings.containsKey(var)) {
                Type t = arg;
                while (t != null) {
                    if (var.equals(t)) {
                        Type x = arg;
                        while (x != null) {
                            x = this.mappings.remove(x);
                        }
                        return;
                    }
                    t = this.mappings.get(t);
                }
                this.mappings.put(var, arg);
            }
        }
    }

    private static final class WildcardCapturer {
        private final AtomicInteger id;

        private WildcardCapturer() {
            this.id = new AtomicInteger();
        }

        /* access modifiers changed from: package-private */
        public Type capture(Type type) {
            Preconditions.checkNotNull(type);
            if ((type instanceof Class) || (type instanceof TypeVariable)) {
                return type;
            }
            if (type instanceof GenericArrayType) {
                return Types.newArrayType(capture(((GenericArrayType) type).getGenericComponentType()));
            }
            if (type instanceof ParameterizedType) {
                ParameterizedType parameterizedType = (ParameterizedType) type;
                return Types.newParameterizedTypeWithOwner(captureNullable(parameterizedType.getOwnerType()), (Class) parameterizedType.getRawType(), capture(parameterizedType.getActualTypeArguments()));
            } else if (type instanceof WildcardType) {
                WildcardType wildcardType = (WildcardType) type;
                if (wildcardType.getLowerBounds().length != 0) {
                    return type;
                }
                Type[] upperBounds = wildcardType.getUpperBounds();
                return Types.newTypeVariable(WildcardCapturer.class, "capture#" + this.id.incrementAndGet() + "-of ? extends " + Joiner.on('&').join((Object[]) upperBounds), wildcardType.getUpperBounds());
            } else {
                throw new AssertionError("must have been one of the known types");
            }
        }

        private Type captureNullable(@Nullable Type type) {
            if (type == null) {
                return null;
            }
            return capture(type);
        }

        private Type[] capture(Type[] types) {
            Type[] result = new Type[types.length];
            for (int i = 0; i < types.length; i++) {
                result[i] = capture(types[i]);
            }
            return result;
        }
    }
}
