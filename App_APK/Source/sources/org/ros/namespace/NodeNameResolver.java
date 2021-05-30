package org.ros.namespace;

public class NodeNameResolver extends NameResolver {
    private final GraphName privateNamespace;

    public NodeNameResolver(GraphName nodeName, NameResolver defaultResolver) {
        super(defaultResolver.getNamespace(), defaultResolver.getRemappings());
        this.privateNamespace = nodeName;
    }

    public GraphName resolve(GraphName name) {
        GraphName graphName = lookUpRemapping(name);
        if (graphName.isPrivate()) {
            return resolve(this.privateNamespace, graphName.toRelative());
        }
        return super.resolve(name);
    }

    public GraphName resolve(String name) {
        return resolve(GraphName.of(name));
    }
}
