package org.bytedeco.javacpp.tools;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Map;
import java.util.Properties;
import org.apache.maven.artifact.Artifact;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.descriptor.PluginDescriptor;
import org.apache.maven.plugin.logging.Log;
import org.apache.maven.plugins.annotations.LifecyclePhase;
import org.apache.maven.plugins.annotations.Mojo;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.project.MavenProject;
import org.bytedeco.javacpp.Loader;

@Mojo(defaultPhase = LifecyclePhase.PROCESS_CLASSES, name = "build", threadSafe = true)
public class BuildMojo extends AbstractMojo {
    @Parameter(property = "javacpp.buildCommand")
    String[] buildCommand = null;
    @Parameter(property = "javacpp.buildPath")
    String buildPath = null;
    @Parameter(property = "javacpp.buildPaths")
    String[] buildPaths = null;
    @Parameter(property = "javacpp.buildResource")
    String buildResource = null;
    @Parameter(property = "javacpp.buildResources")
    String[] buildResources = null;
    @Parameter(property = "javacpp.classOrPackageName")
    String classOrPackageName = null;
    @Parameter(property = "javacpp.classOrPackageNames")
    String[] classOrPackageNames = null;
    @Parameter(defaultValue = "${project.build.outputDirectory}", property = "javacpp.classPath")
    String classPath = null;
    @Parameter(property = "javacpp.classPaths")
    String[] classPaths = null;
    @Parameter(defaultValue = "true", property = "javacpp.compile")
    boolean compile = true;
    @Parameter(property = "javacpp.compilerOptions")
    String[] compilerOptions = null;
    @Parameter(defaultValue = "false", property = "javacpp.copyLibs")
    boolean copyLibs = false;
    @Parameter(defaultValue = "false", property = "javacpp.copyResources")
    boolean copyResources = false;
    @Parameter(defaultValue = "true", property = "javacpp.deleteJniFiles")
    boolean deleteJniFiles = true;
    @Parameter(property = "javacpp.encoding")
    String encoding = null;
    @Parameter(property = "javacpp.environmentVariables")
    Map<String, String> environmentVariables = null;
    @Parameter(defaultValue = "true", property = "javacpp.generate")
    boolean generate = true;
    @Parameter(defaultValue = "false", property = "javacpp.header")
    boolean header = false;
    @Parameter(property = "javacpp.includePath")
    String includePath = null;
    @Parameter(property = "javacpp.includePaths")
    String[] includePaths = null;
    @Parameter(property = "javacpp.includeResource")
    String includeResource = null;
    @Parameter(property = "javacpp.includeResources")
    String[] includeResources = null;
    @Parameter(property = "javacpp.jarPrefix")
    String jarPrefix = null;
    @Parameter(property = "javacpp.linkPath")
    String linkPath = null;
    @Parameter(property = "javacpp.linkPaths")
    String[] linkPaths = null;
    @Parameter(property = "javacpp.linkResource")
    String linkResource = null;
    @Parameter(property = "javacpp.linkResources")
    String[] linkResources = null;
    @Parameter(property = "javacpp.outputDirectory")
    File outputDirectory = null;
    @Parameter(property = "javacpp.outputName")
    String outputName = null;
    @Parameter(defaultValue = "${plugin}", readonly = true, required = true)
    PluginDescriptor plugin;
    @Parameter(property = "javacpp.preloadPath")
    String preloadPath = null;
    @Parameter(property = "javacpp.preloadPaths")
    String[] preloadPaths = null;
    @Parameter(property = "javacpp.preloadResource")
    String preloadResource = null;
    @Parameter(property = "javacpp.preloadResources")
    String[] preloadResources = null;
    @Parameter(defaultValue = "${project}", readonly = true, required = true)
    MavenProject project;
    @Parameter(property = "javacpp.properties")
    String properties = null;
    @Parameter(property = "javacpp.propertyFile")
    File propertyFile = null;
    @Parameter(property = "javacpp.propertyKeysAndValues")
    Properties propertyKeysAndValues = null;
    @Parameter(property = "javacpp.resourcePath")
    String resourcePath = null;
    @Parameter(property = "javacpp.resourcePaths")
    String[] resourcePaths = null;
    @Parameter(defaultValue = "false", property = "javacpp.skip")
    boolean skip = false;
    @Parameter(property = "javacpp.targetDirectory")
    String targetDirectory = null;
    @Parameter(property = "javacpp.workingDirectory")
    File workingDirectory = null;

    /* JADX WARNING: type inference failed for: r2v2, types: [java.lang.Object[]] */
    /* access modifiers changed from: package-private */
    /* JADX WARNING: Multi-variable type inference failed */
    /* Code decompiled incorrectly, please refer to instructions dump. */
    public java.lang.String[] merge(java.lang.String[] r4, java.lang.String r5) {
        /*
            r3 = this;
            r0 = 0
            r1 = 1
            if (r4 == 0) goto L_0x0014
            if (r5 == 0) goto L_0x0014
            int r2 = r4.length
            int r2 = r2 + r1
            java.lang.Object[] r2 = java.util.Arrays.copyOf(r4, r2)
            r4 = r2
            java.lang.String[] r4 = (java.lang.String[]) r4
            int r2 = r4.length
            int r2 = r2 - r1
            r4[r2] = r5
            goto L_0x001b
        L_0x0014:
            if (r5 == 0) goto L_0x001b
            java.lang.String[] r1 = new java.lang.String[r1]
            r1[r0] = r5
            r4 = r1
        L_0x001b:
            if (r4 == 0) goto L_0x001f
            r0 = r4
            goto L_0x0021
        L_0x001f:
            java.lang.String[] r0 = new java.lang.String[r0]
        L_0x0021:
            return r0
        */
        throw new UnsupportedOperationException("Method not decompiled: org.bytedeco.javacpp.tools.BuildMojo.merge(java.lang.String[], java.lang.String):java.lang.String[]");
    }

    public void execute() throws MojoExecutionException {
        String str;
        StringBuilder sb;
        StringBuilder sb2;
        StringBuilder sb3;
        StringBuilder sb4;
        StringBuilder sb5;
        StringBuilder sb6;
        StringBuilder sb7;
        StringBuilder sb8;
        StringBuilder sb9;
        StringBuilder sb10;
        final Log log = getLog();
        try {
            if (log.isDebugEnabled()) {
                log.debug("classPath: " + this.classPath);
                log.debug("classPaths: " + Arrays.deepToString(this.classPaths));
                log.debug("buildPath: " + this.buildPath);
                log.debug("buildPaths: " + Arrays.deepToString(this.buildPaths));
                log.debug("buildResource: " + this.buildResource);
                log.debug("buildResources: " + Arrays.deepToString(this.buildResources));
                log.debug("includePath: " + this.includePath);
                log.debug("includePaths: " + Arrays.deepToString(this.includePaths));
                log.debug("includeResource: " + this.includeResource);
                log.debug("includeResources: " + Arrays.deepToString(this.includeResources));
                log.debug("linkPath: " + this.linkPath);
                log.debug("linkPaths: " + Arrays.deepToString(this.linkPaths));
                log.debug("linkResource: " + this.linkResource);
                log.debug("linkResources: " + Arrays.deepToString(this.linkResources));
                log.debug("preloadPath: " + this.preloadPath);
                log.debug("preloadPaths: " + Arrays.deepToString(this.preloadPaths));
                log.debug("preloadResource: " + this.preloadResource);
                log.debug("preloadResources: " + Arrays.deepToString(this.preloadResources));
                log.debug("resourcePath: " + this.resourcePath);
                log.debug("resourcePaths: " + Arrays.deepToString(this.resourcePaths));
                log.debug("encoding: " + this.encoding);
                log.debug("outputDirectory: " + this.outputDirectory);
                log.debug("outputName: " + this.outputName);
                log.debug("generate: " + this.generate);
                log.debug("compile: " + this.compile);
                log.debug("deleteJniFiles: " + this.deleteJniFiles);
                log.debug("header: " + this.header);
                log.debug("copyLibs: " + this.copyLibs);
                log.debug("copyResources: " + this.copyResources);
                log.debug("jarPrefix: " + this.jarPrefix);
                log.debug("properties: " + this.properties);
                log.debug("propertyFile: " + this.propertyFile);
                log.debug("propertyKeysAndValues: " + this.propertyKeysAndValues);
                log.debug("classOrPackageName: " + this.classOrPackageName);
                log.debug("classOrPackageNames: " + Arrays.deepToString(this.classOrPackageNames));
                log.debug("buildCommand: " + Arrays.deepToString(this.buildCommand));
                log.debug("targetDirectory: " + Arrays.deepToString(this.buildCommand));
                log.debug("workingDirectory: " + this.workingDirectory);
                log.debug("environmentVariables: " + this.environmentVariables);
                log.debug("compilerOptions: " + Arrays.deepToString(this.compilerOptions));
                log.debug("skip: " + this.skip);
            }
            if (this.targetDirectory != null) {
                this.project.addCompileSourceRoot(this.targetDirectory);
            }
            if (this.skip) {
                log.info("Skipping execution of JavaCPP Builder");
                return;
            }
            this.classPaths = merge(this.classPaths, this.classPath);
            this.classOrPackageNames = merge(this.classOrPackageNames, this.classOrPackageName);
            Builder builder = new Builder(new Logger() {
                public void debug(String s) {
                    log.debug(s);
                }

                public void info(String s) {
                    log.info(s);
                }

                public void warn(String s) {
                    log.warn(s);
                }

                public void error(String s) {
                    log.error(s);
                }
            }).classPaths(this.classPaths).encoding(this.encoding).outputDirectory(this.outputDirectory).outputName(this.outputName).generate(this.generate).compile(this.compile).deleteJniFiles(this.deleteJniFiles).header(this.header).copyLibs(this.copyLibs).copyResources(this.copyResources).jarPrefix(this.jarPrefix).properties(this.properties).propertyFile(this.propertyFile).properties(this.propertyKeysAndValues).classesOrPackages(this.classOrPackageNames).buildCommand(this.buildCommand).workingDirectory(this.workingDirectory).environmentVariables(this.environmentVariables).compilerOptions(this.compilerOptions);
            Properties properties2 = builder.properties;
            String extension = properties2.getProperty("platform.extension");
            log.info("Detected platform \"" + Loader.getPlatform() + "\"");
            StringBuilder sb11 = new StringBuilder();
            sb11.append("Building for platform \"");
            sb11.append(properties2.get("platform"));
            sb11.append("\"");
            if (extension == null || extension.length() <= 0) {
                str = "";
            } else {
                str = " with extension \"" + extension + "\"";
            }
            sb11.append(str);
            log.info(sb11.toString());
            String separator = properties2.getProperty("platform.path.separator");
            for (String s : merge(this.buildPaths, this.buildPath)) {
                String v = properties2.getProperty("platform.buildpath", "");
                if (v.length() != 0) {
                    if (!v.endsWith(separator)) {
                        sb10 = new StringBuilder();
                        sb10.append(v);
                        sb10.append(separator);
                        sb10.append(s);
                        properties2.setProperty("platform.buildpath", sb10.toString());
                    }
                }
                sb10 = new StringBuilder();
                sb10.append(v);
                sb10.append(s);
                properties2.setProperty("platform.buildpath", sb10.toString());
            }
            for (String s2 : merge(this.buildResources, this.buildResource)) {
                String v2 = properties2.getProperty("platform.buildresource", "");
                if (v2.length() != 0) {
                    if (!v2.endsWith(separator)) {
                        sb9 = new StringBuilder();
                        sb9.append(v2);
                        sb9.append(separator);
                        sb9.append(s2);
                        properties2.setProperty("platform.buildresource", sb9.toString());
                    }
                }
                sb9 = new StringBuilder();
                sb9.append(v2);
                sb9.append(s2);
                properties2.setProperty("platform.buildresource", sb9.toString());
            }
            for (String s3 : merge(this.includePaths, this.includePath)) {
                String v3 = properties2.getProperty("platform.includepath", "");
                if (v3.length() != 0) {
                    if (!v3.endsWith(separator)) {
                        sb8 = new StringBuilder();
                        sb8.append(v3);
                        sb8.append(separator);
                        sb8.append(s3);
                        properties2.setProperty("platform.includepath", sb8.toString());
                    }
                }
                sb8 = new StringBuilder();
                sb8.append(v3);
                sb8.append(s3);
                properties2.setProperty("platform.includepath", sb8.toString());
            }
            for (String s4 : merge(this.includeResources, this.includeResource)) {
                String v4 = properties2.getProperty("platform.includeresource", "");
                if (v4.length() != 0) {
                    if (!v4.endsWith(separator)) {
                        sb7 = new StringBuilder();
                        sb7.append(v4);
                        sb7.append(separator);
                        sb7.append(s4);
                        properties2.setProperty("platform.includeresource", sb7.toString());
                    }
                }
                sb7 = new StringBuilder();
                sb7.append(v4);
                sb7.append(s4);
                properties2.setProperty("platform.includeresource", sb7.toString());
            }
            for (String s5 : merge(this.linkPaths, this.linkPath)) {
                String v5 = properties2.getProperty("platform.linkpath", "");
                if (v5.length() != 0) {
                    if (!v5.endsWith(separator)) {
                        sb6 = new StringBuilder();
                        sb6.append(v5);
                        sb6.append(separator);
                        sb6.append(s5);
                        properties2.setProperty("platform.linkpath", sb6.toString());
                    }
                }
                sb6 = new StringBuilder();
                sb6.append(v5);
                sb6.append(s5);
                properties2.setProperty("platform.linkpath", sb6.toString());
            }
            for (String s6 : merge(this.linkResources, this.linkResource)) {
                String v6 = properties2.getProperty("platform.linkresource", "");
                if (v6.length() != 0) {
                    if (!v6.endsWith(separator)) {
                        sb5 = new StringBuilder();
                        sb5.append(v6);
                        sb5.append(separator);
                        sb5.append(s6);
                        properties2.setProperty("platform.linkresource", sb5.toString());
                    }
                }
                sb5 = new StringBuilder();
                sb5.append(v6);
                sb5.append(s6);
                properties2.setProperty("platform.linkresource", sb5.toString());
            }
            for (String s7 : merge(this.preloadPaths, this.preloadPath)) {
                String v7 = properties2.getProperty("platform.preloadpath", "");
                if (v7.length() != 0) {
                    if (!v7.endsWith(separator)) {
                        sb4 = new StringBuilder();
                        sb4.append(v7);
                        sb4.append(separator);
                        sb4.append(s7);
                        properties2.setProperty("platform.preloadpath", sb4.toString());
                    }
                }
                sb4 = new StringBuilder();
                sb4.append(v7);
                sb4.append(s7);
                properties2.setProperty("platform.preloadpath", sb4.toString());
            }
            for (String s8 : merge(this.preloadResources, this.preloadResource)) {
                String v8 = properties2.getProperty("platform.preloadresource", "");
                if (v8.length() != 0) {
                    if (!v8.endsWith(separator)) {
                        sb3 = new StringBuilder();
                        sb3.append(v8);
                        sb3.append(separator);
                        sb3.append(s8);
                        properties2.setProperty("platform.preloadresource", sb3.toString());
                    }
                }
                sb3 = new StringBuilder();
                sb3.append(v8);
                sb3.append(s8);
                properties2.setProperty("platform.preloadresource", sb3.toString());
            }
            for (String s9 : merge(this.resourcePaths, this.resourcePath)) {
                String v9 = properties2.getProperty("platform.resourcepath", "");
                if (v9.length() != 0) {
                    if (!v9.endsWith(separator)) {
                        sb2 = new StringBuilder();
                        sb2.append(v9);
                        sb2.append(separator);
                        sb2.append(s9);
                        properties2.setProperty("platform.resourcepath", sb2.toString());
                    }
                }
                sb2 = new StringBuilder();
                sb2.append(v9);
                sb2.append(s9);
                properties2.setProperty("platform.resourcepath", sb2.toString());
            }
            properties2.setProperty("platform.artifacts", this.project.getBuild().getOutputDirectory());
            for (Artifact a : this.plugin.getArtifacts()) {
                String s10 = a.getFile().getCanonicalPath();
                String v10 = properties2.getProperty("platform.artifacts", "");
                if (v10.length() != 0) {
                    if (!v10.endsWith(separator)) {
                        sb = new StringBuilder();
                        sb.append(v10);
                        sb.append(separator);
                        sb.append(s10);
                        properties2.setProperty("platform.artifacts", sb.toString());
                    }
                }
                sb = new StringBuilder();
                sb.append(v10);
                sb.append(s10);
                properties2.setProperty("platform.artifacts", sb.toString());
            }
            Properties projectProperties = this.project.getProperties();
            for (String key : properties2.stringPropertyNames()) {
                projectProperties.setProperty("javacpp." + key, properties2.getProperty(key));
            }
            File[] outputFiles = builder.build();
            if (log.isDebugEnabled()) {
                log.debug("outputFiles: " + Arrays.deepToString(outputFiles));
            }
        } catch (IOException | ClassNotFoundException | InterruptedException | NoClassDefFoundError | ParserException e) {
            log.error("Failed to execute JavaCPP Builder: " + e.getMessage());
            throw new MojoExecutionException("Failed to execute JavaCPP Builder", e);
        }
    }
}
