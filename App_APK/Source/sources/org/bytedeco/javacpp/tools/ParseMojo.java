package org.bytedeco.javacpp.tools;

import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugins.annotations.LifecyclePhase;
import org.apache.maven.plugins.annotations.Mojo;

@Mojo(defaultPhase = LifecyclePhase.GENERATE_SOURCES, name = "parse")
public class ParseMojo extends BuildMojo {
    public void execute() throws MojoExecutionException {
        this.generate = false;
        super.execute();
    }
}
