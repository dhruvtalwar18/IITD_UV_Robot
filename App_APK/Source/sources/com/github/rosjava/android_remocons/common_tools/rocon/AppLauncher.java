package com.github.rosjava.android_remocons.common_tools.rocon;

import android.app.Activity;
import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.content.pm.ApplicationInfo;
import android.content.pm.PackageManager;
import android.net.Uri;
import android.util.Log;
import android.util.Patterns;
import com.github.robotics_in_concert.rocon_rosjava_core.rocon_interactions.InteractionMode;
import com.github.rosjava.android_remocons.common_tools.master.MasterDescription;
import com.github.rosjava.android_remocons.common_tools.master.RoconDescription;
import java.net.MalformedURLException;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLEncoder;
import java.util.List;
import java.util.Map;
import org.yaml.snakeyaml.Yaml;
import rocon_interaction_msgs.Interaction;
import rocon_std_msgs.Remapping;

public class AppLauncher {

    public enum AppType {
        NATIVE,
        URL,
        WEB_URL,
        WEB_APP,
        NOTHING
    }

    public enum Result {
        SUCCESS,
        NOT_INSTALLED,
        NOTHING,
        CANNOT_CONNECT,
        MALFORMED_URI,
        CONNECT_TIMEOUT,
        OTHER_ERROR;
        
        public String message;

        /* access modifiers changed from: package-private */
        public Result withMsg(String message2) {
            this.message = message2;
            return this;
        }
    }

    public static Result launch(Activity parent, RoconDescription concert, Interaction app) {
        Log.i("AppLaunch", "launching concert app " + app.getDisplayName() + " on service " + app.getNamespace());
        AppType app_type = checkAppType(app.getName());
        if (app_type == AppType.URL) {
            return launchUrl(parent, concert, app);
        }
        if (app_type == AppType.WEB_URL) {
            return launchWebUrl(parent, concert, app);
        }
        if (app_type == AppType.WEB_APP) {
            return launchWebApp(parent, concert, app);
        }
        if (app_type == AppType.NATIVE) {
            return launchAndroidApp(parent, concert, app);
        }
        if (app_type == AppType.NOTHING) {
            return Result.NOTHING;
        }
        return Result.NOTHING;
    }

    public static AppType checkAppType(String app_name) {
        if (Patterns.WEB_URL.matcher(app_name).matches()) {
            return AppType.URL;
        }
        if (app_name.length() == 0) {
            return AppType.NOTHING;
        }
        if (app_name.contains("web_app(")) {
            return AppType.WEB_APP;
        }
        if (app_name.contains("web_url(")) {
            return AppType.WEB_URL;
        }
        return AppType.NATIVE;
    }

    private static Result launchAndroidApp(Activity parent, RoconDescription concert, Interaction app) {
        Result result = Result.OTHER_ERROR;
        String appName = app.getName();
        Intent intent = new Intent(appName);
        intent.putExtra("com.github.rosjava.android_remocons.common_tools.rocon.Constants." + InteractionMode.CONCERT + "_app_name", appName);
        intent.putExtra(MasterDescription.UNIQUE_KEY, concert);
        intent.putExtra("RemoconActivity", Constants.ACTIVITY_ROCON_REMOCON);
        intent.putExtra("Parameters", app.getParameters());
        if (app.getRemappings() != null && app.getRemappings().size() > 0) {
            String remaps = "{";
            for (Remapping remap : app.getRemappings()) {
                remaps = remaps + remap.getRemapFrom() + ": " + remap.getRemapTo() + ", ";
            }
            intent.putExtra("Remappings", remaps.substring(0, remaps.length() - 2) + "}");
        }
        try {
            Log.i("AppLaunch", "launchAndroidApp trying to start activity (action: " + appName + " )");
            parent.startActivity(intent);
            return Result.SUCCESS;
        } catch (ActivityNotFoundException e) {
            Log.i("AppLaunch", "launchAndroidApp activity not found for action, find package name: " + appName);
            return launchAndroidAppWithPkgName(parent, concert, app);
        }
    }

    private static Result launchAndroidAppWithPkgName(Activity parent, RoconDescription concert, Interaction app) {
        Result result = Result.OTHER_ERROR;
        String appName = app.getName();
        Intent intent = parent.getPackageManager().getLaunchIntentForPackage(appName);
        if (intent == null) {
            return launchAndroidAppWithAppId(parent, concert, app);
        }
        intent.putExtra("com.github.rosjava.android_remocons.common_tools.rocon.Constants." + InteractionMode.CONCERT + "_app_name", appName);
        intent.putExtra(MasterDescription.UNIQUE_KEY, concert);
        intent.putExtra("RemoconActivity", Constants.ACTIVITY_ROCON_REMOCON);
        intent.putExtra("Parameters", app.getParameters());
        if (app.getRemappings() != null && app.getRemappings().size() > 0) {
            String remaps = "{";
            for (Remapping remap : app.getRemappings()) {
                remaps = remaps + remap.getRemapFrom() + ": " + remap.getRemapTo() + ", ";
            }
            intent.putExtra("Remappings", remaps.substring(0, remaps.length() - 2) + "}");
        }
        try {
            Log.i("AppLaunch", "launchAndroidAppWithPkgName trying to start activity (action: " + appName + " )");
            parent.startActivity(intent);
            return Result.SUCCESS;
        } catch (ActivityNotFoundException e) {
            Log.i("AppLaunch", "launchAndroidAppWithPkgName activity not found for action: " + appName);
            return launchAndroidAppWithAppId(parent, concert, app);
        }
    }

    private static Result launchAndroidAppWithAppId(Activity parent, RoconDescription concert, Interaction app) {
        Intent intent;
        Result result = Result.OTHER_ERROR;
        String launchablePkgName = "";
        PackageManager manager = parent.getPackageManager();
        List<ApplicationInfo> applicationInfo = manager.getInstalledApplications(128);
        int i = 0;
        while (true) {
            if (i >= applicationInfo.size()) {
                break;
            }
            ApplicationInfo appInfo = applicationInfo.get(i);
            if (app.getName().contains(appInfo.processName)) {
                launchablePkgName = appInfo.packageName;
                break;
            }
            i++;
        }
        if (!(launchablePkgName.length() == 0 || (intent = manager.getLaunchIntentForPackage(launchablePkgName)) == null)) {
            intent.putExtra("com.github.rosjava.android_remocons.common_tools.rocon.Constants." + InteractionMode.CONCERT + "_app_name", launchablePkgName);
            intent.putExtra(MasterDescription.UNIQUE_KEY, concert);
            intent.putExtra("RemoconActivity", Constants.ACTIVITY_ROCON_REMOCON);
            intent.putExtra("Parameters", app.getParameters());
            if (app.getRemappings() != null && app.getRemappings().size() > 0) {
                String remaps = "{";
                for (Remapping remap : app.getRemappings()) {
                    remaps = remaps + remap.getRemapFrom() + ": " + remap.getRemapTo() + ", ";
                }
                intent.putExtra("Remappings", remaps.substring(0, remaps.length() - 2) + "}");
            }
            try {
                Log.i("AppLaunch", "launchAndroidAppWithoutRosVer trying to start activity (action: " + launchablePkgName + " )");
                parent.startActivity(intent);
                Result result2 = Result.SUCCESS;
            } catch (ActivityNotFoundException e) {
                Log.i("AppLaunch", "launchAndroidAppWithoutRosVer activity not found for action, find package name: " + launchablePkgName);
                Result result3 = Result.NOT_INSTALLED;
            }
        }
        return Result.NOT_INSTALLED;
    }

    private static Result launchUrl(Activity parent, RoconDescription concert, Interaction app) {
        try {
            String app_name = app.getName();
            new URL(app_name);
            String appUriStr = app_name;
            Intent intent = new Intent("android.intent.action.VIEW", Uri.parse(appUriStr));
            intent.putExtra("com.github.rosjava.android_remocons.common_tools.rocon.Constants." + InteractionMode.CONCERT + "_app_name", app_name);
            Log.i("AppLaunch", "trying to start web app (URI: " + appUriStr + ")");
            parent.startActivity(intent);
            return Result.SUCCESS;
        } catch (MalformedURLException e) {
            Result result = Result.MALFORMED_URI;
            return result.withMsg("App URL is not valid. " + e.getMessage());
        } catch (ActivityNotFoundException e2) {
            return Result.NOT_INSTALLED.withMsg("Activity not found for view action??? muoia???");
        } catch (Exception e3) {
            return Result.OTHER_ERROR.withMsg(e3.getMessage());
        }
    }

    private static Result launchWebUrl(Activity parent, RoconDescription concert, Interaction app) {
        try {
            String app_name = app.getName().substring("web_url".length() + 1, app.getName().length() - 1);
            new URL(app_name);
            String appUriStr = app_name;
            Intent intent = new Intent("android.intent.action.VIEW", Uri.parse(appUriStr));
            intent.putExtra("com.github.rosjava.android_remocons.common_tools.rocon.Constants." + InteractionMode.CONCERT + "_app_name", app_name);
            Log.i("AppLaunch", "trying to start web url (URI: " + appUriStr + ")");
            parent.startActivity(intent);
            return Result.SUCCESS;
        } catch (MalformedURLException e) {
            Result result = Result.MALFORMED_URI;
            return result.withMsg("App URL is not valid. " + e.getMessage());
        } catch (ActivityNotFoundException e2) {
            return Result.NOT_INSTALLED.withMsg("Activity not found for view action??? muoia???");
        } catch (Exception e3) {
            return Result.OTHER_ERROR.withMsg(e3.getMessage());
        }
    }

    private static Result launchWebApp(Activity parent, RoconDescription concert, Interaction app) {
        String remaps;
        try {
            String app_name = app.getName().substring("web_app".length() + 1, app.getName().length() - 1);
            URL appURL = new URL(app_name);
            String appUriStr = app_name;
            String remaps2 = "\"remappings\": {";
            if (app.getRemappings() == null || app.getRemappings().size() <= 0) {
                remaps = remaps2 + "}";
            } else {
                for (Remapping remap : app.getRemappings()) {
                    remaps2 = remaps2 + "\"" + remap.getRemapFrom() + "\":\"" + remap.getRemapTo() + "\",";
                }
                remaps = remaps2.substring(0, remaps2.length() - 1) + "}";
            }
            String interaction_data = "{" + (remaps + ",");
            String displayname = "\"display_name\":";
            if (app.getDisplayName() != null && app.getDisplayName().length() > 0) {
                displayname = displayname + "\"" + app.getDisplayName() + "\"";
            }
            String interaction_data2 = interaction_data + (displayname + ",");
            String parameters = "\"parameters\": {";
            if (app.getParameters() != null && app.getParameters().length() > 2) {
                Map<String, String> params = (Map) new Yaml().load(app.getParameters());
                for (String key : params.keySet()) {
                    parameters = parameters + "\"" + key + "\":\"" + String.valueOf(params.get(key)) + "\",";
                }
                parameters = parameters.substring(0, parameters.length() - 1);
            }
            String interaction_data3 = (interaction_data2 + (parameters + "}")) + "}";
            String appUriStr2 = appUriStr + "?interaction_data=" + URLEncoder.encode(interaction_data3);
            appURL.toURI();
            Intent intent = new Intent("android.intent.action.VIEW", Uri.parse(appUriStr2));
            intent.putExtra("com.github.rosjava.android_remocons.common_tools.rocon.Constants." + InteractionMode.CONCERT + "_app_name", app_name);
            Log.i("AppLaunch", "trying to start web app (URI: " + appUriStr2 + ")");
            try {
                parent.startActivity(intent);
                return Result.SUCCESS;
            } catch (URISyntaxException e) {
                e = e;
                return Result.MALFORMED_URI.withMsg("Cannot convert URL into URI. " + e.getMessage());
            } catch (MalformedURLException e2) {
                e = e2;
                return Result.MALFORMED_URI.withMsg("App URL is not valid. " + e.getMessage());
            } catch (ActivityNotFoundException e3) {
                e = e3;
                ActivityNotFoundException activityNotFoundException = e;
                return Result.NOT_INSTALLED.withMsg("Activity not found for view action??? muoia???");
            } catch (Exception e4) {
                e = e4;
                return Result.OTHER_ERROR.withMsg(e.getMessage());
            }
        } catch (URISyntaxException e5) {
            e = e5;
            Activity activity = parent;
            return Result.MALFORMED_URI.withMsg("Cannot convert URL into URI. " + e.getMessage());
        } catch (MalformedURLException e6) {
            e = e6;
            Activity activity2 = parent;
            return Result.MALFORMED_URI.withMsg("App URL is not valid. " + e.getMessage());
        } catch (ActivityNotFoundException e7) {
            e = e7;
            Activity activity3 = parent;
            ActivityNotFoundException activityNotFoundException2 = e;
            return Result.NOT_INSTALLED.withMsg("Activity not found for view action??? muoia???");
        } catch (Exception e8) {
            e = e8;
            Activity activity4 = parent;
            return Result.OTHER_ERROR.withMsg(e.getMessage());
        }
    }
}
