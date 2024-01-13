// Copyright (c) Team 204 Eastern Robotic Vikings. Year 2023
// This software is protected under the license located in the
// public repository of this file under filename: LICENSE
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants204;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Random;

public class PTZCam extends SubsystemBase {
    private final String CONF_baseURL = Constants204.Controller.PTZ_HOSTNAME;
    private final String CONF_username = "admin";
    private final String CONF_password = "team204frc";
    private final Joystick CONF_js = new Joystick(Constants204.Controller.PTZ_JOYSTICK_PORT);

    private final double deadband = 0.25;
    private final int channel = 0; // do not change
    private final int speed = 1;
    private final String cgiPath = "/cgi-bin/ptz.cgi"; // do not change
    private String digestAuthStr = "<DIGEST_AUTH_NOT_GENERATED>"; // do not manually change

    public PTZCam() {
        this.digestAuthStr = getDigestAuthStr();
    }
    public int sendPTZControlCmd(String action, int ch, String code, int arg2) {
        HttpURLConnection connection = null;

        try {
            String query = String.format("action=%s&ch=%s&code=%s&arg1=0&arg2=%s&arg3=0", action, ch, code, arg2);
            URL url = new URL(CONF_baseURL + cgiPath + "?" + query);
            connection = (HttpURLConnection) url.openConnection();
            connection.setRequestMethod("GET");
            //connection.setRequestProperty("Authorization", "Basic YWRtaW46dGVhbTIwNGZyYw==");
            connection.setRequestProperty("Authorization", digestAuthStr);

            //connection.setDoInput(true);
            connection.setDoOutput(true);

            DataOutputStream wr = new DataOutputStream(connection.getOutputStream());
            wr.close();

            //Get Response
            InputStream is = connection.getInputStream();
            BufferedReader rd = new BufferedReader(new InputStreamReader(is));
            StringBuilder response = new StringBuilder(); // or StringBuffer if Java version 5+
            String line;
            while ((line = rd.readLine()) != null) {
                response.append(line);
                response.append('\r');
            }
            rd.close();
            System.out.println(response.toString());

            return connection.getResponseCode();
        } catch (Exception e) {
            e.printStackTrace();
            return 0;
        } finally {
            if (connection != null) {
                connection.disconnect();
            }
        }
    }

    @Override
    public void periodic() {
        // runs every scheduler loop (every ~20ms)
        if (CONF_js.getTrigger()) {
            double x = Math.abs(CONF_js.getX()) < deadband ? 0.0 : CONF_js.getX();
            double y = Math.abs(CONF_js.getY()) < deadband ? 0.0 : CONF_js.getY();
            double z = Math.abs(CONF_js.getZ()) < deadband ? 0.0 : CONF_js.getZ();

            int result;

            System.out.println("PTZ CONTROL x=" + x + " y=" + y + " z=" + z);

            if (x > 0) {
                result = sendPTZControlCmd("start", channel, "Right", speed);
            } else if (x < 0) {
                result = sendPTZControlCmd("start", channel, "Left", speed);
            } else {
                result = sendPTZControlCmd("stop", channel, "Right", speed);
            }
            System.out.println("xstat="+result);
            if (y > 0) {
                result = sendPTZControlCmd("start", channel, "Up", speed);
            } else if (y < 0) {
                result = sendPTZControlCmd("start", channel, "Down", speed);
            } else {
                result = sendPTZControlCmd("stop", channel, "Up", speed);
            }
            System.out.println("ystat="+result);
            if (z > 0) {
                result = sendPTZControlCmd("start", channel, "ZoomTele", speed);
            } else if (z < 0) {
                result = sendPTZControlCmd("start", channel, "ZoomWide", speed);
            } else {
                result = sendPTZControlCmd("stop", channel, "ZoomTele", speed);
            }
            System.out.println("zstat="+result);
        }
    }

    private String getDigestAuthStr() {
        try {
            URL url = new URL(CONF_baseURL + cgiPath);
            HttpURLConnection connection = (HttpURLConnection) url.openConnection(); // bad java error handling: go>cs>java
            connection.setRequestMethod("GET");
            connection.setDoOutput(true);

            String nonce = null;
            String realm = null;
            String qop = null;
            String algorithm = "MD5";
            String cNonce = String.valueOf(new Random().nextInt(1000000000, Integer.MAX_VALUE));
            int nc = 1;

            // get challenge
            HttpURLConnection challenge = (HttpURLConnection) new URL(CONF_baseURL + cgiPath).openConnection();
            challenge.connect(); // mf ERROR
            String header = challenge.getHeaderField("WWW-Authenticate");

            // goofy ass parsing
            String[] authHeader = header.split(",");
            for (String element : authHeader) {
                if (element.contains("nonce")) {
                    nonce = element.split("=")[1].replace("\"", "");
                } else if (element.contains("realm")) {
                    realm = element.split("=")[1].replace("\"", "");
                } else if (element.contains("qop")) {
                    qop = element.split("=")[1].replace("\"", "");
                }
            }

            challenge.disconnect();

            // make digest auth string
            String h1 = CONF_username + ":" + realm + ":" + CONF_password;
            String h2 = "GET:" + cgiPath;
            String response = null;

            try {
                MessageDigest md = MessageDigest.getInstance(algorithm); // AUGH ERROR HANDLEING
                md.update(h1.getBytes());
                byte[] h1_bytes = md.digest();

                StringBuilder h1_sb = new StringBuilder();
                for (byte b : h1_bytes) {
                    h1_sb.append(String.format("%02x", b & 0xff));
                }

                String h1_f = h1_sb.toString();

                md.reset();
                md.update(h2.getBytes());
                byte[] h2_bytes = md.digest();

                StringBuilder h2_sb = new StringBuilder();
                for (byte b : h2_bytes) {
                    h2_sb.append(String.format("%02x", b & 0xff));
                }

                String h2_f = h2_sb.toString();

                md.reset();
                md.update((h1_f + ":" + nonce + ":" + nc + ":" + cNonce + ":" + qop + ":" + h2_f).getBytes());
                byte[] d_bytes = md.digest();

                StringBuilder d_sb = new StringBuilder();
                for (byte b : d_bytes) {
                    d_sb.append(String.format("%02x", b & 0xff));
                }

                response = d_sb.toString();
            } catch (NoSuchAlgorithmException e) {
                e.printStackTrace(); // this should never ever happen.
            }

            return "Digest username=\"" + CONF_username + "\", realm=\"" + realm + "\", nonce=\"" + nonce + "\", uri=\"" + cgiPath + "\", qop=\"" + qop + "\", nc=" + String.format("%08d", nc) + ", cNonce=\"" + cNonce + "\", response=\"" + response + "\", algorithm=\"" + algorithm + "\"";
        } catch (Exception e) {
            System.out.println("PTZCam error");
            e.printStackTrace();
        }
        System.out.println("PTZCam failed to generate digest auth string");
        return "<FAILED>";
    }
}