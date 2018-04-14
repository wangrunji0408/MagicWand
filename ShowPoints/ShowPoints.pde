// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP output
// 6/20/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-20 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;
import java.nio.*;
import java.util.*;

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
//    (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

final int scale = 2;
Device device1;

void setupSerial() {
    // display serial port list for debugging/clarity
    println(Serial.list());

    // get the first available port (use EITHER this OR the specific port code below)
    //String portName = Serial.list()[4];
    
    // get a specific serial port (use EITHER this OR the first-available code above)
    // String portName = "/dev/cu.usbmodem1421";
    // String portName = "/dev/cu.SLAB_USBtoUART";
    // String portName = "/dev/cu.HC-05-DevB";
    // String portName = "/dev/cu.HC-05-DevB-1";
    
    device1 = new Device();
    device1.port = new Serial(this, "/dev/cu.HC-05-DevB-1", 115200);
    device1.init();
}

void setup() {    
    // 300px square viewport using OpenGL rendering
    // size(300, 300, P3D);
    size(720, 360);

    // setup lights and antialiasing
    lights();
    smooth();
    
    setupSerial();
}

void draw() {
    device1.handleSerial();
    background(0);
    
    translate(width / 2, height / 2);
    device1.draw();
}

class Device {
    Serial port;
    Handwritting hw = new Handwritting();
    PushDetector pd = new PushDetector();
    boolean pressed = true;
    int barValue = 0;

    void setBar(int value) {
        value = Integer.min(9, value);
        value = Integer.max(0, value);
        barValue = value;
        port.write("setbar " + Integer.toString(value) + "\n");
    }

    void setF(int f) {
        port.write("setf " + Integer.toString(f) + "\n");     
    }

    void finish() {
        hw.finish();
        print("Finish: ");
        if(hw.isCircle())
            print("Circle ");
        if(hw.isSquare())
            print("Square ");
        println();           
    }

    void draw() {
        if(hw != null)
            hw.draw();
    }

    void init() {
        setBar(0);
        setF(1);
    }

    void handleSerial() {
        while(true) {
            String str = port.readStringUntil('\n');
            if(str == null)
                return;
            String[] tokens = str.split(" ");
            // print(str);
            if(str.startsWith("button")) {
                pressed = tokens[1].charAt(0) == '1';
                if(pressed) {
                    hw = new Handwritting();
                    // port.write("play 2\n");
                } else {
                    finish();
                    // port.write("stop\n");
                }
            }
            if(str.startsWith("yrp")) {
                float[] yrp = new float[3];  // rotate asix: z-x-y 
                yrp[0] = Float.parseFloat(tokens[1]);
                yrp[1] = Float.parseFloat(tokens[2]);
                yrp[2] = Float.parseFloat(tokens[3]);
                float ay = Float.parseFloat(tokens[4]);
                if(pd.isPush(ay)) {
                    setBar(barValue + 1);
                }
                // println("yrp:\t" + yrp[0]*180.0f/PI + "\t" + yrp[1]*180.0f/PI + "\t" + yrp[2]*180.0f/PI);
                if(pressed) {
                    PVector p = new PVector(yrp[0]*180.0f/PI, yrp[1]*180.0f/PI);
                    hw.push(p);
                }
            }
            if(str.startsWith("ack")) {
                print(str);
            }
        }
    }
}

void draw_dir(float x, float y, int dir) {
    int size = 2;
    pushMatrix();
    translate(x, y);
    rotate(-PI / 2 * dir);
    triangle(0, size*2, -size, 0, size, 0);
    popMatrix();
}

void draw_circle(float x, float y, float r) {
    ellipse(x-r, y-r, r*2, r*2);
}

class Handwritting {
    List<PVector> points = new ArrayList<PVector>();

    List<Integer> dirs = new ArrayList<Integer>();
    List<Integer> dir_change_ids = new ArrayList<Integer>();
    final int UP = 0;
    final int RIGHT = 1;
    final int DOWN = 2;
    final int LEFT = 3;

    List<Integer> corner_ids = new ArrayList<Integer>();
    final float CORNER_ANGLE = PI / 4; // 45 degree

    PVector pmin, pmax;

    int get_direction(PVector p) {
        if(abs(p.x) > abs(p.y)) {
            if(p.x > 0)
                return RIGHT;
            return LEFT;
        } else {
            if(p.y > 0)
                return UP;
            return DOWN;
        }
    }
    void push(PVector p) {
        points.add(p);
    }
    void finish() {
        thin(2);
        smooth(0.2);
        PVector last_d = new PVector(0, 0, 0);
        for(int i=0; i<points.size()-1; ++i) {
            PVector d = PVector.sub(points.get(i+1), points.get(i));
            int dir = get_direction(d);
            dirs.add(dir);
            if(i >= 1 && dir == dirs.get(i-1) &&
                (dir_change_ids.isEmpty() || dir != dirs.get(dir_change_ids.get(dir_change_ids.size()-1))))
                dir_change_ids.add(i);

            float angle = PVector.angleBetween(d, last_d);
            if(angle > CORNER_ANGLE)
                corner_ids.add(i);

            last_d = d;
        }

    }
    void draw() {
        stroke(255); noFill();
        for(PVector p: points) {
            point(p.x * scale, p.y * scale);
        }
        for(int i: dir_change_ids) {
            PVector p = points.get(i);
            draw_dir(p.x * scale, p.y * scale, dirs.get(i));
        }
        noStroke(); fill(255, 0, 0);
        for(int i: corner_ids) {
            PVector p = points.get(i);
            draw_circle(p.x * scale, p.y * scale, 2);
        }
        stroke(255); noFill();
        if(pmin != null && pmax != null) {
            rect(pmin.x * scale, pmin.y * scale, (pmax.x - pmin.x) * scale, (pmax.y - pmin.y) * scale);
        }
    }
    void smooth(float t) {
        for(int i=1; i<points.size(); ++i) {
            PVector last = points.get(i-1);
            PVector p = points.get(i);
            PVector newp = PVector.add(last.copy().mult(t), p.copy().mult(1-t));
            points.set(i, newp);
        }
    }
    void thin(float dist) {
        List<PVector> ps = new ArrayList<PVector>();
        PVector last = null;
        for(PVector p: points) {
            if(last != null &&  PVector.sub(p, last).mag() < dist)
                continue;
            last = p;
            ps.add(p);
        }
        points = ps;
    }
    boolean has4Dir() {
        for(int i=0; i<=dir_change_ids.size()-4; ++i) {
            boolean[] exist = new boolean[4];
            for(int j=0; j<4; ++j) {
                exist[dirs.get(dir_change_ids.get(i+j))] = true;
            }
            int count = 0;
            for(int j=0; j<4; ++j) 
                count += exist[j]? 1: 0;
            print(count);
            if(count == 4) 
                return true;
        }
        return false;
    }
    boolean isCircle() {
        if(dir_change_ids.size() > 5 || corner_ids.size() > 1)
            return false;
        return has4Dir();
    }
    boolean isSquare() {
        // Test if there is no point at centeral small square
        float x0 = 1e3, x1 = -1e3;
        float y0 = 1e3, y1 = -1e3;
        for(PVector p: points) {
            x0 = Float.min(x0, p.x);
            x1 = Float.max(x1, p.x);
            y0 = Float.min(y0, p.y);
            y1 = Float.max(y1, p.y);
        }
        float t = 0.25;
        float x2 = x0 * (1-t) + x1 * t;
        float x3 = x1 * (1-t) + x0 * t;
        float y2 = y0 * (1-t) + y1 * t;
        float y3 = y1 * (1-t) + y0 * t;
        pmin = new PVector(x2, y2);
        pmax = new PVector(x3, y3);
        for(PVector p: points) 
            if(p.x > x2 && p.x < x3 && p.y > y2 && p.y < y3)
                return false;
        // Test if there are 4 dirs
        return has4Dir();
    }
}

class PushDetector {
    long lastPushMs;
    boolean isPush(float ay) {
        if(abs(ay) < 10000)
            return false;
        long nowMs = System.currentTimeMillis();
        if(nowMs - lastPushMs < 200)
            return false;
        lastPushMs = nowMs;
        return true;
    }
}