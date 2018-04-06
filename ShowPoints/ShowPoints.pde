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

ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;

float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];  // rotate asix: z-y-x
float[] yrp = new float[3];  // rotate asix: z-x-y 

final int scale = 2;

void setupSerial() {
    // display serial port list for debugging/clarity
    println(Serial.list());

    // get the first available port (use EITHER this OR the specific port code below)
    //String portName = Serial.list()[4];
    
    // get a specific serial port (use EITHER this OR the first-available code above)
    String portName = "/dev/cu.usbmodem1421";
    // String portName = "/dev/cu.HC-05-DevB";
    
    // open the serial port
    port = new Serial(this, portName, 115200);
    
    //Serial bluetooth1 = new Serial(this, "/dev/cu.HC-05-DevB", 9600);
    //Serial bluetooth2 = new Serial(this, "/dev/cu.HC-05-DevB-1", 9600);
    
    // send single character to trigger DMP init/start
    // (expected by MPU6050_DMP6 example Arduino sketch)
    //port.write('r');
}

void setup() {    
    // 300px square viewport using OpenGL rendering
    // size(300, 300, P3D);
    size(720, 360);
    gfx = new ToxiclibsSupport(this);

    // setup lights and antialiasing
    lights();
    smooth();
    
    setupSerial();
}

Handwritting hw;
boolean pressed = false;

void draw() {
    handleSerial(port);
    background(0);
    
    translate(width / 2, height / 2);
    if(hw != null)
        hw.draw();
}

void handleSerial(Serial port) {
    while(true) {
        String str = port.readStringUntil('\n');
        if(str == null)
            return;
        String[] tokens = str.split(" ");
        if(str.startsWith("button")) {
            pressed = tokens[1].charAt(0) == '1';
            if(pressed) {
                hw = new Handwritting();
                // port.write("play 2\n");
            } else {
                hw.finish();
                // port.write("stop\n");
            }
        }
        if(str.startsWith("yrp") && pressed) {
            yrp[0] = Float.parseFloat(tokens[1]);
            yrp[1] = Float.parseFloat(tokens[2]);
            yrp[2] = Float.parseFloat(tokens[3]);
            // println("yrp:\t" + yrp[0]*180.0f/PI + "\t" + yrp[1]*180.0f/PI + "\t" + yrp[2]*180.0f/PI);
        
            PVector p = new PVector(yrp[0]*180.0f/PI, yrp[1]*180.0f/PI);
            hw.push(p);
        }
        if(str.startsWith("ack")) {
            print(str);
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
    // boolean kill = false;
    void push(PVector p) {
        // if(!kill)
            points.add(p);
        // kill = !kill;
    }
    void finish() {
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
        stroke(255);
        noFill();
        for(PVector p: points) {
            point(p.x * scale, p.y * scale);
        }
        for(int i: dir_change_ids) {
            PVector p = points.get(i);
            draw_dir(p.x * scale, p.y * scale, dirs.get(i));
        }
        noStroke();
        fill(255, 0, 0);
        for(int i: corner_ids) {
            PVector p = points.get(i);
            draw_circle(p.x * scale, p.y * scale, 2);
        }
    }
}