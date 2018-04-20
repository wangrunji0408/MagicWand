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
import processing.sound.*;
import java.nio.*;
import java.util.*;

final String PATH = "/Users/wangrunji/Documents/Codes/Tsinghua/Grade3-2/Entertainment/Magic";
final int scale = 2;
boolean use1 = true, use2 = true;
Device device1, device2;
IGame game;

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
    
    SoundPlayer sound = new SoundPlayer(this);
    if(use1)
    try {
        device1 = new Device();
        device1.port = new Serial(this, "/dev/cu.HC-05-DevB-1", 115200);
        device1.sound = sound;
        device1.init();
        println("Device1 OK");        
    } catch (Exception e) {
        device1 = null;        
        println("Device1 Failed");
    }
    if(use2)
    try {
        device2 = new Device();
        device2.port = new Serial(this, "/dev/cu.HC-05-DevB-2", 115200);
        device2.sound = sound;
        device2.init();
        println("Device2 OK"); 
    } catch (Exception e) {
        device2 = null;
        println("Device2 Failed");
    }
}

void setup() {
    // 300px square viewport using OpenGL rendering
    // size(300, 300, P3D);
    size(720, 360);

    // setup lights and antialiasing
    lights();
    smooth();
    
    setupSerial();
    game = new Game2(device1, device2);
}

void draw() {    
    background(0);
    translate(width / 2, height / 2);
    game.draw();
    if(device1 != null) {
        device1.handleSerial();
        device1.draw();
    }
    if(device2 != null) {
        device2.handleSerial();
        device2.draw();
    }
}

class Device {
    SoundPlayer sound;
    Serial port;
    Handwritting hw = new Handwritting();
    PushDetector pd = new PushDetector();
    CollectManager cm = new CollectManager();
    boolean pressed = false;
    int barValue = -1;
    int lastF = 1;
    boolean autoF = true;
    Runnable finishHandler;
    Runnable pushHandler;

    void setBar(int value) {
        value = Integer.min(9, value);
        value = Integer.max(0, value);
        if(value != barValue)
            port.write("setbar " + Integer.toString(value) + "\n");
        barValue = value;
    }

    void setF(int f) {
        f = Integer.min(9, f);
        f = Integer.max(0, f);
        if(f != lastF)
            port.write("setf " + Integer.toString(f) + "\n");
        lastF = f;        
    }

    void finish() {
        hw.finish();
        // print("Finish: ");
        // if(hw.isCircle())
        //     print("Circle ");
        // if(hw.isSquare())
        //     print("Square ");
        // int round = hw.calcRound();
        // print("Round="); print(round); println();
        if(finishHandler != null)
            finishHandler.run();
    }

    void draw() {
        cm.draw();
        if(hw != null)
            hw.draw();
    }

    void init() {
        setBar(1);
        setF(1);
    }

    void handlePos(PVector p, float ay) {
        if(pd.isPush(ay)) {
            if(pushHandler != null)
                pushHandler.run();
        }
        if(pressed) {
            hw.push(p);
        }
        cm.updatePos(p);
        if(autoF)        
            setF(cm.canCollect()? 0: cm.getFreq());
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
                try {
                    float[] yrp = new float[3];  // rotate asix: z-x-y 
                    yrp[0] = Float.parseFloat(tokens[1]);
                    yrp[1] = Float.parseFloat(tokens[2]);
                    yrp[2] = Float.parseFloat(tokens[3]);
                    float ay = Float.parseFloat(tokens[4]);
                    PVector p = new PVector(yrp[0]*180.0f/PI, yrp[1]*180.0f/PI);
                    handlePos(p, ay);
                } catch (Exception e) {
                    println("Failed to parse yrp!");
                }
                // println("yrp:\t" + yrp[0]*180.0f/PI + "\t" + yrp[1]*180.0f/PI + "\t" + yrp[2]*180.0f/PI);
            }
            if(str.startsWith("ack")) {
                // print(str);
            }
        }
    }
}

interface IGame {
    void draw();
}

// 任意时刻收集/攻击
class Game1 implements IGame {
    Device d1, d2;
    int hp1, hp2;
    PImage bgp = loadImage(PATH + "/picture/background.jpeg");

    Game1(Device device1, Device device2)
    {
        assert(device1 != null && device2 != null);
        hp1 = 10;
        hp2 = 10;
        this.d1 = device1;
        // d1.setBar(9);
        this.d2 = device2;
        d1.finishHandler = new Runnable() {
            public void run() {
                // int round = d1.hw.calcRound();
                int round = d1.hw.getAlphabet();
                if(d1.barValue < round)
                    return;
                d1.setBar(d1.barValue - round);
                playSound(round);
                hp2 -= round;
            }
        };
        d2.finishHandler = new Runnable() {
            public void run() {
                // int round = d2.hw.calcRound();
                int round = d2.hw.getAlphabet();                
                if(d2.barValue < round)
                    return;
                d2.setBar(d2.barValue - round);
                playSound(round);
                hp1 -= round;
            }
        };
        d1.pushHandler = new Runnable() {
            public void run() {
                if(d1.cm.canCollect()) {
                    d1.cm.collect();
                    d1.setBar(d1.barValue + 1);      
                    d1.sound.play("coin");
                }
            }
        };
        d2.pushHandler = new Runnable() {
            public void run() {
                if(d2.cm.canCollect()) {
                    d2.cm.collect();
                    d2.setBar(d2.barValue + 1);      
                    d2.sound.play("coin");
                }
            }
        };
    }
    void playSound(int round) {
        if(round == 1)
            d1.sound.play("风声");
        else if(round == 2)
            d1.sound.play("光波");
        else if(round == 3)
            d1.sound.play("闪耀");
    }
    void draw() {
        background(bgp);
        
        // HP Bar
        noStroke();
        fill(255, 0, 0);
        text("P1", -340, -140);
        text("P2", -340, -100);
        rect(-300, -160, hp1 * 20, 10);
        rect(-300, -120, hp2 * 20, 10);
    }
}

// 王芷规则：
// 1. 开始时集中收集，结束后随机分配ABC个数
// 2. 之后回合制出招A>B>C，大克小
class Game2 implements IGame {
    Device d1, d2;
    int[] count1 = new int[4];
    int[] count2 = new int[4];
    int last1, last2;
    int total1, total2;
    int score1, score2;
    long startTime = System.currentTimeMillis() / 1000;
    int stage = -1, lastStage = -1;
    final int COLL_TIME = 10;
    final int ROUND_TIME = 10;
    PImage bgp = loadImage(PATH + "/picture/background.jpeg");

    Game2(Device device1, Device device2)
    {
        assert(device1 != null && device2 != null);
        this.d1 = device1;
        this.d2 = device2;
        d1.pushHandler = new Runnable() {
            public void run() {
                if(d1.cm.canCollect()) {
                    d1.cm.collect();
                    total1 ++;
                    d1.sound.play("coin");
                }
            }
        };
        d2.pushHandler = new Runnable() {
            public void run() {
                if(d2.cm.canCollect()) {
                    d2.cm.collect();
                    total2 ++;
                    d2.sound.play("coin");
                }
            }
        };
    }
    int getStage() {
        int time = getTimeS();
        if(time < COLL_TIME)
            return 0;
        return (time - COLL_TIME) / ROUND_TIME + 1;
    }
    int getCountdown() {
        int time = getTimeS();
        if(time < COLL_TIME)
            return COLL_TIME - time;
        return ROUND_TIME-1 - (time - COLL_TIME) % ROUND_TIME;
    }
    int getTimeS() {
        return (int)(System.currentTimeMillis() / 1000 - startTime);
    }
    void playSound(int round) {
        if(round == 1)
            d1.sound.play("风声");
        else if(round == 2)
            d1.sound.play("光波");
        else if(round == 3)
            d1.sound.play("闪耀");
    }
    void generate(int total, int[] c) {
        if(total > 10) {
            c[1] = 2; c[2] = 1; c[3] = 2;
        } else if(total > 5) {
            c[1] = 3; c[2] = 1; c[3] = 1;
        } else {
            c[1] = 3; c[2] = 2; c[3] = 0;
        }
    }
    String countToStr(int[] c) {
        return "A" + Integer.toString(c[3]) 
            + " B" + Integer.toString(c[2])
            + " C" + Integer.toString(c[1]);
    }
    void event() {
        stage = getStage();
        if(stage == 1 && lastStage == 0) {
            d1.autoF = false; d1.setF(0);
            d2.autoF = false; d2.setF(0);
            d1.pushHandler = null;
            d2.pushHandler = null;
            d1.finishHandler = new Runnable() {
                public void run() {
                    last1 = d1.hw.calcRound();
                    if(count1[last1] > 0)
                        playSound(last1);
                }
            };
            d2.finishHandler = new Runnable() {
                public void run() {
                    last2 = d2.hw.calcRound();
                    if(count2[last2] > 0)
                        playSound(last2);
                }
            };
            generate(total1, count1);
            generate(total2, count2);
        } else if(stage != lastStage && lastStage <= 5) {
            // check validation
            if(count1[last1] > 0)
                count1[last1]--;
            else
                last1 = 0;
            if(count2[last2] > 0)
                count2[last2]--;
            else
                last2 = 0;
            // debug
            print("Stage "); print(lastStage); print(": "); 
            print(last1); print(" "); println(last2);
            // score
            if(last1 > last2)
                score1++;
            else if(last2 > last1)
                score2++;
            // reset
            last1 = last2 = 0;
        }
        lastStage = stage;
    }
    void draw() {
        event();
        background(bgp);
        
        noStroke();
        fill(255, 0, 0);
        textSize(100);
        if(stage > 5) {
            text("GameOver", 0, 0);
        } else {
            text(Integer.toString(getCountdown()), 0, 0);
        }
        textSize(50);
        text(Integer.toString(score1), -200, 0);
        text(Integer.toString(score2), 200, 0);
        text(countToStr(count1), -250, -100);
        text(countToStr(count2), 150, -100);
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

    int[][] alphabets = {
        {UP, LEFT, DOWN, RIGHT, UP, -1},    // alpha
        {DOWN, RIGHT, UP, LEFT, RIGHT, UP, LEFT, -1},    // beta
        {UP, LEFT, DOWN, -1},           // gamma1
        {RIGHT, UP, LEFT, DOWN, -1},    // gamma2
        {UP, LEFT, DOWN, RIGHT, -1},    // gamma3
        {RIGHT, UP, LEFT, DOWN, RIGHT, -1} // gamma4
    };

    final boolean DRAW_DEBUG = false;
    final boolean DRAW_LINK = true;

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
        PVector last = null;
        for(PVector p: points) {
            point(p.x * scale, p.y * scale);
            if(DRAW_LINK && last != null)
                line(last.x * scale, last.y * scale, p.x * scale, p.y * scale);
            last = p;
        }
        if(!DRAW_DEBUG)
            return;
        for(int i: dir_change_ids) {
            PVector p = points.get(i);
            draw_dir(p.x * scale, p.y * scale, dirs.get(i));
        }
        noStroke(); fill(255, 0, 0);
        for(int i: corner_ids) {
            PVector p = points.get(i);
            draw_circle(p.x * scale, p.y * scale, 2);
        }
        // stroke(255); noFill();
        // if(pmin != null && pmax != null) {
        //     rect(pmin.x * scale, pmin.y * scale, (pmax.x - pmin.x) * scale, (pmax.y - pmin.y) * scale);
        // }
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
            if(count == 4) 
                return true;
        }
        return false;
    }
    int calcRound() {
        int last_d = 0;
        int last_dir = -1;
        int count = 0;
        int max_count = 0;
        for(int id: dir_change_ids) {
            int dir = dirs.get(id);
            int d = (dir - last_dir + 4) % 4; // expect 1 or -1
            if(last_d == d) {
                count++;
                max_count = Integer.max(max_count, count);
            }
            else
                count = 0;
            last_d = d;
            last_dir = dir;
        }
        return (max_count + 1) / 4;
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
    boolean isAlphabet0(int k) {
        for(int i=0; i < dir_change_ids.size(); ++i) {
            if(dirs.get(dir_change_ids.get(i)) != alphabets[k][i])
                return false;
        }
        return alphabets[k][dir_change_ids.size()] == -1;
    }
    int getAlphabet() { 
        if(isAlphabet0(0))
            return 1;
        if(isAlphabet0(1))
            return 2;
        if(isAlphabet0(2) || isAlphabet0(3) || isAlphabet0(4) || isAlphabet0(5))
            return 3;
        return 0;
    }
    boolean isLastDirDown() {
        if(dirs.size() == 0)
            return false;
        println(dirs.get(dirs.size() - 1));
        return dirs.get(dirs.size() - 1) == UP;
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

class CollectManager {
    final int COLLECT_POINT_COUNT = 5;
    PVector[] collectPoints = new PVector[COLLECT_POINT_COUNT];
    PVector pos;

    CollectManager() {
        for(int i=0; i<COLLECT_POINT_COUNT; ++i)
            collectPoints[i] = newCollectPoint();
    }
    void collect() {
        if(!canCollect())
            return;
        int id = getMinDistId();
        collectPoints[id] = newCollectPoint();
    }
    float getMinDist() {
        float d = 1e3;
        for(PVector p: collectPoints)
            d = Float.min(d, dist(p, pos));
        return d;
    }
    int getMinDistId() {
        float d = 1e3;
        int id = -1;
        for(int i=0; i<COLLECT_POINT_COUNT; ++i) {
            float dd = dist(collectPoints[i], pos);
            if(dd >= d)
                continue;
            d = dd;
            id = i;
        }
        return id;
    }
    boolean canCollect() {
        return getFreq() >= 10;
    }
    int getFreq() {
        float degree = getMinDist() / PI * 180;
        return (int)(200 / degree);
    }
    void updatePos(PVector p) {
        pos = xy2real(p);
    }
    void draw() {
        stroke(0, 255, 0);
        for(PVector p: collectPoints) {
            if(p == null)
                continue;
            PVector xy = real2xy(p);
            point(xy.x * scale, xy.y * scale);
        }
        if(pos != null) {
            stroke(0, 0, 255);   
            PVector xy = real2xy(pos);     
            point(xy.x * scale, xy.y * scale);
        }
    }
    PVector real2xy(PVector p) {
        float y = asin(p.z) / PI * 180;
        float x = atan2(p.y, p.x) / PI * 180;
        return new PVector(x, y, 0);
    }
    PVector xy2real(PVector p) {
        float x = p.x / 180 * PI;
        float y = p.y / 180 * PI;
        return new PVector(cos(x) * cos(y), sin(x) * cos(y), sin(y));
    }
    PVector newCollectPoint() {
        PVector p = PVector.random3D();
        if(p.z < 0)
            p.z = -p.z;
        return p;
    }
    float dist(PVector p1, PVector p2) {
        return PVector.angleBetween(p1, p2);
    }
}

class SoundPlayer {
    Map<String, SoundFile> files = new HashMap();

    SoundPlayer(ShowPoints parent) {
        String soundPath = PATH + "/sound";
        files.put("风声", new SoundFile(parent, soundPath + "/风声卷过.mp3"));
        files.put("光波", new SoundFile(parent, soundPath + "/发射光波.mp3"));
        files.put("闪耀", new SoundFile(parent, soundPath + "/闪耀.mp3"));
        files.put("coin", new SoundFile(parent, soundPath + "/coin.mp3"));
    }

    void play(String name) {
        println("playing: " + name);
        files.get(name).play();
    }
}