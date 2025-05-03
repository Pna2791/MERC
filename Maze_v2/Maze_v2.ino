
#define cell_size       27
#define button_pin      A6


#include "wheel.h"
#include "maze_encoder.h"
#include "my_maze.h"
#include "my_hi229.h"

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"


int global_dir = 0;
int global_dir_index = 0;
PDController forward_pid    = PDController(3, 0.1, 2*FW_SPEED/3);
PDController turn_pid       = PDController(0.65, 0.035, TURN_SPEED);

Motor motor_left = Motor(11, 10);   // dir, speed
Motor motor_right = Motor(12, 9);
Chassis dual_wheel = Chassis(motor_left, motor_right);

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled;

My_Maze my_maze;
int read_button(){
    int value = analogRead(button_pin) + 100;
    return value / 205;
}


#define TOP     5
#define BOT     1
#define LEF     4
#define RIG     2
#define CEN     3

#define n_maps  5
int map_index = 0;


#define COLS    20
void savemap_mode(int index=0){
    oled.clear();
    oled.setCursor(0, 0);
    oled.print("Typing");
    while(true){
        int btt_value = read_button();
        if(btt_value && btt_value != 3){
            oled.setCursor(index%COLS*6, (index%100)/COLS+2);
            if(btt_value == 5){
                my_maze.save(index, 0);
                oled.print('F');
            }
            if(btt_value == 4){
                my_maze.save(index, 1);
                oled.print('L');
            }
            if(btt_value == 1){
                my_maze.save(index, 2);
                oled.print('B');
            }
            if(btt_value == 2){
                my_maze.save(index, 3);
                oled.print('R');
            }
            delay(250);
            index++;
        }
    }

}

void show_map_index(){
    oled.setCursor(0, 2);
    oled.print("Checkpoint: " + String(map_index));
}
void choose_map(){
    show_map_index();
    while(true){
        int btt_value = read_button();
        if(btt_value == LEF){
            if(map_index == 0)  map_index = n_maps;
            else                map_index--;
            show_map_index();
            delay(250);
        }
        if(btt_value == RIG){
            if(map_index == n_maps) map_index = 0;
            else                    map_index++;
            show_map_index();
            delay(250);
        }
        if(btt_value == CEN){
            savemap_mode(map_index*100);
        }
    }
}

void setup(){
    setup_encoder();

    Wire.begin();
    Wire.setClock(400000L);
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(Adafruit5x7);
    
    Serial.begin(115200);
    pinMode(button_pin, INPUT);
    
    delay(10);
    if(read_button() == CEN){    
        oled.println("Set map");
        delay(1000);  
        oled.clear();
        delay(1000);
        savemap_mode();
    }
    if(read_button() == TOP){    
        oled.println("Update maps");
        delay(1000);  
        oled.clear();
        delay(1000);
        choose_map();
    }
}




void turn_forward(int value=25, int n_step=1){
    forward_pid.reset();
    long next_checkpoint = encoderCount + plush_per_cm*value;

    // long time_out = millis() + 900;
    // while(millis() < time_out){
    while(long(encoderCount+ENCODER_OFFSET) < next_checkpoint){
        int direction = get_direction();
        if(direction != 0xFFF){
            if(global_dir > 1350 && direction < -450)       direction += 3600;
            else if(global_dir > 450 && direction < -1350)  direction += 3600;

            if(global_dir < -450 && direction > 450)
                direction -= 3600;

            int forward_value = forward_pid.compute((global_dir), direction);
            dual_wheel.move_forward(forward_value);
        }
    }
    dual_wheel.stop();
}



void turn_CCW(int delta){
    global_dir = (global_dir+3600)%3600;
    global_dir += delta;
    if(global_dir > 2250)   global_dir -= 3600;

    turn_pid.reset();
    float mean_error = 900;
    long time_out = millis() + 600;
    while(true){
        int direction = get_direction();
        if(direction != 0xFFF){
            mean_error = 0.6*mean_error + 0.4*(abs(global_dir-direction)%3600);
            if(mean_error < 100)        break;
            if(millis() > time_out)     break;

            if(global_dir > 1350 && direction < -450)       direction += 3600;
            else if(global_dir > 450 && direction < -1350)  direction += 3600;

            if(global_dir < -450 && direction > 450)
                direction -= 3600;

            int turn_value = turn_pid.compute(global_dir, direction);
            dual_wheel.rotate_CCW(turn_value);
        }
    }
}


void run_traced(int dir, int n_step=1){
    int deltal = int(4 + dir-global_dir_index)%4;
    global_dir_index = dir;

    if(deltal == 1){
        turn_CCW(900);
    }else if(deltal == 0){
        // Forward
    }else if(deltal == 3 || deltal == -1){
        turn_CCW(-900);
    }else{
        turn_CCW(1800);
    }
    dual_wheel.stop();

    turn_forward(cell_size*n_step, n_step);
    dual_wheel.stop();
}


void run_map(){
    global_dir = 0;
    global_dir_index = 0;
    int step_count = 100*map_index;
    delay(1000);

    while(true){
        int dir = my_maze.load(step_count);
        if(dir == 255)  break;
        int n_step = 1;
        
        for(int i=1; i<22; i++){
            if(dir == my_maze.load(step_count+i))   n_step = i+1;
            else                                    break;
        }

        // run by direction.
        run_traced(dir, n_step);
        step_count += n_step;
    }
    dual_wheel.stop();
}
void reset_IMU(){
    Serial.println("AT+RST");
    delay(1000);
    Serial.println("AT+RST");
}
void test_line(){
    delay(1000);
    global_dir = 0;
    turn_forward(300);

    dual_wheel.stop();
}


void handle_button(){
    int btt_value = read_button();
    if(btt_value == 0)  return;

    if(btt_value == CEN)    run_map();
    if(btt_value == BOT)    reset_IMU();
    if(btt_value == TOP)    test_line();
    if(btt_value == LEF){
        if(map_index == 0)  map_index = n_maps;
        else                map_index--;
    }
    if(btt_value == RIG){
        if(map_index == n_maps) map_index = 0;
        else                    map_index++;
    }
    delay(250);
}



void sy_map(){
    delay(1000);

    turn_forward(90);
    turn_CCW(-900);
    turn_forward(25);
    turn_CCW(-900);

    turn_forward(90);
    turn_CCW(900);
    turn_forward(25);
    turn_CCW(900);

    turn_forward(90);
    turn_CCW(-900);
    turn_forward(25);
    turn_CCW(-900);

    turn_forward(90);
}


void check_hcmute(){
    static bool run_mode = true;
    if(run_mode && encoderCount){
        sy_map();
        run_mode = false;
    }
}


void loop(){
    handle_button();

    int direction = get_direction();
    if(direction != 0xFFF){
        oled.setCursor(0, 0);
        oled.print(String(direction) + "  ");

        oled.setCursor(0, 1);
        oled.print(String(encoderCount) + "  ");

        show_map_index();
    }

}
