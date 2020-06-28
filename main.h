#include <string>
#include <stdlib.h>

#include "mbed.h"
#include "SDBlockDevice.h"
#include "FATFileSystem.h"
#include "mbedtls/aes.h"

#include <Adafruit_GFX.h>
#include <TFT_ILI9163C.h>
// #include <Org_01.h>
#include <calibril_8.h>
#include <calibril_5.h>
#include "img.cpp"
#include "SerialHandler.h"
#include "SIM800.h"
#include "tinyxml.h"
#include "TinyGPS.h"

#if (OS_THREAD_OBJ_MEM == 0) 
    #define OS_THREAD_LIBSPACE_NUM 4 
#else
    #define OS_THREAD_LIBSPACE_NUM OS_THREAD_NUM 
#endif

#define ARDUINO_BUFFER_SIZE 1500
#define MENU_BUFFER_SIZE 5000
#define ARDUINO_SENSOR_COUNT 16
#define RTU_SENSOR_COUNT 3
#define SIM800_RETRIES 3
#define CALCULATED_DATA_COUNT 3
#define SENSOR_COUNT ARDUINO_SENSOR_COUNT + RTU_SENSOR_COUNT + CALCULATED_DATA_COUNT

#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define TRANSPARENT     -1

mbedtls_aes_context aes;

unsigned char key[16] = {
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
};

unsigned char iv[16];

EventQueue ev_queue(100 * EVENTS_EVENT_SIZE);

string arduino_sensors[ARDUINO_SENSOR_COUNT] = {"pt", "a1", "a2", "c1", "c2", "ra", "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8", "s9"};
string arduino_sensors_name[ARDUINO_SENSOR_COUNT] = {"PT100", "AI24(1)", "AI24(2)", "4-20(1)", "4-20(2)", "PERC(T)", "SDI#0", "SDI#1", "SDI#2", "SDI#3", "SDI#4", "SDI#5", "SDI#6", "SDI#7", "SDI#8", "SDI#9"};
string rtu_sensors[RTU_SENSOR_COUNT] = {"a3", "a4", "rs"};
string rtu_sensros_name[RTU_SENSOR_COUNT] = {"AI12(1)", "AI12(2)", "RS485"};
string calculated_data[CALCULATED_DATA_COUNT] = {"ra_1", "ra_12", "bat"};
string calculated_data_name[CALCULATED_DATA_COUNT] = {"PERC(1)", "PERC(12)", "BATT"};

int display_order[22] = {0, 1, 2, 16, 17, 3, 4, 5, 19, 20, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 18, 21};

RawSerial serial(PA_9, PA_10, 9600); //Serial_1
RawSerial gps(NRF_TX, NRF_RX, 9600); //Serial_3
RawSerial rs_menu(PA_2,PA_3, 14400); //serial_2
RawSerial arduino(PA_0, PA_1, 9600); //Serial_4
RawSerial   pc(PC_12, PD_2, 115200); //Serial_5

TFT_ILI9163C tft(PB_15, PB_14, PB_13, PC_6, PC_4, PA_7);

Watchdog &watchdog = Watchdog::get_instance();

SerialHandler ser(0);
SIM800 sim800(PB_7, PC_9);
TinyGPS GPS;

SDBlockDevice sd(SD_MOSI, SD_MISO, SD_SCLK, SD_CS, 20000000);
FATFileSystem fs("fs");

DigitalOut rs_control(PC_2);
DigitalOut led(PA_8, 1);
DigitalIn next_button(PA_15);
DigitalIn prev_button(PB_6);
DigitalIn power(PB_2, PullDown);
AnalogIn rtu_adc_1(PC_1);
AnalogIn rtu_adc_2(PC_3);
AnalogIn batt_volt(PB_0);

char arduino_buffer[ARDUINO_BUFFER_SIZE];
int arduino_buffer_index = 0;
int arduino_rx_state = 0;
int arduino_tag_start_index;

char menu_receive_buffer[MENU_BUFFER_SIZE];
char menu_parse_buffer[MENU_BUFFER_SIZE];
char rs_receive_buffer[10];
int rs_receive_index = 0;
int menu_buffer_index = 0;
int menu_parse_size = 0;
int lcd_page_number = 1;

bool sd_available = false;
bool menu_running = false;
bool new_serial_data = false;
bool incoming_serial_data = false;
bool incoming_rs_data = false;
bool time_set = false;
bool valid_location = false;
bool rs_data_available = false;
bool arduino_cli_ready = false;
bool arduino_cli_result_ready = false;
bool next_ready = false;
bool prev_ready = false;
bool on_battery = false;
bool one_time_gps_sent = false;
bool screen = true;
bool is_gps_parser_free = true;
bool is_arduino_parser_free = true;
bool log_to_sd = false;

float lat, lon;
int sd_log_interval = 300000; // ms
int data_sms_interval = 600000; //ms
int data_post_interval = 600000; // ms
int write_percip_interval = 300; //s
int update_check_interval = 3600000;
int update_retry_interval = 600000;
int cnt = 0;
int led_cnt = 0;
int batt_state = -1;
time_t last_data_timestamp = 0;
time_t last_log_timestamp = 0;
time_t last_gps_timestamp = 0;
time_t last_log_time = 0;
time_t last_data_sms_time = 0;
time_t last_gps_sms_time = 0;
time_t last_post_time = 0;
time_t last_update_check = 0;
time_t last_lcd_op = 0;
time_t pc_connect_time = 0;

string phone_no_1 = "";
string phone_no_2 = "";
string gprs_url = "http://gw.abfascada.ir/ahv_rtu/GetData.php";
string device_id = "000000";
double firmware_version = 2.7;
double temp_firmware_version;
bool created_file = false;
char disp[50];

char temp_buffer[1500];
unsigned char input[1510];
unsigned char output[1510];
char post_buffer[4000];
char sms_buffer[1500];
char ch[1360];
char filename[30];
char data_buffer[300];
char download_buffer[1500];
char sec_st[4], min_st[4], hr_st[4],day_st[4],month_st[4],year_st[4];

struct sensor_t{
    string name;
    string display_name;
    string raw_value;
    string scaled_value;
    string unit = "";
    string a = "1.0";
    string b = "0.0";
    string high_th = "";
    string low_th = "";
    bool valid_raw = false;
    bool valid_fun = false;
    bool send_in_sms = false;
    bool send_raw_in_sms = false;
    int sms_order = 0;
    int warning;
} sensor[SENSOR_COUNT];

void show_battery(){
    if(batt_state != -1){
        int x = 90, y = 3;
        if(batt_state == 3){
            tft.drawRGBBitmap(x, y, full_bat, 10, 23);
        }
        else if(batt_state == 2){
            tft.drawRGBBitmap(x, y, medium_bat, 10, 23);
        }
        else if(batt_state == 1){
            tft.drawRGBBitmap(x, y, low_bat, 10, 23);
        }
    }
}

void draw_footer(){
    if(!screen){
        return;
    }
    tft.drawRGBBitmap(0, 130, alt_wide, 160, 38);
    tft.setFont(&calibril_5);
    tft.setTextSize(1);
    tft.setTextColor(BLACK);
    tft.setCursor(100, 157);
    sprintf(disp,"fw:%.1f",firmware_version);
    tft.print(disp);
}

void show_gsm_state(bool state){
    if(!screen){
        return;
    }
    if(state){
        tft.drawRGBBitmap(106, 4, gsm_ok, 22, 22);
    }
    else{
        tft.drawRGBBitmap(106, 4, gsm_not_ok, 22, 22);
    }
}

void generate_random_iv(int sz, unsigned char* iv){
    srand((unsigned int)time(NULL));
    for (size_t i = 0; i < sz; i++){
        int r = rand();
        iv[i] = r % 255;
    }
}

void show_date_time(){
    if(!time_set){
        ev_queue.call_in(30000, show_date_time);
        return;
    }
    time_t seconds = time(NULL);
    last_lcd_op = seconds;
    strftime(min_st, 32, "%M", localtime(&seconds));
    strftime(sec_st, 32, "%S", localtime(&seconds));
    strftime(hr_st, 32, "%H", localtime(&seconds));
    strftime(day_st, 32, "%d", localtime(&seconds));
    strftime(month_st, 32, "%m", localtime(&seconds));
    strftime(year_st, 32, "%y", localtime(&seconds));
    int x = 3, y = 10;
    tft.setFont(&calibril_5);
    tft.setTextSize(1);
    tft.fillRect(0,0,85,20,BLACK); 

    tft.setTextColor(WHITE);

    tft.setCursor(x, y);
    sprintf(disp,"%s/%s/%s",year_st,month_st,day_st);
    tft.print(disp);

    tft.setCursor(x+45, y);
    sprintf(disp,"%s:%s",hr_st,min_st);
    tft.print(disp);
}

void status_update(string mystr){
    if(!screen){
        return;
    }
    last_lcd_op = time(NULL);
    int x = 35, y = 144;
    tft.setCursor(x, y);
    tft.fillRect(x,y-6,128,9,BLACK);
    tft.setTextSize(1);
    tft.setFont(&calibril_5);
    tft.setTextColor(YELLOW);
    tft.setCursor(x, y);
    sprintf(disp, "%s", mystr.c_str());
    tft.print(disp);
}

void show_sensor_data(){
    if(!screen){
        return;
    }
    tft.fillRect(0,20,10,17,BLACK);
    tft.setFont(&calibril_5);
    tft.setCursor(3, 27);
    tft.setTextColor(WHITE);
    sprintf(disp, "%d/5", lcd_page_number);
    tft.print(disp);

    tft.setFont(&calibril_8);
    tft.setTextSize(1);
    int x = 3, y = 42;
    int color[5] = {GREEN, WHITE, GREEN, WHITE, GREEN};
    if(lcd_page_number < 5){
        for(int i = 0; i < 5;i++){
            tft.fillRect(0,y-10,128,15,BLACK);
            tft.setCursor(x, y);
            tft.setTextColor(color[i]);
            int idx = display_order[(lcd_page_number - 1) * 5 + i];
            sprintf(disp, "%s: %s %s", sensor[idx].display_name.c_str(), sensor[idx].scaled_value.c_str(), sensor[idx].unit.c_str());
            tft.print(disp);
            y += 19;
        }
    }
    else{
        int x = 3, y = 42;
        tft.fillRect(x,y-10,128,15,BLACK);
        tft.setCursor(x, y);
        tft.setTextColor(color[0]);
        int idx = display_order[20];
        sprintf(disp, "%s: %s %s", sensor[idx].display_name.c_str(), sensor[idx].scaled_value.c_str(), sensor[idx].unit.c_str());
        tft.print(disp);
        y += 19;
        tft.fillRect(x,y-10,128,15,BLACK);
        tft.setCursor(x, y);
        tft.setTextColor(color[1]);
        if(valid_location){
            sprintf(disp, "%s: %f", "LAT", lat);
        }
        else{
            sprintf(disp, "%s:", "LAT");
        }
        tft.print(disp);
        y += 19;
        tft.fillRect(x,y-10,128,15,BLACK);
        tft.setCursor(x, y);
        tft.setTextColor(color[2]);
        if(valid_location){
            sprintf(disp, "%s: %f", "LON", lon);
        }
        else{
            sprintf(disp, "%s:", "LON");
        }
        
        tft.print(disp);
        y += 19;
        tft.fillRect(x,y-10,128,15,BLACK);
        // tft.setCursor(x, y);
        // tft.setTextColor(color[0]);
        // sprintf(disp, "%s: %s %s", "lon", "25.5", "C");
        // tft.print(disp);
        y += 19;
        tft.fillRect(x,y-10,128,15,BLACK);
        // tft.setCursor(x, y);
        // tft.setTextColor(color[0]);
        // sprintf(disp, "%s: %s %s", "lon", "25.5", "C");
        // tft.print(disp);
        y += 19;
    }
}

void update_lcd(){
    screen = true;
    show_sensor_data();
    show_date_time();
    show_battery();
    show_gsm_state(sim800.sim_registered);
    draw_footer();
    status_update("Running...");
    last_lcd_op = time(NULL);
}

void init_lcd(){
    tft.begin();
    tft.setBitrate(50000000);
    tft.setRotation(2);
    tft.clearScreen();
}

void logg(const char *fmt, ...){
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    if(log_to_sd && sd_available){
        Watchdog::get_instance().kick();
        time_t seconds = time(NULL);
        strftime(min_st, 32, "%M", localtime(&seconds));
        strftime(sec_st, 32, "%S", localtime(&seconds));
        strftime(hr_st, 32, "%H", localtime(&seconds));
        strftime(day_st, 32, "%d", localtime(&seconds));
        strftime(month_st, 32, "%m", localtime(&seconds));
        strftime(year_st, 32, "%y", localtime(&seconds));
        sprintf(filename, "/fs/log/%s-%s-%s_%s.txt", year_st, month_st, day_st, hr_st);
        FILE *file = fopen(filename, "a");
        if(file){
            fprintf(file, "%s-%s-%s %s:%s:%s -> ", year_st, month_st, day_st, hr_st, min_st, sec_st);
            va_start(args, fmt);
            vfprintf(file, fmt, args);
            fclose(file);
            va_end(args);
        }
    }
}

int check_sim800(){
    logg("\r\n\r\nFrom check_sim:\r\n");
    ser.sendCmd((char*)"\r");
    sim800.AT_CREG(READ_CMND, true);
    string data(sim800.data);
    if(data.compare("0,1") == 0){
        sim800.ATEx(0);
        return 0;
    }
    int retries = 0, r = 0;
    while(retries < SIM800_RETRIES){
        Watchdog::get_instance().kick();
        logg("Registering sim800 on the network...");
        sim800.disable();
        wait_us(100000);
        sim800.enable();
        wait_us(100000);
        sim800.power_key();
        while(r < 20){
            Watchdog::get_instance().kick();
            sim800.AT_CREG(READ_CMND, false);
            string data(sim800.data);
            if(data.compare("0,1") == 0 || data.compare("0,5") == 0){
                logg("Done\r\n");
                sim800.ATEx(0);
                sim800.sim_registered = true;
                wait_us(1000000);
                return 0;
            }
            r++;
            wait_us(1000000);
        }
        logg("Failed!\r\n");
        r = 0;
        retries++;
    }
    return -1;
}

void generate_sms_data(){
    int data_to_send_cnt = 0;
    for(int i = 0;i < SENSOR_COUNT;i++){
        if(sensor[i].send_in_sms){
            data_to_send_cnt++;
        }
    }

    int data_idx[(const int)data_to_send_cnt];

    for(int i = 0;i < SENSOR_COUNT;i++){
        if(sensor[i].send_in_sms){
            data_idx[sensor[i].sms_order - 1] = i;
        }
    } 

    
    // strftime(sms_buffer, 10, "%H:%M:%S", localtime(&last_data_timestamp));
    sprintf(sms_buffer, "%s", device_id.c_str());
    for(int i = 0;i < data_to_send_cnt;i++){
        int idx = data_idx[i];
        if(sensor[idx].valid_fun){
            sprintf(sms_buffer, "%s,%s", sms_buffer, sensor[idx].scaled_value.c_str());
        }
        else{
            sprintf(sms_buffer, "%s,_", sms_buffer);
        }
        if(sensor[idx].send_raw_in_sms && sensor[idx].valid_raw){
            sprintf(sms_buffer, "%s(%s)", sms_buffer, sensor[idx].raw_value.c_str());
        }
    }
}

void send_gps_sms(){
    logg("\r\n\r\nFrom send_gps_sms:\r\n");

    bool phone_1 = false, phone_2 = false;
    if(phone_no_1.length() > 0){
        phone_1 = true;
    }
    if(phone_no_2.length() > 0){
        phone_2 = true;
    }
    if(!phone_1 && !phone_2){
        logg("No phone numbers entered!");
        return;
    }
    status_update("Sending sms...");
    if(check_sim800() != 0){
        logg("Could not register SIM800 on network");
        return;
    }
    Watchdog::get_instance().kick();
    generate_sms_data();
    char temp[10];
    strftime(temp, 10, "%H:%M:%S", localtime(&last_gps_timestamp));
    sprintf(sms_buffer, "%s,%s", sms_buffer, temp);
    if(valid_location){
        sprintf(sms_buffer, "%s,%f,%f", sms_buffer, lat, lon);
    }
    else{
        sprintf(sms_buffer, "%s,_,_", sms_buffer);
    }

    sim800.AT_CMGF(WRITE_CMND, 1);
    sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
    if(phone_1){
        sim800.AT_CMGS(WRITE_CMND, phone_no_1, sms_buffer);
    }
    if(phone_2){
        sim800.AT_CMGS(WRITE_CMND, phone_no_2, sms_buffer);
    }
    status_update("Done.");
    one_time_gps_sent = true;
    ev_queue.call_in(7200000, send_gps_sms);
}

void parse_gps_data(){
    logg("\r\n\r\nFrom parse_gps_data\r\n");
	GPS.f_get_position(&lat, &lon);
    if(lat == TinyGPS::GPS_INVALID_F_ANGLE || lon == TinyGPS::GPS_INVALID_F_ANGLE){
        logg("GPS Invalid Angle\r\n");
        is_gps_parser_free = true;
        return;
    }
    logg("LAT=%f", lat);
    logg(" LON=%f\r\n", lon);
    if(time_set){
        last_gps_timestamp = time(NULL);
    }
    valid_location = true;
    is_gps_parser_free = true;
}

void gps_rx(){
	char c = gps.getc();
	if(GPS.encode(c)){
        if(is_gps_parser_free){
            is_gps_parser_free = false;
            ev_queue.call(parse_gps_data);
        }
	}
}

int string_to_double(string s, double* d){
    if(s.length() == 0){
        return -2;
    }
    int indx = s.find('.');
    string f;
    if (indx != -1) {
        f = s.substr(indx+1);
        s = s.substr(0, indx);
    }
    int neg = 1;
    if (s.find('-') != -1) {
        neg = -1;
        s = s.substr(1);
    }
    else if(s.find('+') != -1){
        neg = 1;
        s = s.substr(1);
    }

    double t = 0;
    int l = s.length();
    for(int i = l-1; i >= 0; i--){
        if(!isdigit(s[i])){
            return -1;
        }
        t += (s[i] - '0') * pow(10.0, l - i - 1);
    }
    l = f.length();
    for(int i = 0; i < l; i++){
        if(!isdigit(f[i])){
            return -1;
        }
        t += (f[i] - '0') * pow(10.0, -1 * (i+1));
    }
    *d = neg * t;
    return 0;
}

int string_to_int(string str, int* d){
    if(str.length() == 0){
        return -2;
    }
    int indx = str.find('.');
    if (indx != -1) {
        str = str.substr(0, indx);
    }
    int neg = 1;
    if (str.find('-') == 0) {
        neg = -1;
        str = str.substr(1);
    }

    double t = 0;
    int l = str.length();
    for(int i = l-1; i >= 0; i--){
        if(!isdigit(str[i])){
            return -1;
        }
        t += (str[i] - '0') * pow(10.0, l - i - 1);
    }
    *d = (int)(neg * t);
    return 0;
}

int8_t hex_ch_to_int(char chr){
    if(chr <= '9' && chr >= '0'){
        return chr - '0';
    }
    if(chr >= 'A' && chr <= 'F'){
        return chr - 'A' + 10;
    }
    if(chr >= 'a' && chr <= 'f'){
        return chr - 'a' + 10;
    }
    return -1;
}

int8_t hex_string_to_byte(string s){
    if (s.length() == 1){
        return hex_ch_to_int(s[0]);
    }
    
    if(s.length() != 2){
        return -1;
    }
    int8_t d0 = hex_ch_to_int(s[1]);
    int8_t d1 = hex_ch_to_int(s[0]);
    if (d1 == -1 || d0 == -1){
        return -2;
    }
    int8_t t = d0 + (d1<<4);
    return t;
}

bool string_to_bool(string s){
    if(s.compare("true") == 0){
        return true;
    }
    return false;
}

bool char_compare(char* a, char* b, int start, int end){
    int l_a = strlen(a);
    if (start < 0 || start >= l_a || end >= l_a || start >= end || l_a < end - start + 1 || strlen(b) < end - start + 1){
        return false;
    }

    for (size_t i = 0; i < end - start + 1; i++){
        if(a[start+i] != b[i]){
            return false;
        }
    }
    return true;
}

void get_time(){
    logg("\r\n\r\nFrom get_time\r\n");
    status_update("Sync time...");
    int retries = 3, result;
    int data_len = 0;
    while(retries > 0){
        if(check_sim800() != 0){
            logg("Could not register SIM800 on network");
            time_set = false;
            status_update("Failed");
            return;
        }
        Watchdog::get_instance().kick();
        show_gsm_state(true);
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "Contype", "GPRS");
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "APN", "www");
        sim800.AT_HTTPINIT(EXEC_CMND);
        sim800.AT_HTTPPARA(WRITE_CMND, "CID", "1");
        sim800.AT_HTTPPARA(WRITE_CMND, "URL", "http://gw.abfascada.ir/ahv_rtu/settings.php");
        wait_us(100000);
        result = sim800.AT_SAPBR(WRITE_CMND, 1, 1);
        if(result != 0){
            retries--;
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        wait_us(100000);
        result = sim800.AT_HTTPACTION(WRITE_CMND, 0, 20000);
        printf("action data: %s\r\n", sim800.data);
        if(result != 0){
            retries--;
            sim800.disable();
            show_gsm_state(false);
            continue;
        }

        result = sim800.AT_HTTPREAD(EXEC_CMND, &data_len, data_buffer);
        if(result != 0){
            retries--;
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        break;
    }
    if(retries <= 0){
        time_set = false;
        return;
        status_update("Failed");
    }
    wait_us(100000);
    sim800.AT_HTTPTERM(EXEC_CMND);
    wait_us(100000);
    sim800.AT_SAPBR(WRITE_CMND, 0, 1);
    data_buffer[data_len] = '\0';
    string temp_data(data_buffer);
    for(int i = 0;i < 4;i++){
        int idx = temp_data.find("%");
        temp_data = temp_data.substr(idx+1);
    }
    int idx = temp_data.find("%");
    temp_data = temp_data.substr(0, idx).c_str();
    logg("timestamp = %s\r\n", temp_data.c_str());
    int tm;
    string_to_int(temp_data, &tm);
    set_time(tm);
    time_set = true;
    status_update("Done.");
}

void parse_config_xml(TiXmlDocument doc){
    logg("\r\r\nFrom parse_config_xml:\r\n");
    string tag = doc.RootElement()->Value();
    
    for (TiXmlNode* child = doc.RootElement()->FirstChild(); child; child = child->NextSibling() ) {
        Watchdog::get_instance().kick();
        if (child->Type()==TiXmlNode::TINYXML_ELEMENT) {
            tag = child->Value();
            string text(child->ToElement()->GetText());
            if(tag.compare("device_id") == 0){
                device_id = text;
                logg("Loaded device_id = %s\r\n", device_id.c_str());
            }
            else if(tag.compare("sdi") == 0){
            }
            else if (tag.compare("rs485") == 0) {
            }
            else if (tag.compare("sms") == 0) {
                for (TiXmlNode* new_child = child->FirstChild(); new_child; new_child = new_child->NextSibling() ) {
                    Watchdog::get_instance().kick();
                    if (new_child->Type()==TiXmlNode::TINYXML_ELEMENT) {
                        tag = new_child->Value();
                        text = new_child->ToElement()->GetText();
                        if(tag.compare("p_n_1") == 0){
                            phone_no_1 = text;
                            logg("loaded phone_no_1 = %s\r\n", phone_no_1.c_str());
                        }
                        else if(tag.compare("p_n_2") == 0){
                            phone_no_2 = text;
                            logg("loaded phone_no_2 = %s\r\n", phone_no_2.c_str());
                        }
                        else if(tag.compare("interval") == 0){
                            int tmp;
                            if(string_to_int(text, &tmp) != 0){
                                continue;
                            }
                            data_sms_interval = tmp * 60000;
                            logg("loaded data_sms_interval = %d\r\n", data_sms_interval);
                        }
                    }
                }
            }
            else if (tag.compare("gprs") == 0) {
                for (TiXmlNode* new_child = child->FirstChild(); new_child; new_child = new_child->NextSibling() ) {
                    Watchdog::get_instance().kick();
                    if (new_child->Type()==TiXmlNode::TINYXML_ELEMENT) {
                        tag = new_child->Value();
                        text = new_child->ToElement()->GetText();
                        if(tag.compare("url") == 0){
                            gprs_url = text;
                            logg("loaded gprs_url = %s\r\n", gprs_url.c_str());
                        }
                        else if(tag.compare("interval") == 0){
                            int tmp;
                            if(string_to_int(text, &tmp) != 0){
                                continue;
                            }
                            data_post_interval = tmp * 60000;
                            logg("loaded data_post_interval = %d\r\n", data_post_interval);
                        }
                    }
                }
            }
            else if (tag.compare("encryption") == 0) {
                for (TiXmlNode* new_child = child->FirstChild(); new_child; new_child = new_child->NextSibling() ) {
                    if (new_child->Type()==TiXmlNode::TINYXML_ELEMENT) {
                        tag = new_child->Value();
                        text = new_child->ToElement()->GetText();
                        if(tag.compare("key") == 0){
                            string key_string = text;
                            if(key_string.length() != 32){
                                continue;
                            }
                            for(int i = 0;i < 16;i++){
                                key[i] = (unsigned char)hex_string_to_byte(key_string.substr(2 * i, 2));
                            }
                            char key_temp[65];
                            sprintf(key_temp, "loaded encryption key = 0x");
                            for(int i = 0;i<16;i++){
                                sprintf(key_temp, "%s%02X", key_temp, key[i]);
                            }
                            sprintf(key_temp, "%s\r\n", key_temp);
                            logg(key_temp);
                        }
                    }
                }
            }
            else if (tag.compare("s") == 0) {
                for (TiXmlNode* new_child = child->FirstChild(); new_child; new_child = new_child->NextSibling() ) {
                    Watchdog::get_instance().kick();
                    if (new_child->Type()==TiXmlNode::TINYXML_ELEMENT) {
                        logg("\r\n");
                        tag = new_child->Value();
                        int idx = -1;
                        for(int i = 0;i < SENSOR_COUNT;i++){
                            if(tag.compare(sensor[i].name) == 0){
                                idx = i;
                                break;
                            }
                        }
                        if(idx == -1){
                            logg("Invalid sensor name: %s", tag.c_str());
                            continue;
                        }
                        for(TiXmlNode* new_new_child = new_child->FirstChild();new_new_child;new_new_child = new_new_child->NextSibling()){
                            if(new_new_child->Type() == TiXmlNode::TINYXML_ELEMENT){
                                string var = new_new_child->Value();
                                text = new_new_child->ToElement()->GetText();
                                // if(text.length() == 0){
                                //     continue;
                                // }
                                if(var.compare("u") == 0){
                                    sensor[idx].unit = text;
                                    logg("loaded %s->unit = %s\r\n", tag.c_str(), sensor[idx].unit.c_str());
                                }
                                else if(var.compare("a") == 0){
                                    sensor[idx].a = text;
                                    logg("loaded %s->a = %s\r\n", tag.c_str(), sensor[idx].a.c_str());
                                }
                                else if(var.compare("b") == 0){
                                    sensor[idx].b = text;
                                    logg("loaded %s->b = %s\r\n", tag.c_str(), sensor[idx].b.c_str());
                                }
                                else if(var.compare("low") == 0){
                                    sensor[idx].low_th = text;
                                    logg("loaded %s->low_th = %s\r\n", tag.c_str(), sensor[idx].low_th.c_str());
                                }
                                else if(var.compare("high") == 0){
                                    sensor[idx].high_th = text;
                                    logg("loaded %s->high_th = %s\r\n", tag.c_str(), sensor[idx].high_th.c_str());
                                }
                                else if(var.compare("s_i_s") == 0){
                                    sensor[idx].send_in_sms = string_to_bool(text);
                                    logg("loaded %s->send_in_sms = %s\r\n", tag.c_str(), sensor[idx].send_in_sms?"true":"false");
                                }
                                else if(var.compare("s_r_i_s") == 0){
                                    sensor[idx].send_raw_in_sms = string_to_bool(text);
                                    logg("loaded %s->send_raw_in_sms = %s\r\n", tag.c_str(), sensor[idx].send_raw_in_sms?"true":"false");
                                }
                                else if(var.compare("s_o") == 0){
                                    int tmp;
                                    if(string_to_int(text, &tmp) != 0){
                                        continue;
                                    }
                                    sensor[idx].sms_order = tmp;
                                    logg("loaded %s->sms_order = %d\r\n", tag.c_str(), sensor[idx].sms_order);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    status_update("Done");
}

void serial_menu(){
    logg("\r\n\r\nFrom serial_menu:\r\n");
    if(!new_serial_data){
        logg("no new data\r\n");
        return;
    }
    logg("menu received = %s\r\n", menu_parse_buffer);
    string data(menu_parse_buffer);
    if(data.compare("connect") == 0){
        new_serial_data = false;
        logg("{\"device_id\":\"%s\", \"firmware_version\":\"%.1f\"}\r\n", device_id.c_str(), firmware_version);
        rs_menu.printf("{\"device_id\":\"%s\", \"firmware_version\":\"%.1f\"}\n", device_id.c_str(), firmware_version);
        menu_running = true;
    }
    arduino_cli_ready = false;
    arduino.printf("$enter$");
    int cntr = 0;
    status_update("Connected to pc");
    while(true){
        Watchdog::get_instance().kick();
        if(new_serial_data){
            cntr = 0;
            new_serial_data = false;
            logg("new serial data: %s\r\n", menu_parse_buffer);
            data = string(menu_parse_buffer);
            if(data.compare("disconnect") == 0){
                menu_running = false;
                arduino.printf("exit");
                status_update("Disconnected");
                break;
            }
            else if(data.compare("read_config") == 0){
                if(!sd_available){
                    logg("SD not available!\r\n");
                    rs_menu.printf("SD not available\n");
                    cntr = 0;
                    continue;
                }
                Watchdog::get_instance().kick();
                TiXmlDocument doc;
                FILE* file = fopen("/fs/config.xml", "r");
                doc.LoadFile(file);
                fclose(file);
                TiXmlPrinter printer;
                printer.SetLineBreak(""); //Default is "\n".
                doc.Accept(&printer);
                rs_menu.printf("%s\n", printer.CStr());
                doc.Clear();
                cntr = 0;
            }
            else if(data[0] == '<'){
                Watchdog::get_instance().kick();
                TiXmlDocument doc;
                doc.Parse(data.c_str());
                parse_config_xml(doc);
                FILE* file = fopen("/fs/config.xml", "w");
                doc.SaveFile(file);
                fclose(file);
                rs_menu.printf("OK\n");
                cntr = 1800;
                
            }
            else if(data.compare("scan_sdi") == 0){
                if(!arduino_cli_ready){
                    rs_menu.printf("Arduino cli not ready\n");
                    logg("Arduino cli not ready\r\n");
                    cntr = 0;
                    continue;
                }
                arduino_cli_ready = false;
                arduino.printf("s_sdi ??");
                logg("Command (s_sdi ??) sent to arduino\r\n");
                logg("Waiting for response\r\n");
                int timeout = 55;
                while(!arduino_cli_result_ready){
                    Watchdog::get_instance().kick();
                    logg(".\r\n");
                    wait_us(1000000);
                    timeout--;
                    if(timeout<=0){
                        logg("Timeout\r\n");
                        break;
                    }
                }
                if(!arduino_cli_result_ready){
                    cntr = 0;
                    continue;
                }
                rs_menu.printf("%s\n", arduino_buffer);
                arduino_cli_result_ready = false;
                cntr = 0;
            }
            else if(data.substr(0, 10).compare("change_sdi") == 0){
                if(!arduino_cli_ready){
                    rs_menu.printf("Arduino cli not ready\n");
                    logg("Arduino cli not ready\r\n");
                    cntr = 0;
                    continue;
                }
                arduino_cli_ready = false;
                arduino.printf("c_sdi %c%c ??", data[10], data[11]);
                logg("Command (c_sdi %c%c ??) sent to arduino\r\n", data[10], data[11]);
                logg("Waiting for response\r\n");
                int timeout = 55;
                while(!arduino_cli_result_ready){
                    Watchdog::get_instance().kick();
                    logg(".\r\n");
                    wait_us(1000000);
                    timeout--;
                    if(timeout<=0){
                        logg("Timeout\r\n");
                        break;
                    }
                }
                if(!arduino_cli_result_ready){
                    cntr = 0;
                    continue;
                }
                rs_menu.printf("%s\n", arduino_buffer);
                arduino_cli_result_ready = false;
                cntr = 0;
            }
            else if(data.substr(0, 10).compare("select_sdi") == 0){
                if(!arduino_cli_ready){
                    rs_menu.printf("Arduino cli not ready\n");
                    logg("Arduino cli not ready\r\n");
                    cntr = 0;
                    continue;
                }
                arduino_cli_ready = false;
                char temp[20];
                data = data.substr(10);
                sprintf(temp, "i_sdi %s ??", data.c_str());
                arduino.printf("%s", temp);
                logg("Command (%s) sent to arduino\r\n", temp);
                logg("Waiting for response\r\n");
                int timeout = 55;
                while(!arduino_cli_result_ready){
                    Watchdog::get_instance().kick();
                    logg(".\r\n");
                    wait_us(1000000);
                    timeout--;
                    if(timeout<=0){
                        logg("Timeout\r\n");
                        break;
                    }
                }
                if(!arduino_cli_result_ready){
                    cntr = 0;
                    continue;
                }
                rs_menu.printf("%s\n", arduino_buffer);
                arduino_cli_result_ready = false;
                cntr = 0;
            }
            else if(data.compare("get_sensor_data") == 0){
                cntr = 0;
                rs_menu.printf("<data>");
                for(int i = 0;i < SENSOR_COUNT;i++){
                    if(sensor[i].valid_fun){
                        rs_menu.printf("<%s>%s</%s>", sensor[i].name.c_str(), sensor[i].scaled_value.c_str(), sensor[i].name.c_str());
                    }
                }
                rs_menu.printf("</data>");
                rs_menu.putc('\n');
            }
        }
        wait_us(100000);
        cntr++;
        if(cntr >= 1800){
            menu_running = false;
            arduino.printf("exit");
            status_update("Disconnected");
            break;
        }
    }
}

void rs_menu_rx(){
    char c = rs_menu.getc();
    if(incoming_serial_data){
        if(c == '#'){
            incoming_serial_data = false;
            for(int i = 0;i<menu_buffer_index;i++){
                menu_parse_buffer[i] = menu_receive_buffer[i];
            }
            menu_parse_buffer[menu_buffer_index] = '\0';
            menu_buffer_index=0;
            new_serial_data = true;
            if(!menu_running){
                ev_queue.call(serial_menu);
            }
        }
        menu_receive_buffer[menu_buffer_index] = c;
        menu_buffer_index++;
        return;
    }
    if(incoming_rs_data){
        if(c == '*'){
            incoming_rs_data = false;
            rs_receive_buffer[9] = '\0';
            rs_receive_index=0;
            rs_data_available = true;
            return;
        }
        rs_receive_buffer[rs_receive_index] = c;
        rs_receive_index++;
        return;
    }
    if(c == '$'){
        incoming_serial_data = true;
        menu_buffer_index = 0;
    }
    if(c == '/'){
        incoming_rs_data = true;
        rs_receive_index=0;
    }
}

void init_SD(){
    logg("\r\n\r\nfrom init_SD:\r\n");
	logg("Mounting SD...");
	sd.init();
	int err = fs.mount(&sd);
	if (err) {
		logg("Failed!\r\n");
        status_update("No SD Card");
        wait_us(100000);
        return;
	}
	else{
		logg("Done\r\n");
		sd_available = true;
	}
     Watchdog::get_instance().kick();
    logg("Cheking for \"log\" directory...");
    DIR *d = opendir("/fs/log");
    if(d){
        logg("Exists.\r\n");
    }
    else{
        logg("Doesn't exist!\r\n Creating directory \"log\"...");
        int rs = mkdir("/fs/log", 777);
        if(rs == 0){
            logg("Done.\r\n");
        }
        else{
            logg("Failed with %d code\r\n", rs);
            return;
        }
    }

    logg("Cheking for \"percip\" directory...");
    d = opendir("/fs/percip");
    if(d){
        logg("Exists.\r\n");
    }
    else{
        logg("Doesn't exist!\r\n Creating directory \"percip\"...");
        int rs = mkdir("/fs/percip", 777);
        if(rs == 0){
            logg("Done.\r\n");
        }
        else{
            logg("Failed with %d code\r\n", rs);
            return;
        }
    }

    logg("Cheking for \"data\" directory...");
    d = opendir("/fs/data");
    if(d){
        logg("Exists.\r\n");
    }
    else{
        logg("Doesn't exist!\r\n Creating directory \"data\"...");
        int rs = mkdir("/fs/data", 777);
        if(rs == 0){
            logg("Done.\r\n");
        }
        else{
            logg("Failed with %d code\r\n", rs);
            return;
        }
    }
    logg("Cheking for \"raw\" directory...");
    d = opendir("/fs/data/raw");
    if(d){
        logg("Exists.\r\n");
    }
    else{
        logg("Doesn't exist!\r\n Creating directory \"raw\"...");
        int rs = mkdir("/fs/data/raw", 777);
        if(rs == 0){
            logg("Done.\r\n");
        }
        else{
            logg("Failed with %d code\r\n", rs);
            return;
        }
    }

    logg("Cheking for \"scaled\" directory...");
    d = opendir("/fs/data/scaled");
    if(d){
        logg("Exists.\r\n");
    }
    else{
        logg("Doesn't exist!\r\n Creating directory \"scaled\"...");
        int rs = mkdir("/fs/data/scaled", 777);
        if(rs == 0){
            logg("Done.\r\n");
        }
        else{
            logg("Failed with %d code\r\n", rs);
            return;
        }
    }
}

unsigned int roundUp(unsigned int numToRound, int multiple){
    if (multiple == 0){
        return numToRound;
    }

    int rm = numToRound % multiple;
    if (rm == 0){
        return numToRound;
    }
    return numToRound + multiple - rm;
}

void print_sensors(){
    logg("\r\n\r\nFrom print_sensors:\r\n");
    for(int i = 0;i < SENSOR_COUNT;i++){
        Watchdog::get_instance().kick();
        logg("%s: %s(%s)\r\n", sensor[i].name.c_str(), sensor[i].scaled_value.c_str(), sensor[i].raw_value.c_str());
    }
}

void send_alarm_sms(bool is_high, int idx){
    logg("\r\n\r\nFrom send_alarm_sms:\r\n");
    bool phone_1 = false, phone_2 = false;
    if(phone_no_1.length() > 0){
        phone_1 = true;
    }
    if(phone_no_2.length() > 0){
        phone_2 = true;
    }
    if(!phone_1 && !phone_2){
        logg("No phone numbers entered!");
        return;
    }
    status_update("Sending alarm sms");
    if(check_sim800() != 0){
        logg("Could not register SIM800 on network");
        show_gsm_state(false);
        return;
    }
    Watchdog::get_instance().kick();
    show_gsm_state(true);
    char text[500];
    strftime(text, 10, "%H:%M:%S", localtime(&last_data_timestamp));
    sprintf(text, "%s: %s ->", text, device_id.c_str());
    if(is_high){
        sprintf(text, "%s Alarm! %s's value is %s and is higher than it's high threshold: %s", text, sensor[idx].name.c_str(), sensor[idx].scaled_value.c_str(), sensor[idx].high_th.c_str());
    }
    else{
        sprintf(text, "%s Alarm! %s's value is %s and is lower than it's low threshold: %s", text, sensor[idx].name.c_str(), sensor[idx].scaled_value.c_str(), sensor[idx].low_th.c_str());
    }

    sim800.AT_CMGF(WRITE_CMND, 1);
    sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
    if(phone_1){
        sim800.AT_CMGS(WRITE_CMND, phone_no_1, text);
    }
    if(phone_2){
        sim800.AT_CMGS(WRITE_CMND, phone_no_2, text);
    }
    status_update("Done.");
}

void apply_function(int idx, double value){
    logg("\r\n\r\nFrom apply_function to %s\r\n", sensor[idx].name.c_str());
    double a, b;
    if(string_to_double(sensor[idx].a, &a) != 0){
        bool got_a = false;
        for(int i = 0; i < SENSOR_COUNT; i++){
            if(sensor[i].name.compare(sensor[idx].a) == 0){
                if(sensor[i].valid_fun){
                    string_to_double(sensor[i].scaled_value, &a);
                    got_a = true;
                    break;
                }
                else{
                    sensor[idx].valid_fun = false;
                    logg("Dependancy not ready\r\n");
                    return;
                }
            }
        }
        if(!got_a){
            a = 1.0;
        }
    }

    if(string_to_double(sensor[idx].b, &b) != 0){
        bool got_b = false;
        for(int i = 0; i < SENSOR_COUNT; i++){
            if(sensor[i].name.compare(sensor[idx].b) == 0){
                if(sensor[i].valid_fun){
                    string_to_double(sensor[i].scaled_value, &b);
                    got_b = true;
                    break;
                }
                else{
                    sensor[idx].valid_fun = false;
                    logg("Dependancy not ready\r\n");
                    return;
                }
            }
        }
        if(!got_b){
            b = 0.0;
        }
    }
    double sc_value = a * value + b;
    char temp[10];
    sprintf(temp, "%.2f", sc_value);
    sensor[idx].scaled_value = string(temp);
    sensor[idx].valid_fun = true;
    double high_th, low_th;
    if(string_to_double(sensor[idx].high_th, &high_th) == 0 && sc_value > high_th){
        // logg("Alarm! %s is higher than it's threshold(%f)", sensor[idx].name.c_str(), temp);
        sensor[idx].warning = 11;
        ev_queue.call(send_alarm_sms, true, idx);
    }
    if(string_to_double(sensor[idx].low_th, &low_th) == 0 && sc_value < low_th){
        // logg("Alarm! %s is lower than it's threshold(%f)", sensor[idx].name.c_str(), temp);
        sensor[idx].warning = 10;
        ev_queue.call(send_alarm_sms, false, idx);
    }
}

void write_percip() {
    Watchdog::get_instance().kick();
    logg("From write percip: \r\n");
    if (!sd_available) {
        logg("SD not available!\r\n");
        return;
    }
    
    int idx = -1;
    for(int i = 0;i < SENSOR_COUNT;i++){
        if(sensor[i].name.compare("ra") == 0){
            idx = i;
        }
    }

    if(idx != -1 && sensor[idx].valid_fun){
        char temp[30];
        unsigned int cl = roundUp(last_data_timestamp, write_percip_interval);
        sprintf(temp, "/fs/percip/%u.txt", cl);
        logg("Opening file %s...", temp);
        FILE *file = fopen(temp, "w");
        if (!file) {
            logg("Failed\r\n");
            return;
        }
        else {
            logg("Done\r\n");
        }
        fprintf(file, "%s", sensor[idx].scaled_value.c_str());
        fclose(file);
    }
    else{
        logg("Rain data not available!\r\n");
    }
}

void read_percip_1(){
    Watchdog::get_instance().kick();
    logg("\r\n\r\nFrom read_percip_1:\r\n");

    if (!sd_available) {
        logg("SD not available!\r\n");
        return;
    }
    if(!time_set){
        logg("Time is not set!");
        return;
    }
    time_t time_stamp = time(NULL);
    unsigned int cl1 = roundUp(time_stamp - 3600, write_percip_interval);
    char temp[30];
    sprintf(temp, "/fs/percip/%u.txt", cl1);
    logg("Opening file %s...", temp);
    FILE *f = fopen(temp, "r");
    float hr_1;
    if (!f) {
        logg("Failed\r\n");
    }
    else{
        fscanf(f, "%f", &hr_1);
        int idx=-1, idx_1=-1;
        for(int i = 0;i<SENSOR_COUNT;i++){
            if(sensor[i].name.compare("ra_1") == 0){
                idx_1 = i;
            }
            if(sensor[i].name.compare("ra") == 0){
                idx = i;
            }
        }
        char temp[10];
        double sc_value;
        if(string_to_double(sensor[idx].scaled_value, &sc_value) != 0){
            return;
        }

        sc_value -= hr_1;
        sprintf(temp, "%.2f", sc_value);
        sensor[idx_1].scaled_value = string(temp);
        apply_function(idx_1, sc_value);
        sensor[idx_1].valid_fun = true;
        logg("Got %f | %s\r\n",hr_1, sensor[idx_1].scaled_value.c_str());
        fclose(f);
    }
        
}

void read_percip_12(){
    Watchdog::get_instance().kick();
    logg("\r\n\r\nFrom read_percip_12:\r\n");

    if (!sd_available) {
        logg("SD not available!\r\n");
        return;
    }
    if(!time_set){
        logg("Time is not set!");
        return;
    }
    time_t time_stamp = time(NULL);
    unsigned int cl1 = roundUp(time_stamp - 43200, write_percip_interval);
    char temp[30];
    sprintf(temp, "/fs/percip/%u.txt", cl1);
    logg("Opening file %s...", temp);
    FILE *f = fopen(temp, "r");
    float hr_12;
    if (!f) {
        logg("Failed\r\n");
    }
    else{
        fscanf(f, "%f", &hr_12);
        int idx=-1, idx_1=-1;
        for(int i = 0;i<SENSOR_COUNT;i++){
            if(sensor[i].name.compare("ra_12") == 0){
                idx_1 = i;
            }
            if(sensor[i].name.compare("ra") == 0){
                idx = i;
            }
        }
        char temp[10];
        double sc_value;
        if(string_to_double(sensor[idx].scaled_value, &sc_value) != 0){
            return;
        }

        sprintf(temp, "%.2f", sc_value  - hr_12);
        sensor[idx_1].scaled_value = string(temp);
        sensor[idx_1].valid_fun = true;
        logg("Got %f | %s\r\n",hr_12, sensor[idx_1].scaled_value.c_str());
        fclose(f);
    }
        
}

int check_version(){
    int retries = 3, result, version_size;
	while(retries > 0){
		if(check_sim800() != 0){
            show_gsm_state(false);
            logg("Could not register SIM800 on network");
            time_set = false;
            status_update("Failed");
            return -1;
        }
        Watchdog::get_instance().kick();
        show_gsm_state(true);
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "Contype", "GPRS");
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "APN", "www");
        sim800.AT_HTTPINIT(EXEC_CMND);
        sim800.AT_HTTPPARA(WRITE_CMND, "CID", "1");
        // sim800.AT_HTTPPARA(WRITE_CMND, "USERDATA", "\"Cache-Control\": \"no-cache\"");
        sim800.AT_HTTPPARA(WRITE_CMND, "URL", "fw.abfascada.ir/ahv_rtu/version.php");
        wait_us(500000);
		result = sim800.AT_SAPBR(WRITE_CMND, 1, 1);
        if(result != 0){
            retries--;
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        wait_us(100000);
        
        result = sim800.AT_HTTPACTION(WRITE_CMND, 0, 20000);
        if(result != 0){
            retries--;
            sim800.disable();
            show_gsm_state(false);
            continue;
        }

        string temp_data(sim800.data);
        int idx = temp_data.find(",");
        temp_data = temp_data.substr(idx+1);

        idx = temp_data.find(",");
        string parse = temp_data.substr(0, idx).c_str();
        int result_code;
        string_to_int(parse, &result_code);

        printf("result = %d\r\n", result_code);
        if(result_code != 200){
            sim800.AT_HTTPTERM(EXEC_CMND);
            sim800.AT_SAPBR(WRITE_CMND, 0, 1);
            return -1;
        }
        result = sim800.AT_HTTPREAD(EXEC_CMND, &version_size, data_buffer);
        if(result != 0){
            retries--;
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        break;
    }
    sim800.AT_HTTPTERM(EXEC_CMND);
    sim800.AT_SAPBR(WRITE_CMND, 0, 1);
    data_buffer[version_size] = '\0';
    if(string_to_double(string(data_buffer), &temp_firmware_version) == 0){
         printf("new version =%f\r\n", temp_firmware_version);
        if(temp_firmware_version > firmware_version){
            return 0;
        }
        else{
            return -1;
        }
    }
    else{
        return -1;
    }
    return -1;
}

int download_update_file(){
	int retries = 3, result;
	while(retries > 0){
		if(check_sim800() != 0){
			return -2;
            show_gsm_state(true);
		}
       Watchdog::get_instance().kick();
        show_gsm_state(true);
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "Contype", "GPRS");
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "APN", "www");
        sim800.AT_HTTPINIT(EXEC_CMND);
        sim800.AT_HTTPPARA(WRITE_CMND, "CID", "1");
        // sim800.AT_HTTPPARA(WRITE_CMND, "USERDATA", "\"Cache-Control\": \"no-cache\"");
        char url[100];
        sprintf(url, "fw.abfascada.ir/ahv_rtu/update_%.1f.bin", temp_firmware_version);
        sim800.AT_HTTPPARA(WRITE_CMND, "URL", string(url));
        wait_us(500000);
		result = sim800.AT_SAPBR(WRITE_CMND, 1, 1);
        if(result != 0){
            retries--;
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        status_update("Fetching data...");
        wait_us(100000);
        result = sim800.AT_HTTPACTION(WRITE_CMND, 0, 120000);
        if(result != 0){
            retries--;
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        printf("action data: %s\r\n", sim800.data);
        string temp_data(sim800.data);
        int idx = temp_data.find(",");
        temp_data = temp_data.substr(idx+1);

        idx = temp_data.find(",");
        string parse = temp_data.substr(0, idx).c_str();
        int result_code;
        string_to_int(parse, &result_code);
        temp_data = temp_data.substr(idx+1);

        idx = temp_data.find("\r\n");
        parse = temp_data.substr(0, idx).c_str();
        int file_size;
        string_to_int(parse, &file_size);
    
        printf("result = %d, file size = %d\r\n", result_code, file_size);
        if(result_code != 200){
            retries--;
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        logg("Creating file update.bin...");
        FILE *file = fopen("/fs/update.bin", "w+b");
        if (!file) {
            logg("Failed\r\n");
            sim800.AT_HTTPTERM(EXEC_CMND);
            sim800.AT_SAPBR(WRITE_CMND, 0, 1);
            return -2;
        }
        else {
            logg("Done\r\n");
            created_file = true;
        }
        int data_size = 1500;
        int blockCnt = (int) file_size / data_size;
        for(int i = 0;i < blockCnt;i++){
            char temp[30];
            sprintf(temp, "Downloading %02d %%", (i * 100 / blockCnt));
            logg("%s\r\n", temp);
            status_update(string(temp));
            Watchdog::get_instance().kick();
            data_size = 1500;
            result = sim800.AT_HTTPREAD(WRITE_CMND, &data_size, download_buffer, i * data_size, data_size);
            
            printf("data size = %d\r\n", data_size);
            logg("Writing %d bytes to SD card...", data_size);
            if(fwrite(download_buffer, sizeof(char), data_size, file) == data_size){
                logg("OK\r\n");
            }
            else{
                logg("Failed\r\n");
                if(file){
                    fclose(file);
                }
                sim800.AT_HTTPTERM(EXEC_CMND);
                sim800.AT_SAPBR(WRITE_CMND, 0, 1);;
                return -4;
            }
        }
        int left = file_size % data_size;
        Watchdog::get_instance().kick();
        result = sim800.AT_HTTPREAD(WRITE_CMND, &data_size, download_buffer, file_size - left, left);
        printf("data size = %d\r\n", data_size);
        logg("Writing %d bytes to SD card...", left);
        if(fwrite(download_buffer, sizeof(char), left, file) == left){
            logg("OK\r\n");
        }
        else{
            logg("Failed\r\n");
            if(file){
                fclose(file);
            }
            sim800.AT_HTTPTERM(EXEC_CMND);
            sim800.AT_SAPBR(WRITE_CMND, 0, 1);;
            return -4;
        }
        status_update("Downloading 100%%");
        logg("Downloading 100%%");
        wait_us(500000);
        fclose(file);
        break;
	}
    if(retries <= 0){
        return -1;
    }
	return 0;
}

void check_for_update(){
    Watchdog::get_instance().kick();
    logg("\r\n\r\nFrom check_for_update:\r\n");
    status_update("Checking update...");
    if(!sd_available){
        logg("SD is not available\r\n");
        status_update("No SD card!");
        return;
    }
    int result = check_version();
    Watchdog::get_instance().kick();
    if(result != 0){
        logg("No updates\r\n");
        status_update("No update.");
        ev_queue.call_in(update_check_interval, check_for_update);
        return;
    }
    Watchdog::get_instance().kick();
    status_update("update Found.");
    int rslt = download_update_file();
    if(rslt == 0){
        status_update("Restarting!");
        wait_us(1000000);
        logg("Restarting to apply update.\r\n");
        NVIC_SystemReset();
    }
    else{
        status_update("Download Failed!");
        wait_us(1000000);
        status_update("Retry in 10 Minutes");
        wait_us(1000000);
        ev_queue.call_in(update_retry_interval, check_for_update);
        if(created_file){
            remove("/fs/update.bin");
            created_file = false;
        }
    }
}

void check_power_state(){
    on_battery = !power;
    char temp[10];
    double battery_voltage = batt_volt.read();

    sprintf(temp, "%f", battery_voltage * 12.27);
    sensor[21].raw_value = string(temp);
    sensor[21].valid_raw = true;
    apply_function(21, battery_voltage * 12.27);

    if(battery_voltage > 0.71){
        batt_state = 3;
    }
    else if(battery_voltage > 0.7){
        if(batt_state == -1){
            batt_state = 3;
        }
        batt_state = batt_state;
    }
    else if(battery_voltage > 0.61){
        batt_state = 2;
    }
    else if(battery_voltage > 0.6){
        if(batt_state == -1){
            batt_state = 2;
        }
        batt_state = batt_state;
    }
    else{
        batt_state = 1;
    }
}

void get_rs485_data(){
    logg("\r\n\r\nFrom get_rs485_data:\r\n");
    Watchdog::get_instance().kick();
    int idx = -1;
    for(int i = 0;i < SENSOR_COUNT;i++){
        if(sensor[i].name.compare("rs") == 0){
            idx = i;
            break;
        }
    }
    rs_control = 1;
    rs_menu.printf("/?*\r\n");
    rs_control = 0;
    logg("/?* command sent\r\n");
    logg("Waiting for response\r\n");
    int timeout = 10;
    Watchdog::get_instance().kick();
    while(!rs_data_available){
        logg(".\r\n");
        wait_us(500000);
        timeout--;
        if(timeout <= 0){
            logg("Timeout\r\n");
            return;
        }
    }
    logg("\r\n");
    // logg("rs485 data:\r\n", rs_receive_buffer);
    // for(int i = 1;i < 10;i++){
    //     logg("%02X", rs_receive_buffer[i]);
    // }
    int p_c = rs_receive_buffer[1] + (rs_receive_buffer[2] << 8);
    logg("peizo current = %d\r\n", p_c);
    char temp[10];
    sprintf(temp, "%d", p_c);
    sensor[idx].raw_value = string(temp);
    sensor[idx].valid_raw = true;
    apply_function(idx, (double)p_c);
}

int check_data(string sensor_name, double value){
    if(sensor_name.compare("pt") == 0){
        if(value > 980.0 || value < -100){
            return 2;
        }
        return 0;
    }
    return 0;
}

void parse_arduino_data(){
    Watchdog::get_instance().kick();
    logg("\r\n\r\nFrom parse_arduino_data:\r\n");
    status_update("Updating sensor data");
    last_data_timestamp = time(NULL);
    TiXmlDocument doc;
    doc.Parse(arduino_buffer);
    logg("Received data: ");
    TiXmlPrinter printer;
    doc.Accept(&printer);
    logg("%s\r\n\r\n", printer.CStr());
    for (TiXmlNode* child = doc.RootElement()->FirstChild(); child; child = child->NextSibling() ) {
        Watchdog::get_instance().kick();
        if (child->Type()==TiXmlNode::TINYXML_ELEMENT) {
            string tag = child->Value();
            string text = string(child->ToElement()->GetText());
            for(int i = 0;i < SENSOR_COUNT;i++){
                if(tag.compare(sensor[i].name) == 0){
                    sensor[i].raw_value = text;
                    double value;
                    if(string_to_double(sensor[i].raw_value, &value) == 0){
                        sensor[i].valid_raw = true;
                        sensor[i].warning = check_data(sensor[i].name, value);
                        apply_function(i, value);
                    }
                    else{
                        sensor[i].warning = 1;
                    }
                }
            }
        }
    }
    Watchdog::get_instance().kick();
    float value_1 = 0, value_2 = 0;
    for(int i = 0;i < 50;i++){
        value_1 += rtu_adc_1;
        value_2 += rtu_adc_2;
        wait_us(100000);
    }
    value_1 = (float)(value_1 / 50 * 12.269);
    value_2 = (float)(value_2 / 50 * 12.269);
    
    sensor[ARDUINO_SENSOR_COUNT].raw_value = to_string(value_1);
    sensor[ARDUINO_SENSOR_COUNT].valid_raw = true;
    apply_function(ARDUINO_SENSOR_COUNT, (double)value_1);

    sensor[ARDUINO_SENSOR_COUNT+1].raw_value = to_string(value_2);
    sensor[ARDUINO_SENSOR_COUNT+1].valid_raw = true;
    apply_function(ARDUINO_SENSOR_COUNT+1, (double)value_2);
    Watchdog::get_instance().kick();
    get_rs485_data();

    write_percip();
    read_percip_1();
    read_percip_12();
    print_sensors();
    if(screen){
        show_sensor_data();
    }
    is_arduino_parser_free = true;
}

void arduino_rx(){
    char c = arduino.getc();
    switch(arduino_rx_state){
        case 0:{
            if(c == '>'){
                arduino_cli_ready = true;
                break;
            }
            if(c != '<'){
                break;
            }
            arduino_rx_state = 1;
            arduino_buffer[arduino_buffer_index] = c;
            arduino_buffer_index++;
            arduino_tag_start_index = arduino_buffer_index;
            break;
        }
        case 1:{
            if(c == '<'){
                arduino_rx_state = 0;
                arduino_buffer_index = 0;
                break;
            }
            arduino_buffer[arduino_buffer_index] = c;
            arduino_buffer_index++;
            if(c == '>'){
                if(char_compare(arduino_buffer, (char*)"/data", arduino_tag_start_index, arduino_buffer_index - 2)){
                    arduino_buffer[arduino_buffer_index] = '\0';
                    arduino_rx_state = 0;
                    arduino_buffer_index = 0;
                    if(is_arduino_parser_free){
                        is_arduino_parser_free = false;
                        ev_queue.call(parse_arduino_data);
                    }
                    break;
                }
                if(char_compare(arduino_buffer, (char*)"/cmnd", arduino_tag_start_index, arduino_buffer_index - 2)){
                    arduino_buffer[arduino_buffer_index] = '\0';
                    arduino_rx_state = 0;
                    arduino_buffer_index = 0;
                    arduino_cli_result_ready = true;
                    break;
                }
                arduino_rx_state = 9;
            }
            break;
        }
        case 9:{
            arduino_buffer[arduino_buffer_index] = c;
            arduino_buffer_index++;
            if(c == '<'){
                arduino_rx_state = 1;
                arduino_tag_start_index = arduino_buffer_index;
            }
            break;
        }
    }
}

void log_data_to_sd(){
    Watchdog::get_instance().kick();
    logg("\r\n\r\nfrom log_data_to_sd:\r\n");
    if(!sd_available){
        logg("SD not available\r\n");
        return;
    }

    if(last_data_timestamp == 0 || last_log_timestamp == last_data_timestamp){
        logg("Data not available\r\n");
        ev_queue.call_in(sd_log_interval, log_data_to_sd);
        return;
    }
    status_update("Logging data on SD");

    time_t seconds = time(NULL);
    strftime(day_st, 32, "%d", localtime(&seconds));
    strftime(month_st, 32, "%m", localtime(&seconds));
    strftime(year_st, 32, "%y", localtime(&seconds));
    char file_raw[30];
    char file_scaled[35];
    sprintf(file_raw, "/fs/data/raw/%s-%s-%s.csv", year_st, month_st, day_st);
    sprintf(file_scaled, "/fs/data/scaled/%s-%s-%s.csv", year_st, month_st, day_st);
    logg("Cheking for file %s...", file_raw);
    FILE *raw_data_file = fopen(file_raw, "r");
    if (!raw_data_file) {
    	logg("file not created yet.\r\n");
        logg("Creating file %s with header...", file_raw);
        FILE *temp_file = fopen(file_raw, "w");
        if(!temp_file){
            logg("failed\r\n");
            ev_queue.call_in(sd_log_interval, log_data_to_sd);
            return;
        }
        else{
            char temp[50];
            sprintf(temp, "timestamp");
            for(int i = 0;i < ARDUINO_SENSOR_COUNT;i++){
                sprintf(temp, "%s,%s", temp, arduino_sensors[i].c_str());
            }
            for(int i = 0;i < RTU_SENSOR_COUNT;i++){
                sprintf(temp, "%s,%s", temp, rtu_sensors[i].c_str());
            }
            for(int i=0; i<CALCULATED_DATA_COUNT;i++){
                sprintf(temp, "%s,%s", temp, calculated_data[i].c_str());
            }
            fprintf(temp_file, "%s\r\n", temp);
            fclose(temp_file);
            logg("Done\r\n");
        }
    }
    else {
    	logg("Done\r\n");
        fclose(raw_data_file);
    }
    
    Watchdog::get_instance().kick();
    logg("Cheking for file %s...", file_scaled);
    FILE *scaled_data_file = fopen(file_scaled, "r");
    if (!scaled_data_file) {
    	logg("file not created yet.\r\n");
        logg("Creating file %s with header...", file_scaled);
        FILE *temp_file = fopen(file_scaled, "w");
        if(!temp_file){
            logg("failed\r\n");
            ev_queue.call_in(sd_log_interval, log_data_to_sd);
            return;
        }
        else{
            logg("Done\r\n");
            char temp[50];
            sprintf(temp, "timestamp");
            for(int i = 0;i < ARDUINO_SENSOR_COUNT;i++){
                sprintf(temp, "%s,%s", temp, arduino_sensors[i].c_str());
            }
            for(int i = 0;i < RTU_SENSOR_COUNT;i++){
                sprintf(temp, "%s,%s", temp, rtu_sensors[i].c_str());
            }
            for(int i=0; i<CALCULATED_DATA_COUNT;i++){
                sprintf(temp, "%s,%s", temp, calculated_data[i].c_str());
            }
            fprintf(temp_file, "%s\r\n", temp);
            fclose(temp_file);
        }
    }
    else {
    	logg("Done\r\n");
        fclose(scaled_data_file);
    }
    Watchdog::get_instance().kick();
    logg("Writting raw data...");
    raw_data_file = fopen(file_raw, "a");
    if(!raw_data_file){
        logg("failed\r\n");
        ev_queue.call_in(sd_log_interval, log_data_to_sd);
    }
    else{
        char temp[300];
        sprintf(temp, "%u", last_data_timestamp);
        for(int i= 0;i < SENSOR_COUNT;i++){
            if(sensor[i].valid_raw){
                sprintf(temp, "%s,%s", temp, sensor[i].raw_value.c_str());
            }
            else{
                sprintf(temp, "%s,", temp);
            }
        }
        fprintf(raw_data_file, "%s\r\n", temp);
        fclose(raw_data_file);
        logg("Done\r\n");
    }
    Watchdog::get_instance().kick();
    logg("Writting scaled data...");
    scaled_data_file = fopen(file_scaled, "a");
    if(!scaled_data_file){
        logg("failed\r\n");
        ev_queue.call_in(sd_log_interval, log_data_to_sd);
    }
    else{
        char temp[300];
        sprintf(temp, "%u", last_data_timestamp);
        for(int i= 0;i < SENSOR_COUNT;i++){
            if(sensor[i].valid_fun){
                sprintf(temp, "%s,%s", temp, sensor[i].scaled_value.c_str());
            }
            else{
                sprintf(temp, "%s,", temp);
            }
        }
        fprintf(scaled_data_file, "%s\r\n", temp);
        fclose(scaled_data_file);
        logg("Done\r\n");
    }
    last_log_timestamp = last_data_timestamp;
    status_update("Done.");
    ev_queue.call_in(sd_log_interval, log_data_to_sd);
}

void send_data_sms(){
    logg("\r\n\r\nFrom send_data_sms:\r\n");
    if(last_data_timestamp == 0){
        logg("Data not available");
        ev_queue.call_in(data_sms_interval, send_data_sms);
        return;
    }
    bool phone_1 = false, phone_2 = false;
    if(phone_no_1.length() > 0){
        phone_1 = true;
    }
    if(phone_no_2.length() > 0){
        phone_2 = true;
    }
    if(!phone_1 && !phone_2){
        logg("No phone numbers entered!");
        ev_queue.call_in(data_sms_interval, send_data_sms);
        return;
    }
    status_update("Sending data sms...");
    if(check_sim800() != 0){
        logg("Could not register SIM800 on network");
        status_update("Failed");
        show_gsm_state(false);
        ev_queue.call_in(data_sms_interval, send_data_sms);
        return;
    }
    Watchdog::get_instance().kick();
    show_gsm_state(true);

    generate_sms_data();
    
    sim800.AT_CMGF(WRITE_CMND, 1);
    sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
    if(phone_1){
        sim800.AT_CMGS(WRITE_CMND, phone_no_1, sms_buffer);
    }
    if(phone_2){
        sim800.AT_CMGS(WRITE_CMND, phone_no_2, sms_buffer);
    }
    status_update("Done.");
    ev_queue.call_in(data_sms_interval, send_data_sms);
}

void post_data(){
    Watchdog::get_instance().kick();
    logg("\r\n\r\nFrom post_data:\r\n");
    if(on_battery){
        logg("on battery, not sending\r\n");
        ev_queue.call_in(data_post_interval, post_data);
        return;
    }
    if(last_data_timestamp == 0){
        logg("Data not available\r\n");
        ev_queue.call_in(data_post_interval, post_data);
        return;
    }
    if(!time_set){
        get_time();
    }
    status_update("Sending data to server");
    wait_us(100000);
    
    int sz = 0;
    strftime(temp_buffer, 25, "{\"time\":\"%H:%M:%S\"", localtime(&last_data_timestamp));
    sprintf(temp_buffer, "%s,\"device_id\":\"%s\"", temp_buffer, device_id.c_str());
    sprintf(temp_buffer, "%s,\"firmware_version\":\"%.1f\"", temp_buffer, firmware_version);
    if(valid_location){
        sprintf(temp_buffer, "%s,\"location\":{\"lat\":\"%f\", \"lon\":\"%f\"}", temp_buffer, lat, lon);
    }
    else{
        sprintf(temp_buffer, "%s,\"location\":{\"lat\":\"\", \"lon\":\"\"}", temp_buffer);
    }
    time_t now = time(NULL);
    if(now >= last_data_timestamp + 3600){
        sprintf(temp_buffer, "%s,\"interface_warning\":\"1\"", temp_buffer);
    }
    else{
        sprintf(temp_buffer, "%s,\"interface_warning\":\"0\"", temp_buffer);
    }
    for(int i = 0;i < SENSOR_COUNT;i++){
        sz = sprintf(temp_buffer, "%s,\"%s\":{\"raw\":\"%s\", \"scaled\":\"%s\", \"warning\":\"%d\"}", temp_buffer, sensor[i].name.c_str(), sensor[i].valid_raw ? sensor[i].raw_value.c_str() : "", sensor[i].valid_fun ? sensor[i].scaled_value.c_str() : "", sensor[i].warning);
    }
    temp_buffer[sz++] = '}';
    temp_buffer[sz] = '\0';
    logg("unencrypted data = %s\r\n", temp_buffer);
    const int input_size = 16 * (((int)sz/16) + 1);
    
    for(int i = 0;i < sz;i++){
        input[i] = temp_buffer[i];
    }
    for(int i = sz;i < input_size;i++){
        input[i] = (unsigned char)input_size - sz;
    }
    
    status_update("Connecting to server");
    int retries = 3;
    while(retries > 0){
        Watchdog::get_instance().kick();
        if(check_sim800() != 0){
            logg("Could not register SIM800 on network");
            show_gsm_state(false);
            status_update("Failed");
            ev_queue.call_in(data_post_interval, post_data);
            return;
        }
        show_gsm_state(true);
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "Contype", "GPRS");
        sim800.AT_SAPBR(WRITE_CMND, 3, 1, "APN", "www");
        sim800.AT_HTTPINIT(EXEC_CMND);
        sim800.AT_HTTPPARA(WRITE_CMND, "CID", "1");
        sim800.AT_HTTPPARA(WRITE_CMND, "URL", gprs_url);
        sim800.AT_HTTPPARA(WRITE_CMND, "CONTENT", "application/x-www-form-urlencoded");
        wait_us(100000);
        if(sim800.AT_SAPBR(WRITE_CMND, 1, 1) != 0){
            retries--;
            status_update("Failed! Retrying...");
            sim800.disable();
            show_gsm_state(false);
            continue;
        }

        status_update("Connected");
        generate_random_iv(16, iv);
        sprintf(post_buffer, "data=");
        for(int i = 0;i<16;i++){
            sprintf(post_buffer, "%s%02x", post_buffer, iv[i]);
        }
        mbedtls_aes_setkey_enc(&aes, key, 128);
        mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, input_size, iv, input, output); 
        int l;
        for(int i = 0;i < input_size;i++){
            l = sprintf(post_buffer, "%s%02x", post_buffer, output[i]);
        }
        logg("send: %s\r\n", post_buffer);
        status_update("sending data...");
        if(sim800.AT_HTTPDATA(WRITE_CMND, l, 5000, post_buffer) != 0){
            retries--;
            status_update("Failed! Retrying...");
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        if(sim800.AT_HTTPACTION(WRITE_CMND, 1, 60000) != 0){
            retries--;
            status_update("Failed! Retrying...");
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        int data_len;
        if(sim800.AT_HTTPREAD(EXEC_CMND, &data_len, data_buffer) != 0){
            retries--;
            status_update("Failed! Retrying...");
            sim800.disable();
            show_gsm_state(false);
            continue;
        }
        sim800.AT_HTTPTERM(EXEC_CMND);
        sim800.AT_SAPBR(WRITE_CMND, 0, 1);
        logg("data len = %d\r\n", data_len);
        for(int i = 0;i < data_len; i++){
            logg("%02x", data_buffer[i]);
        }
        break;
    }
    if(retries <= 0){
        status_update("Failed!");
        ev_queue.call_in(data_post_interval, post_data);
        return;
    }
    ev_queue.call_in(data_post_interval, post_data);
    status_update("Sent.");
}

void load_config_from_sd(){
    Watchdog::get_instance().kick();
    logg("\r\n\r\nFrom load_config_from_sd:\r\n");
    status_update("Check config file");
    if(!sd_available){
        logg("SD not available!\r\n");
        status_update("No SD");
        return;
    }
    TiXmlDocument doc;
    FILE* file = fopen("/fs/config.xml", "r");
    if(!file){
        logg("config file not available");
        status_update("file not found");
        return;
    }
    status_update("Loading config file");
    doc.LoadFile(file);
    parse_config_xml(doc);
    doc.Clear();
    fclose(file);
    status_update("Loaded config");
}

void initialize_data(){
    logg("\r\n\r\nFrom initialize_sata:");

    for(int i = 0;i < ARDUINO_SENSOR_COUNT;i++){
        sensor[i].name = arduino_sensors[i];
        sensor[i].display_name = arduino_sensors_name[i];
    }
    for(int i = ARDUINO_SENSOR_COUNT;i < ARDUINO_SENSOR_COUNT+RTU_SENSOR_COUNT;i++){
        sensor[i].name = rtu_sensors[i-ARDUINO_SENSOR_COUNT];
        sensor[i].display_name = rtu_sensros_name[i-ARDUINO_SENSOR_COUNT];
    }
    for(int i = ARDUINO_SENSOR_COUNT+RTU_SENSOR_COUNT;i < SENSOR_COUNT;i++){
        sensor[i].name = calculated_data[i-(ARDUINO_SENSOR_COUNT+RTU_SENSOR_COUNT)];
        sensor[i].display_name = calculated_data_name[i-(ARDUINO_SENSOR_COUNT+RTU_SENSOR_COUNT)];
    }
    load_config_from_sd();
}

void check_for_sms(){
    logg("\r\n\r\nCheck for sms\r\n");
    status_update("Check sms command");
    if(check_sim800() != 0){
        logg("Could not register SIM800 on network");
        show_gsm_state(false);
        ev_queue.call_in(300000, check_for_sms);
        return;
    }
    show_gsm_state(true);
    sim800.AT_CMGF(WRITE_CMND, 1);
    for(int i=1;i<16;i++){
        Watchdog::get_instance().kick();
        if(sim800.AT_CMGR(WRITE_CMND, i) != 0){
            ev_queue.call_in(300000, check_for_sms);
            return;
        }
        string output(sim800.data);
        remove(output.begin(), output.end(), '\r');
        remove(output.begin(), output.end(), '\n');
        for (size_t j = 0; j < 3; j++) {
            int u = output.find("\"");
            output = output.substr(u+1);
        }
        int u = output.find("\"");
        string phone_number = output.substr(0, u);
        for (size_t j = 0; j < 5; j++) {
            int u = output.find("\"");
            output = output.substr(u+1);
        }
        u = output.find("OK");
        output = output.substr(0, u);
        pc.printf("sms: %s, from %s\n", output.c_str(), phone_number.c_str());
        sim800.AT_CMGD(WRITE_CMND, i);
        if(output.find("#stat") == 0){
            generate_sms_data();
            sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
            sim800.AT_CMGS(WRITE_CMND, phone_number, sms_buffer);
        }
        else if(output.find("#gp") == 0){
            ev_queue.call(post_data);
        }
        else if(output.find("#qu") == 0){
            sim800.AT_CSQ(EXEC_CMND);
            sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
            sim800.AT_CMGS(WRITE_CMND, phone_number, sim800.data);
        }
        else if(output.find("#reset") == 0){
            NVIC_SystemReset();
        }
        else if(output.find("#update") == 0){
            ev_queue.call(check_for_update);
        }
        else if(output.find("#balance") == 0){
            sim800.AT_CUSD(WRITE_CMND, 1);
            sim800.AT_CUSD(WRITE_CMND, 1, "*555*4*3*2#");
            sim800.AT_CUSD(WRITE_CMND, 1, "*555*1*2#");
            sim800.AT_CSMP(WRITE_CMND, 17, 167, 0, 0);
            sim800.AT_CMGS(WRITE_CMND, phone_number, sim800.data);
        }
    }
    status_update("Done.");
    ev_queue.call_in(300000, check_for_sms);
}

void check_buttons(){
    if(next_button == 1){
        next_ready = true;
    }
    if(prev_button == 1){
        prev_ready = true;
    }
    
    if(!next_button && next_ready){
        next_ready = false;
        if(lcd_page_number == 5){
            lcd_page_number = 1;
        }
        else{
            lcd_page_number++;
        }
        if(on_battery){
            ev_queue.call(update_lcd);
        }
        ev_queue.call(show_sensor_data);
    }

    if(!prev_button && prev_ready){
        prev_ready = false;
        if(lcd_page_number == 1){
            lcd_page_number = 5;
        }
        else{
            lcd_page_number--;
        }
        if(on_battery){
            ev_queue.call(update_lcd);
        }
        ev_queue.call(show_sensor_data);
    }
    ev_queue.call_in(200, check_buttons);
}

void blink(){
    led_cnt++;
	if(on_battery){
        if(led){
            led = 0;
        }
        else{
            if(led_cnt >= 20){
                led = 1;
                led_cnt=0;
            }
        }
    }
    else{
        if(led_cnt >= 10){
            led = !led;
            led_cnt=0;
        }
    }
    ev_queue.call_in(100, blink);
}