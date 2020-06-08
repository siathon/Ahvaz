#include "main.h"

void main_loop(){
    status_update_cnt++;
    if(on_battery){
    }
    else{
        if(!time_set){
            get_time();
        }
        time_t now = time(NULL);
        if(now > last_log_time + 300){
            ev_queue.call(log_data_to_sd);
            last_log_time = now;
        }
        if(now > last_data_sms_time + data_sms_interval){
            ev_queue.call(send_data_sms);
            last_data_sms_time = now;
        }
        if(now > last_post_time + data_post_interval){
            ev_queue.call(post_data);
            last_post_time = now;
        }
        if(status_update_cnt >= 10){
            status_update_cnt = 0;
            status_update("Running...");
        }
    }

}

int main(){
    init_lcd();
    arduino.attach(arduino_rx);
    serial.attach(callback(&ser, &SerialHandler::rx));
    gps.attach(gps_rx);
    rs_menu.attach(rs_menu_rx);
    printf("\r\nSystem core clock = %d\r\n", SystemCoreClock);
    init_SD();
    initialize_data();
    get_time();
    show_date_time();
    show_sensor_data();
    printf("\r\nFirmware version = %d\r\n", firmware_version);
    status_update("Running...");
    ev_queue.call(send_gps_sms);
	ev_queue.call_every(500, blink);
    ev_queue.call_every(200, check_buttons);
    ev_queue.call_every(60000, show_date_time);
    ev_queue.call_every(1000, main_loop);
	ev_queue.dispatch_forever();
}
