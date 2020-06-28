#include "main.h"

void update_header(){
    if(screen){
        show_date_time();
        show_battery();
    }
    ev_queue.call_in(30000, update_header);
}

void main_loop(){
    time_t now = time(NULL);
    check_power_state();
    Watchdog::get_instance().kick();
    if(on_battery){
        if(!one_time_gps_sent){
            send_gps_sms();
        }
        sim800.disable();
        if(now > last_lcd_op + 10){
            tft.clearScreen();
            screen = false;
        }
    }
    else{
        status_update("Running...");
        one_time_gps_sent = false;
        if(!screen){
            update_lcd();
        }
        if(!time_set){
            get_time();
        }
    }
    ev_queue.call_in(1000, main_loop);
}

int main(){
    watchdog.start();
    printf("\r\nFirmware version = %.1f\r\n", firmware_version);

    Watchdog::get_instance().kick();
    init_lcd();
    check_power_state();
    show_battery();
    show_gsm_state(false);
    draw_footer();
    status_update("Initializing");
    
    Watchdog::get_instance().kick();
    init_SD();
    initialize_data();
    status_update("Initializing.");

    Watchdog::get_instance().kick();
    status_update("Initializing. .");
    serial.attach(callback(&ser, &SerialHandler::rx));
    status_update("Initializing. . . .");
    gps.attach(gps_rx);
    status_update("Initializing. . . . .");
    rs_menu.attach(rs_menu_rx);
    status_update("Initializing. . . . . .");
    arduino.printf("exit");
    if(arduino.readable()){
        while(arduino.readable()){
            arduino.getc();
        }
    }
    arduino.attach(arduino_rx);
    status_update("Initializing. . . . . . .");
    
    
    Watchdog::get_instance().kick();
    get_time();
    show_date_time();
    show_sensor_data();
    status_update("Running...");

	ev_queue.call_in(100, blink);
    ev_queue.call_in(200, check_buttons);
    ev_queue.call_in(1000, main_loop);

    // ev_queue.call_in(60000, check_for_update);
    ev_queue.call(check_for_update);
    ev_queue.call_in(sd_log_interval, log_data_to_sd);
    ev_queue.call_in(data_post_interval, post_data);
    ev_queue.call_in(data_sms_interval, send_data_sms);
    ev_queue.call_in(30000, update_header);
    ev_queue.call_in(180000, check_for_sms);

	ev_queue.dispatch_forever();
}