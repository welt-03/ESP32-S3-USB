
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_freertos_hooks.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lvgl.h"
#include "lvgl_helpers.h"

#define TAG "main"
#define LV_TICK_PERIOD_MS 1
#define LED_GREEN GPIO_NUM_15
#define LED_YELLOW GPIO_NUM_16

#define BUTTON_OK GPIO_NUM_0
#define BUTTON_DW GPIO_NUM_11
#define BUTTON_UP GPIO_NUM_10
#define BUTTON_MENU GPIO_NUM_14

#define HIGH 1
#define LOW 0

#define BUF_SIZE 1024

typedef struct {
    uint8_t num;     // IO号
    uint8_t clicks;  // 点击次数
    uint8_t state;   // 按键状态
    uint8_t level;   // 按键电平
} keypad_t;
static keypad_t keypad;
static esp_timer_handle_t timer_handle = NULL;
static QueueHandle_t gpio_evt_queue = NULL;
static lv_disp_t* disp = NULL;
static lv_indev_t* indev_keypad = NULL;

static void gpio_task_handler(void* arg) {
    uint8_t level;
    uint32_t io_num;
    TickType_t tick = 0;

    while (true) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            // ESP_LOGI(TAG, "GPIO[%" PRIu32 "] intr, val: %d", io_num, gpio_get_level((gpio_num_t)io_num));
            level = gpio_get_level(io_num);
            keypad.num = io_num;
            keypad.level = level;
            if (!level) {
                if (!esp_timer_is_active(timer_handle))
                    esp_timer_start_periodic(timer_handle, 20000);

            } else {
                if (keypad.clicks == 0)
                    keypad.clicks++;
                if (xTaskGetTickCount() - tick < 300)
                    keypad.clicks++;
                if (keypad.clicks > 0)
                    tick = xTaskGetTickCount();
            }
            ESP_LOGI(TAG, "keypad.clicks: %d", keypad.clicks);
            switch (io_num) {
                case BUTTON_OK:
                    gpio_set_level(LED_GREEN, !level);
                    gpio_set_level(LED_YELLOW, !level);
                    break;
                case BUTTON_DW:
                    gpio_set_level(LED_GREEN, !level);
                    break;
                case BUTTON_UP:
                    gpio_set_level(LED_YELLOW, !level);
                    break;
                case BUTTON_MENU:
                    gpio_set_level(LED_GREEN, !level);
                    gpio_set_level(LED_YELLOW, !level);
                    break;
            }
            vTaskDelay(30 / portTICK_PERIOD_MS);
            gpio_intr_enable((gpio_num_t)io_num);
        }
    }
}

static void keypad_cb(void* arg) {
    static uint16_t count = 0;

    count++;
    if (count > 18) {
        switch (keypad.num) {
            case BUTTON_OK:
                if (keypad.clicks == 1)
                    keypad.state = LV_KEY_ENTER;
                else if (keypad.clicks == 2)
                    keypad.state = LV_KEY_ESC;
                break;
            case BUTTON_DW:
                keypad.state = LV_KEY_DOWN;
                break;
            case BUTTON_UP:
                keypad.state = LV_KEY_UP;
                break;
            case BUTTON_MENU:
                if (keypad.clicks == 1)
                    keypad.state = LV_KEY_NEXT;
                else if (keypad.clicks == 2)
                    keypad.state = LV_KEY_PREV;
                break;
        }
        if (gpio_get_level(keypad.num)) {
            esp_timer_stop(timer_handle);
            keypad.clicks = 0;
            count = 0;
        }
    }
}

/**
 * @brief 中断处理函数
 *
 * @param arg
 */
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t io_num = (uint32_t)arg;

    gpio_intr_disable((gpio_num_t)io_num);
    xQueueSendFromISR(gpio_evt_queue, &io_num, NULL);
}
static void lv_tick_task(void* arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        lv_tick_inc(LV_TICK_PERIOD_MS);
        vTaskDelayUntil(&xLastWakeTime, LV_TICK_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

static void lv_keypad_read(lv_indev_drv_t* indev_drv, lv_indev_data_t* data) {
    static uint8_t last_key = 0;

    if (keypad.state) {
        data->state = LV_INDEV_STATE_PR;
        last_key = keypad.state;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
    keypad.state = 0;
    data->key = last_key;
}

static void keypad_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = BIT64(BUTTON_OK) | BIT64(BUTTON_DW) | BIT64(BUTTON_UP) | BIT64(BUTTON_MENU),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = BIT64(LED_GREEN) | BIT64(LED_YELLOW);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_handler, "gpio_task", BUF_SIZE * 2, NULL, 10, NULL);

    gpio_install_isr_service(0);

    esp_timer_create_args_t timer_args = {
        .callback = keypad_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "keypad_cb",
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));

    gpio_isr_handler_add(BUTTON_DW, gpio_isr_handler, (void*)BUTTON_DW);
    gpio_isr_handler_add(BUTTON_UP, gpio_isr_handler, (void*)BUTTON_UP);
    gpio_isr_handler_add(BUTTON_MENU, gpio_isr_handler, (void*)BUTTON_MENU);
    gpio_isr_handler_add(BUTTON_OK, gpio_isr_handler, (void*)BUTTON_OK);
}

static void lvgl_init() {
    lv_init();
    lvgl_driver_init();

    // 初始化显示器
    lv_color_t* lv_buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(lv_buf1 != NULL);
    lv_color_t* lv_buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(lv_buf2 != NULL);

    lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, lv_buf1, lv_buf2, DISP_BUF_SIZE);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);  // 显示驱动初始化默认值
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;
    disp = lv_disp_drv_register(&disp_drv);  // 注册显示驱动
    // 配置输入设备
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.disp = disp;
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = lv_keypad_read;
    indev_keypad = lv_indev_drv_register(&indev_drv);

    xTaskCreate(lv_tick_task, "lv_tick", BUF_SIZE, NULL, 10, NULL);
}

static void lvgl_app() {
    lv_obj_t* scr = lv_disp_get_scr_act(NULL);
    lv_obj_t* label = lv_label_create(scr);
    lv_obj_t* slider1 = lv_slider_create(scr);
    lv_obj_t* slider2 = lv_slider_create(scr);
    lv_obj_t* slider3 = lv_slider_create(scr);
    lv_obj_t* slider4 = lv_slider_create(scr);
    lv_obj_t* spinner = lv_spinner_create(scr, 1000, 40);
    lv_obj_t* bar = lv_switch_create(scr);
    lv_group_t* group = lv_group_create();

    lv_obj_set_x(bar, 10);
    lv_obj_set_y(bar, 25);
    lv_obj_set_style_width(slider1, 100, LV_STATE_DEFAULT);
    lv_obj_set_style_height(slider1, 20, LV_STATE_DEFAULT);
    lv_obj_set_style_width(slider2, 100, LV_STATE_DEFAULT);
    lv_obj_set_style_height(slider2, 20, LV_STATE_DEFAULT);
    lv_obj_set_style_width(slider3, 100, LV_STATE_DEFAULT);
    lv_obj_set_style_height(slider3, 20, LV_STATE_DEFAULT);
    lv_obj_set_style_width(slider4, 100, LV_STATE_DEFAULT);
    lv_obj_set_style_height(slider4, 20, LV_STATE_DEFAULT);
    lv_obj_set_x(slider1, 100);
    lv_obj_set_y(slider1, 50);
    lv_obj_set_x(slider2, 100);
    lv_obj_set_y(slider2, 100);
    lv_obj_set_x(slider3, 100);
    lv_obj_set_y(slider3, 150);
    lv_obj_set_x(slider4, 100);
    lv_obj_set_y(slider4, 200);
    lv_group_add_obj(group, bar);
    lv_group_add_obj(group, slider1);
    lv_group_add_obj(group, slider2);
    lv_group_add_obj(group, slider3);
    lv_group_add_obj(group, slider4);
    lv_group_add_obj(group, spinner);

    lv_indev_set_group(indev_keypad, group);
    lv_obj_set_style_text_color(label, lv_color_hex(0xf50057), LV_STATE_DEFAULT);
    lv_label_set_text(label, "Hello World");
}

void app_main() {
    keypad_init();
    lvgl_init();

    lvgl_app();

    while (1) {
        vTaskDelay(5 / portTICK_PERIOD_MS);
        lv_task_handler();
    }
}
