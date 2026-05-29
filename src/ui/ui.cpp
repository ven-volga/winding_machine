// ui.cpp
//
// Макет екрану (4×20):
//
//   Col: 01234567890123456789
//   Row 0: >SPD: 300rpm FWD>>>
//   Row 1:  TURNS: 0000
//   Row 2: >POS:   0.00mm
//   Row 3: >DIA:0.20  >W: 30.0
//
// Параметри курсора:
//   SPEED     — row 0, col 0
//   DIRECTION — row 0, col 11
//   POS       — row 2, col 0  (нове!)
//   DIAMETER  — row 3, col 0
//   WIDTH     — row 3, col 11
//
// Логіка POS в EDIT режимі:
//   Крутиш              → встановлюєш цільове значення (мм)
//   Утримання 5 сек     → Set Home (carriagePos = 0.0)
//   Короткий натиск     → скасувати
//   Довгий натиск       → зберегти і каретка фізично їде туди

#include "ui.h"
#include "encoder.h"
#include "lcd.h"
#include "shared_state.h"
#include "config.h"
#include "motion.h"

#include <cstdio>
#include <cmath>

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_timer.h"
}

// ─────────────────────────────────────────
// Режими і параметри
// ─────────────────────────────────────────

typedef enum { UI_VIEW, UI_EDIT } UiMode;

typedef enum
{
    PARAM_SPEED     = 0,
    PARAM_DIRECTION = 1,
    PARAM_POS       = 2,  // позиція каретки
    PARAM_DIAMETER  = 3,
    PARAM_WIDTH     = 4,
    PARAM_COUNT     = 5,
} Param;

// ─────────────────────────────────────────
// Стан UI
// ─────────────────────────────────────────

static UiMode  uiMode        = UI_VIEW;
static uint8_t selectedParam = PARAM_SPEED;
static bool    needFullRedraw = true;

// Буфер редагування
static uint32_t editSpeed = 0;
static float    editDia   = 0.0f;
static float    editWidth = 0.0f;
static bool     editDir   = true;
static float    editPos   = 0.0f;  // цільова позиція каретки

// Кеш відображених значень
static uint32_t lastTurns = 0xFFFFFFFF;
static float    lastPos   = -9999.0f;
static uint32_t lastSpeed = 0xFFFFFFFF;
static bool     lastDir   = true;

// Set Home — час початку утримання кнопки в EDIT POS
static uint32_t setHomeStartMs = 0;
static bool     setHomeArmed   = false;

// Позиції курсорів {col, row}
static const uint8_t CURSOR_POS[PARAM_COUNT][2] =
{
    {0,  0},   // SPEED
    {11, 0},   // DIRECTION
    {0,  2},   // POS
    {0,  3},   // DIAMETER
    {11, 3},   // WIDTH
};

// ─────────────────────────────────────────
// Таймер енкодера (1мс)
// ─────────────────────────────────────────

static esp_timer_handle_t encoderTimer;

static void IRAM_ATTR encoder_timer_cb(void* arg)
{
    encoder_update();

    if (encDelta == 0) return;

    shared_lock();
    bool running = shared.running;
    shared_unlock();

    if (!running) return;

    // Під час намотки — крутіння міняє швидкість
    shared_lock();
    uint32_t spd = shared.spindleSpeed;
    if (encDelta > 0 && spd < SPINDLE_SPEED_MAX)
        spd += SPINDLE_SPEED_STEP;
    if (encDelta < 0 && spd > SPINDLE_SPEED_MIN)
        spd -= SPINDLE_SPEED_STEP;
    shared.spindleSpeed = spd;
    shared_unlock();

    encDelta = 0;
}

// ─────────────────────────────────────────
// Функції малювання
// ─────────────────────────────────────────

static void draw_speed(uint32_t spd)
{
    char buf[11];
    snprintf(buf, sizeof(buf), "SPD:%3lurpm", (unsigned long)spd);
    lcd_set_cursor(1, 0);
    lcd_print(buf);
    lastSpeed = spd;
}

static void draw_direction(bool fwd)
{
    lcd_set_cursor(12, 0);
    lcd_print(fwd ? "FWD>>>" : "REV<<<");
    lastDir = fwd;
}

static void draw_turns(uint32_t t)
{
    char buf[21];
    snprintf(buf, sizeof(buf), " TURNS: %04lu       ", (unsigned long)t);
    buf[20] = '\0';
    lcd_set_cursor(0, 1);
    lcd_print(buf);
    lastTurns = t;
}

// Row 2: позиція каретки з курсором
static void draw_position(float pos)
{
    char buf[21];
    snprintf(buf, sizeof(buf), " POS:  %6.2fmm     ", pos);
    buf[20] = '\0';
    lcd_set_cursor(0, 2);
    lcd_print(buf);
    lastPos = pos;
}

static void draw_diameter(float dia)
{
    char buf[10];
    snprintf(buf, sizeof(buf), "DIA:%.2f ", dia);
    lcd_set_cursor(1, 3);
    lcd_print(buf);
}

static void draw_width(float w)
{
    char buf[9];
    snprintf(buf, sizeof(buf), "W:%5.1f", w);
    lcd_set_cursor(12, 3);
    lcd_print(buf);
}

static void draw_cursor(uint8_t prev)
{
    lcd_set_cursor(CURSOR_POS[prev][0], CURSOR_POS[prev][1]);
    lcd_print(" ");
    lcd_set_cursor(CURSOR_POS[selectedParam][0], CURSOR_POS[selectedParam][1]);
    lcd_print(uiMode == UI_EDIT ? "*" : ">");
}

static void draw_full_screen(const SharedState& s)
{
    lcd_clear();
    lastTurns = 0xFFFFFFFF;
    lastPos   = -9999.0f;
    lastSpeed = 0xFFFFFFFF;
    lastDir   = !s.dirForward;

    draw_speed(s.spindleSpeed);
    draw_direction(s.dirForward);
    draw_turns(s.turns);
    draw_position(s.carriagePos);
    draw_diameter(s.wireDiameter);
    draw_width(s.windingWidth);

    lcd_set_cursor(CURSOR_POS[selectedParam][0], CURSOR_POS[selectedParam][1]);
    lcd_print(">");
}

// ─────────────────────────────────────────
// Публічні функції
// ─────────────────────────────────────────

void ui_init()
{
    lcd_init();
    encoder_init();

    esp_timer_create_args_t timerArgs = {
        .callback              = encoder_timer_cb,
        .arg                   = nullptr,
        .dispatch_method       = ESP_TIMER_TASK,
        .name                  = "enc_timer",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&timerArgs, &encoderTimer);
    esp_timer_start_periodic(encoderTimer, 1000);

    needFullRedraw = true;
}

void ui_update()
{
    SharedState s;
    shared_lock();
    s = shared;
    shared_unlock();

    // ── Повне перемалювання ──────────────────────────────────────
    if (needFullRedraw)
    {
        draw_full_screen(s);
        needFullRedraw = false;
        return;
    }

    // ── VIEW режим ───────────────────────────────────────────────
    if (uiMode == UI_VIEW)
    {
        if (encDelta != 0 && !s.running)
        {
            uint8_t prev = selectedParam;
            if (encDelta > 0)
                selectedParam = (selectedParam + 1) % PARAM_COUNT;
            else
                selectedParam = (selectedParam + PARAM_COUNT - 1) % PARAM_COUNT;
            draw_cursor(prev);
            encDelta = 0;
        }

        if (evLong)
        {
            editSpeed = s.spindleSpeed;
            editDia   = s.wireDiameter;
            editWidth = s.windingWidth;
            editDir   = s.dirForward;
            editPos   = s.carriagePos;
            setHomeArmed = false;
            uiMode = UI_EDIT;
            lcd_set_cursor(CURSOR_POS[selectedParam][0], CURSOR_POS[selectedParam][1]);
            lcd_print("*");
        }

        if (s.spindleSpeed != lastSpeed) draw_speed(s.spindleSpeed);
        if (s.dirForward   != lastDir)   draw_direction(s.dirForward);
        if (s.turns        != lastTurns) draw_turns(s.turns);
        if (fabsf(s.carriagePos - lastPos) > 0.005f)
            draw_position(s.carriagePos);
    }

    // ── EDIT режим ───────────────────────────────────────────────
    else
    {
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        if (encDelta != 0)
        {
            switch (selectedParam)
            {
                case PARAM_SPEED:
                    if (encDelta > 0 && editSpeed < SPINDLE_SPEED_MAX)
                        editSpeed += SPINDLE_SPEED_STEP;
                    if (encDelta < 0 && editSpeed > SPINDLE_SPEED_MIN)
                        editSpeed -= SPINDLE_SPEED_STEP;
                    draw_speed(editSpeed);
                    break;

                case PARAM_DIRECTION:
                    editDir = !editDir;
                    draw_direction(editDir);
                    break;

                case PARAM_POS:
                    // Крутіння задає цільову позицію
                    if (encDelta > 0 && editPos < CARRIAGE_POS_MAX)
                        editPos += CARRIAGE_POS_STEP;
                    if (encDelta < 0 && editPos > CARRIAGE_POS_MIN)
                        editPos -= CARRIAGE_POS_STEP;
                    draw_position(editPos);
                    // Скидаємо таймер Set Home якщо крутять
                    setHomeArmed = false;
                    break;

                case PARAM_DIAMETER:
                    if (encDelta > 0 && editDia < WIRE_DIAMETER_MAX)
                        editDia += WIRE_DIAMETER_STEP;
                    if (encDelta < 0 && editDia > WIRE_DIAMETER_MIN)
                        editDia -= WIRE_DIAMETER_STEP;
                    draw_diameter(editDia);
                    break;

                case PARAM_WIDTH:
                    if (encDelta > 0 && editWidth < WINDING_WIDTH_MAX)
                        editWidth += WINDING_WIDTH_STEP;
                    if (encDelta < 0 && editWidth > WINDING_WIDTH_MIN)
                        editWidth -= WINDING_WIDTH_STEP;
                    draw_width(editWidth);
                    break;
            }
            encDelta = 0;
        }

        // ── Set Home для POS ─────────────────────────────────────
        // Якщо в EDIT POS і тримаємо кнопку 5 секунд — скидаємо позицію
        if (selectedParam == PARAM_POS)
        {
            if (evLong)
            {
                // Перший довгий натиск в POS — починаємо відлік
                if (!setHomeArmed)
                {
                    setHomeArmed   = true;
                    setHomeStartMs = now;
                    // Показуємо користувачу що відлік пішов
                    lcd_set_cursor(0, 2);
                    lcd_print("*HOME 5sec...       ");
                }
            }

            if (setHomeArmed && (now - setHomeStartMs) >= SET_HOME_HOLD_MS)
            {
                // 5 секунд витримали — Set Home!
                shared_lock();
                shared.carriagePos = 0.0f;
                shared_unlock();

                editPos      = 0.0f;
                setHomeArmed = false;

                // Показуємо підтвердження
                lcd_set_cursor(0, 2);
                lcd_print(" HOME SET!          ");
                vTaskDelay(pdMS_TO_TICKS(1000));

                uiMode = UI_VIEW;
                needFullRedraw = true;
                return;
            }
        }

        // Короткий натиск — скасувати
        if (evClick)
        {
            setHomeArmed = false;
            uiMode = UI_VIEW;
            needFullRedraw = true;
        }

        // Довгий натиск — зберегти (крім POS де він запускає Set Home)
        if (evLong && selectedParam != PARAM_POS)
        {
            shared_lock();
            shared.spindleSpeed = editSpeed;
            shared.wireDiameter = editDia;
            shared.windingWidth = editWidth;
            shared.dirForward   = editDir;
            shared_unlock();

            uiMode = UI_VIEW;
            needFullRedraw = true;
        }

        // Довгий натиск на POS — запустити переміщення каретки
        if (evLong && selectedParam == PARAM_POS && !setHomeArmed)
        {
            motion_move_to(editPos);
            uiMode = UI_VIEW;
            needFullRedraw = true;
        }
    }
}