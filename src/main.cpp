// Copyright (C) 2026, cubic9com All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#include "EPD.h"
#include <SensirionI2CScd4x.h>

// E-Paper Settings
// E-Paper設定

// Size of E-Paper display buffer
// E-Paperディスプレイバッファのサイズ
const int epdBufferSize = 27200;

// Key GPIO pin definitions
// キーのGPIOピン定義

// Exit button
// Exitボタン
const int exitButtonPin = 1;

// Menu button
// Menuボタン
const int menuButtonPin = 2;

// Dial push button
// ダイヤル押下ボタン
const int dialPushPin = 5;

// GPIO pin for E-Paper power control
// E-Paper電源制御用GPIOピン
const int epdPowerPin = 7;

// E-Paper Display Buffer
// E-Paperディスプレイバッファ
uint8_t imageBW[epdBufferSize];

// I2C pin definitions
// I2Cピン定義
const int i2cSdaPin = 38;
const int i2cSclPin = 19;

// CO2 sensor settings
// CO2センサー設定

// Outdoor CO2 reference (ppm)
// 屋外CO2基準値（ppm）
const uint16_t co2ConcentrationInOutside = 420; 

// Deep Sleep duration for power-cycled single shot operation.
// Power-Cycledシングルショット動作のためのDeep Sleep時間。
const int sleepDurationInSec = 1200;

// Sentinel value returned by measureCo2() when measurement fails.
// 測定に失敗した際にmeasureCo2()が返す番兵値。
const uint16_t CO2_INVALID = 0xFFFF;

// FRC (Forced Recalibration) settings
// FRC（Forced Recalibration）設定

// Number of single-shot measurements for equilibration before FRC.
// FRC前の安定化用シングルショット測定回数。
const int frcEquilibrationShots = 5;

// Interval between equilibration single-shot measurements (ms) = 60 s
// 安定化用シングルショット測定の間隔（ミリ秒）= 60秒
const int frcShotIntervalMs = 60000;

// Number of progress dots displayed per shot interval
// 1ショット間隔あたりに表示する進捗ドットの数
const int frcDotsPerShot = 6;

// Interval between dots (ms) = 10 s
// ドット表示の間隔（ミリ秒）= 10秒
const int frcDotIntervalMs = 10000;

SensirionI2CScd4x scd4x;

/**
 * Log a sensor error to Serial if error is non-zero.
 * Returns true if an error occurred, false otherwise.
 *
 * エラーが発生している場合にSerialへログ出力するヘルパー関数。
 * エラーがあれば true、なければ false を返す。
 */
bool logSensorError(uint16_t error, const char *context)
{
    if (!error)
    {
        return false;
    }
    char errorMessage[256];
    Serial.print("Error trying to execute ");
    Serial.print(context);
    Serial.print(": ");
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.println(errorMessage);
    return true;
}

/**
 * Disable Automatic Self-Calibration (ASC).
 * Note: ASC ensures the highest long-term stability of the SCD4x
 *       without the need of manual action steps from the user.
 *       But for power-cycled single shot operation, ASC is not available.
 *
 *       To avoid wearing out the sensor's EEPROM, this function first checks
 *       the current ASC setting. Only if ASC is currently enabled, it disables
 *       it and persists the setting to NVM via persistSettings().
 *       persistSettings() is NOT called if ASC is already disabled.
 *
 * Automatic Self-Calibration（ASC）を無効にする。
 * 注：ASCは、ユーザーが手動で操作することなく、SCD4xの最高の長期安定性を保証するが、
 *     電力サイクルによるシングルショット動作では、ASCは使用できない。
 *
 *     センサーのEEPROMの寿命消耗を避けるため、この関数はまず現在のASC設定を確認する。
 *     ASCが有効な場合のみ、無効化してpersistSettings()でNVMに保存する。
 *     既にASCが無効ならpersistSettings()は呼び出さない。
 */
void disableAsc()
{
    uint16_t error;

    // Check current ASC state before modifying.
    // 変更前に現在のASC状態を確認する。
    uint16_t ascEnabled = 0;
    error = scd4x.getAutomaticSelfCalibration(ascEnabled);
    if (logSensorError(error, "getAutomaticSelfCalibration()"))
    {
        return;
    }

    // Skip if ASC is already disabled to avoid unnecessary EEPROM writes.
    // 不要なEEPROM書き込みを避けるため、既に無効なら何もしない。
    if (ascEnabled == 0)
    {
        Serial.println("ASC is already disabled. Skipping.");
        return;
    }

    // Disable ASC.
    // ASCを無効化する。
    error = scd4x.setAutomaticSelfCalibration(0);
    if (logSensorError(error, "setAutomaticSelfCalibration()"))
    {
        return;
    }

    // Persist the setting to NVM so it survives power cycles (e.g. Deep Sleep).
    // Deep Sleep等の電源断後も設定が保持されるよう、NVMに保存する。
    error = scd4x.persistSettings();
    if (logSensorError(error, "persistSettings()"))
    {
        return;
    }

    Serial.println("ASC disabled and persisted to NVM.");
}

/**
 * Perform Forced recalibration (FRC).
 * Note: FRC immediately restores high accuracy with the assistance of
 *       an external CO2 reference value.
 *
 * Forced recalibration（FRC）を実行する。
 * 注：FRCは、外部CO2基準値を用いて高精度を即座に回復する。
 */
void performFrc()
{
    const int textBufferSize = 40;
    char buffer[textBufferSize];
    uint16_t error;

    // Initialize the E-Paper canvas and clear the previous content.
    // E-Paperのキャンバスを初期化し、以前の表示内容をクリアする。
    Paint_NewImage(imageBW, EPD_W, EPD_H, Rotation, WHITE);
    Paint_Clear(WHITE);
    EPD_FastMode1Init();
    EPD_Display_Clear();

    // Notify the user via Serial and the display that FRC is starting.
    // FRC開始をSerialおよびディスプレイでユーザーに通知する。
    Serial.println("Performing Forced Recalibration (FRC).");
    EPD_ShowString(0, 0, "Performing Forced Recalibration (FRC).", 24, BLACK);
    Serial.print("Equilibrating with ");
    Serial.print(frcEquilibrationShots);
    Serial.println(" single shots at 1-minute intervals.");
    snprintf(buffer, sizeof(buffer), "Equilibrating: %d single shots x 1 min.", frcEquilibrationShots);
    EPD_ShowString(0, 30, buffer, 24, BLACK);
    EPD_Display(imageBW);
    EPD_FastUpdate();
    EPD_Clear_R26A6H();

    // Equilibrate in single-shot mode matching the normal operation mode.
    //
    // 通常運用モードに合わせたシングルショットモードでセンサーを安定化させる。
    for (int i = 1; i <= frcEquilibrationShots; i++)
    {
        // Trigger a single-shot measurement (result is used only for equilibration).
        // 安定化用にシングルショット測定を実行する（結果は安定化目的のみで使用）。
        error = scd4x.measureSingleShot();
        if (logSensorError(error, "measureSingleShot() [FRC equilibration]"))
        {
            return;
        }
        Serial.print("Equilibration shot ");
        Serial.print(i);
        Serial.print("/");
        Serial.print(frcEquilibrationShots);
        Serial.println(" triggered.");
        snprintf(buffer, sizeof(buffer), "Shot %d/%d", i, frcEquilibrationShots);
        EPD_ShowString(0, 30 * i + 30, buffer, 24, BLACK);
        EPD_Display(imageBW);
        EPD_FastUpdate();

        if (i < frcEquilibrationShots)
        {
            // Wait frcShotIntervalMs between shots, showing a progress dot every frcDotIntervalMs ms.
            // ショット間はfrcShotIntervalMs待機し、frcDotIntervalMs msごとに進捗ドットを表示する。
            for (int j = 0; j < frcDotsPerShot; j++)
            {
                delay(frcDotIntervalMs);
                Serial.print(".");
                EPD_ShowString(20 * j + 100, 30 * i + 30, ".", 24, BLACK);
                EPD_Display(imageBW);
                EPD_FastUpdate();
            }
            Serial.println();
        }
    }

    // Wait for the last single-shot measurement to complete (5 s), then stop.
    // 最後のシングルショット測定の完了を待ち（5秒）、その後停止する。
    // measureSingleShot() issues the command and returns immediately; the sensor
    // needs ~5 s to finish. We simply delay here instead of polling getDataReadyFlag(),
    // because we only need the equilibration effect, not the actual reading.
    // measureSingleShot()はコマンドを発行してすぐ戻る。センサーは完了に約5秒かかる。
    // 実際の測定値は不要で安定化効果のみ目的のため、getDataReadyFlag()ポーリングの代わりに
    // 単純にdelayする。
    delay(5000);

    // Perform Forced Recalibration (FRC).
    // FRCを実行する。校正結果はセンサーのEEPROMに自動保存される。
    uint16_t frcCorrection = 0;
    error = scd4x.performForcedRecalibration(co2ConcentrationInOutside, frcCorrection);
    if (logSensorError(error, "performForcedRecalibration()"))
    {
        return;
    }
    Serial.print("FRC correction applied: ");
    Serial.print((int16_t)frcCorrection - 0x8000); // Convert to signed offset in ppm / 符号付きオフセット（ppm）に変換する
    Serial.println(" ppm");
}

/**
 * Measure CO2 concentration, humidity and temperature on demand.
 * Implements the full power-cycled single shot sequence:
 *   1. wake_up   – bring sensor from sleep to idle (20 ms)
 *   2. measure_single_shot (discard) – stabilization shot required after
 *      every power cycle; result is intentionally ignored per the Sensirion
 *      application note "SCD4x Low Power Operation" (5 s)
 *   3. measure_single_shot (use)     – the measurement to display (5 s)
 *
 * CO2濃度、湿度、温度をオンデマンドで測定する。
 * Power-Cycledシングルショットの完全なシーケンスを実装する：
 *   1. wake_up   – センサーをスリープからアイドルへ復帰（20 ms）
 *   2. measureSingleShot（破棄）– 電源サイクル後に必要な安定化用測定。
 *      SensirionアプリノートSCD4x Low Power Operationに従い結果を意図的に捨てる（5 s）
 *   3. measureSingleShot（採用）– 表示に使う本番測定（5 s）
 */
uint16_t measureCo2()
{
    uint16_t error;

    // Trigger a stabilization single-shot measurement and discard the result.
    // Per the Sensirion application note, the first reading after a power cycle
    // (including power_down/wake_up) must be discarded to maximize accuracy.
    // センサー安定化用のシングルショット測定を実行し、結果を破棄する。
    // Sensirionアプリノートによると、電源サイクル（power_down/wake_up含む）後の
    // 最初の測定値は精度最大化のために破棄しなければならない。
    Serial.println("Discarding first reading for stabilization...");
    error = scd4x.measureSingleShot();
    if (logSensorError(error, "measureSingleShot() [stabilization]"))
    {
        return CO2_INVALID;
    }
    // Read and discard the stabilization measurement to empty the buffer.
    // バッファをクリアするため、安定化測定の値を読み出して捨てる。
    {
        uint16_t dummy_co2 = 0;
        float dummy_temp = 0.0f;
        float dummy_hum = 0.0f;
        scd4x.readMeasurement(dummy_co2, dummy_temp, dummy_hum);
        Serial.println("Stabilization reading discarded.");
    }

    // Trigger the actual single-shot measurement (CO2 + RHT).
    // 本番のシングルショット測定（CO2 + 温湿度）を開始する。
    error = scd4x.measureSingleShot();
    if (logSensorError(error, "measureSingleShot()"))
    {
        return CO2_INVALID;
    }

    // Check whether new measurement data is available for read-out.
    // 測定データが読み出し可能かどうか確認する。
    bool isDataReady = false;
    error = scd4x.getDataReadyFlag(isDataReady);
    if (logSensorError(error, "getDataReadyFlag()"))
    {
        return CO2_INVALID;
    }
    if (!isDataReady)
    {
        Serial.println("Data not ready.");
        return CO2_INVALID;
    }

    // Read sensor output.
    // センサーの測定値を読み出す。
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (logSensorError(error, "readMeasurement()"))
    {
        return CO2_INVALID;
    }
    if (co2 == 0)
    {
        Serial.println("Invalid sample detected, skipping.");
        return CO2_INVALID;
    }

    Serial.print("CO2:");
    Serial.print(co2);
    Serial.print("\t");
    Serial.print("Temperature:");
    Serial.print(temperature);
    Serial.print("\t");
    Serial.print("Humidity:");
    Serial.println(humidity);

    return co2;
}

/**
 * Draw CO2 concentration to E-ink display.
 *
 * CO2濃度をE-inkディスプレイに描画する。
 */
void drawCo2(int value)
{
    const int textBufferSize = 40;
    char buffer[textBufferSize];

    // Prepare a blank canvas for drawing.
    // 描画用に白紙のキャンバスを準備する。
    Paint_NewImage(imageBW, EPD_W, EPD_H, Rotation, WHITE);
    Paint_Clear(WHITE);
    EPD_FastMode1Init();

    // Draw the "CO2" label.
    // 「CO2」ラベルを描画する。
    EPD_ShowString(20, 10, "CO2", 48, BLACK);

    // Draw the CO2 concentration value.
    // CO2濃度の値を描画する。
    snprintf(buffer, sizeof(buffer), "%3d", value);
    EPD_ShowString(220, 20, buffer, 224, BLACK);

    // Draw the "ppm" unit label.
    // 単位「ppm」を描画する。
    EPD_ShowString(700, 210, "ppm", 48, BLACK);

    // Transfer the canvas to the display and refresh it.
    // キャンバスの内容をディスプレイに転送し、更新する。
    EPD_Display(imageBW);
    EPD_FastUpdate();
    EPD_Clear_R26A6H();

    delay(300);
}

/**
 * Enter Deep-Sleep mode.
 *
 * Deep-Sleepモードに入る。
 */
void deepSleep()
{
    // Cut power to the E-Paper display.
    // E-Paperディスプレイへの給電を停止する。
    digitalWrite(epdPowerPin, LOW);

    // Put the CO2 sensor into sleep mode to reduce its current consumption
    // during the ESP32 Deep-Sleep interval.
    // ESP32のDeep-Sleep中のCO2センサー消費電流を削減するため、センサーをスリープ状態にする。
    scd4x.powerDown();

    // Power down every RTC domain that is not required for timer wake-up.
    // - RTC_PERIPH : RTC IO, internal sensors, ULP
    // - RTC_SLOW_MEM: RTC_DATA_ATTR variables
    // - RTC_FAST_MEM: RTC fast SRAM
    // - XTAL        : 40 MHz crystal
    // タイマーウェイクアップに不要なRTCドメインをすべてパワーダウンする。
    // - RTC_PERIPH  : RTC IO・内部センサー・ULP
    // - RTC_SLOW_MEM: RTC_DATA_ATTR変数用
    // - RTC_FAST_MEM: RTCファストSRAM
    // - XTAL        : 40 MHz水晶発振器
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH,   ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,         ESP_PD_OPTION_OFF);

    // Start deep sleep to wake up on RTC timer.
    // RTCタイマーで復帰するようにDeep-Sleepを開始する。
    esp_sleep_enable_timer_wakeup(sleepDurationInSec * 1000000ULL);
    esp_deep_sleep_start();
}

/**
 * Arduino entry point, executed once after each power-on or wake-up from Deep-Sleep.
 * Initializes peripherals, optionally performs FRC, measures CO2, updates the
 * display, and finally enters Deep-Sleep again.
 *
 * Arduinoのエントリーポイント。電源投入時またはDeep-Sleepからの復帰時に一度だけ実行される。
 * 周辺機器を初期化し、必要に応じてFRCを実行し、CO2を測定してディスプレイを更新した後、
 * 再びDeep-Sleepに入る。
 */
void setup()
{
    Serial.begin(115200);

    // Initialize E-Paper related GPIO pins.
    // E-Paper関連のGPIOピンを初期化する。
    EPD_GPIOInit();

    // Configure button pins as inputs with internal pull-up resistors.
    // ボタンピンを内部プルアップ付きの入力として設定する。
    pinMode(exitButtonPin, INPUT_PULLUP);
    pinMode(menuButtonPin, INPUT_PULLUP);
    pinMode(dialPushPin, INPUT_PULLUP);

    // Power on the E-Paper display.
    // E-Paperディスプレイの電源を投入する。
    pinMode(epdPowerPin, OUTPUT);
    digitalWrite(epdPowerPin, HIGH);

    // Re-initialize E-Paper GPIO pins now that power is supplied.
    // 電源投入後、E-PaperのGPIOピンを再初期化する。
    EPD_GPIOInit();

    // Initialize I2C bus and the CO2 sensor.
    // I2Cバスを初期化し、CO2センサーを初期化する。
    Wire1.begin(i2cSdaPin, i2cSclPin);
    scd4x.begin(Wire1);

    // Wake the sensor up unconditionally here.
    // 無条件にセンサーをウェイクアップする。
    scd4x.wakeUp();

    // Disable Automatic self-calibration (ASC).
    // Automatic Self-Calibration（ASC）を無効にする。
    disableAsc();

    // Detect any button presses.
    // 何らかのボタンが押されていることを検知する。
    delay(100);
    bool anyButtonPressed = (digitalRead(exitButtonPin) == LOW) ||
                            (digitalRead(menuButtonPin) == LOW) ||
                            (digitalRead(dialPushPin) == LOW);
    if (anyButtonPressed)
    {
        // Perform Forced recalibration (FRC).
        // Forced recalibration（FRC）を実行する。
        performFrc();
    }

    // Measure CO2 concentration, humidity and temperature on demand.
    // CO2濃度、湿度、温度をオンデマンドで測定する。
    uint16_t co2 = measureCo2();

    // Draw CO2 concentration to E-ink display only when measurement succeeded.
    // CO2濃度をE-inkディスプレイに描画する（測定成功時のみ）。
    if (co2 == CO2_INVALID)
    {
        Serial.println("Measurement failed. Skipping display update.");
    }
    else
    {
        drawCo2(co2);
    }

    // Enter Deep-Sleep mode.
    // Deep Sleepに入り、sleepDurationInSec秒後にsetup()から再起動する。
    deepSleep();
}

void loop()
{
    // This function is never reached because deepSleep() in setup() resets
    // the chip. Execution always restarts from setup() after wake-up.
    //
    // deepSleep()がsetup()内で呼ばれるため、この関数には到達しない。
    // 復帰後は常にsetup()から再実行される。
}
