/**
 * Project     : 6DOF arm
 * Author      : Vladimir V. Laykov aka Yago
 * Group       :
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-09-18
 */

/*
 * eeprom_storage.h — Заголовочный файл модуля хранения данных в EEPROM
 *
 * Модуль обеспечивает постоянное хранение конфигурации роботической манипуляции
 * в энергонезависимой памяти EEPROM. Включает:
 *
 * 1) Структуру данных EEPROMData:
 *    - magic (0xABCD) — магическое число для валидации данных
 *    - servos[7] — конфигурация 7 сервоприводов ( ServoConfig )
 *    - homeAngles[7] — домашние углы 7 сервоприводов
 *    - workspace — ограничения рабочей области
 *    - linkSafety — параметры безопасности звеньев
 *    - backlash[7] — компенсация люфта 7 сервоприводов
 *    - fingerLength — длина пальца захвата
 *    - strategy — стратегия интерполяции (по умолчанию INTERP_AUTO)
 *    - checksum — контрольная сумма для целостности данных
 *
 * 2) Валидация данных через checksum:
 *    - calculateChecksum() — вычисление контрольной суммы (XOR + rotate)
 *    - validateData() — проверка magic и checksum
 *
 * 3) Методы сохранения/загрузки:
 *    - saveConfig() — сохранение полной конфигурации + домашних углов
 *    - loadConfig() — загрузка полной конфигурации + домашних углов
 *    - saveHome() — сохранение только домашних углов (с сохранением конфигурации)
 *    - loadHome() — загрузка только домашних углов (с fallback на DEFAULT_HOME_ANGLES)
 *    - clear() — очистка EEPROM (заполнение нулями)
 *    - isConfigValid() — проверка корректности сохранённых данных
 *
 * 4) Структура EEPROM в памяти (адрес 0):
 *    [magic][servos][homeAngles][workspace][linkSafety][backlash][fingerLength][strategy][checksum]
 *
 * При записи данные записываются целиком через EEPROM.put() и фиксируются
 * через EEPROM.commit(). При чтении данные читаются через EEPROM.get() и
 * проверяются через validateData().
 */

#pragma once

#include <EEPROM.h>
#include "config.h"
#include "types.h"

/** EEPROM_SIZE — размер выделяемой памяти EEPROM (512 байт) */
#define EEPROM_SIZE 512

/** EEPROM_MAGIC — магическое число для валидации структуры данных (0xABCD) */
#define EEPROM_MAGIC 0xABCD

/**
 * EEPROMData — структура данных, хранимых в EEPROM.
 * Записывается/читается целиком через EEPROM.put()/EEPROM.get().
 *
 * Поля:
 *   magic       — магическое число EEPROM_MAGIC (0xABCD) для проверки наличия данных
 *   servos[7]   — конфигурация 7 сервоприводов (углы, компенсация люфта и т.д.)
 *   homeAngles[7] — домашние углы 7 сервоприводов (начальная позиция)
 *   workspace   — ограничения рабочей области (допустимые диапазоны перемещения)
 *   linkSafety  — параметры безопасности звеньев (ограничения крутящего момента)
 *   backlash[7] — компенсация люфта 7 сервоприводов (для точного позиционирования)
 *   fingerLength — длина пальца захвата (для кинематических вычислений)
 *   strategy    — стратегия интерполяции движений
 *   checksum    — контрольная сумма для проверки целостности данных
 */
struct EEPROMData {
    uint16_t           magic;           // Магическое число для валидации (0xABCD)
    ServoConfig        servos[7];       // Конфигурация 7 сервоприводов
    float              homeAngles[7];   // Домашние углы 7 сервоприводов
    WorkspaceLimits    workspace;       // Ограничения рабочей области
    LinkSafety         linkSafety;      // Параметры безопасности звеньев
    float              backlash[7];     // Компенсация люфта 7 сервоприводов
    float              fingerLength;    // Длина пальца захвата
    InterpolationStrategy strategy;     // Стратегия интерполяции
    uint16_t           checksum;        // Контрольная сумма (в самом конце структуры)
};

/**
 * EEPROMStorage — класс для управления постоянным хранением данных в EEPROM.
 *
 * Обеспечивает сохранение/загрузку конфигурации робота и домашних углов
 * с валидацией через контрольную сумму. Данные записываются/читаются
 * блоком начиная с адреса 0.
 */
class EEPROMStorage {
public:
    /**
     * begin — Инициализация EEPROM.
     * Вызывается один раз при старте системы.
     * Выделяет EEPROM_SIZE байт памяти.
     */
    void begin();

    /**
     * saveConfig — Сохранение полной конфигурации + домашних углов в EEPROM.
     *
     * Алгоритм:
     *   1. Создание структуры EEPROMData и заполнение её данными из config и homeAngles.
     *   2. Установка magic = EEPROM_MAGIC (0xABCD).
     *   3. Вычисление контрольной суммы через calculateChecksum().
     *   4. Запись структуры в EEPROM через EEPROM.put() и EEPROM.commit().
     *
     * @param config — конфигурация манипуляции для сохранения
     * @param homeAngles — массив из 7 домашних углов сервоприводов
     */
    void saveConfig(const ManipulatorConfig &config, const float homeAngles[7]);

    /**
     * loadConfig — Загрузка полной конфигурации + домашних углов из EEPROM.
     *
     * Алгоритм:
     *   1. Чтение структуры EEPROMData из EEPROM через EEPROM.get().
     *   2. Валидация данных через validateData() (проверка magic и checksum).
     *   3. Если данные валидны — копирование в config и homeAngles, возврат true.
     *   4. Если данные невалидны — возврат false (используются значения по умолчанию).
     *
     * @param config — ссылка на структуру конфигурации для заполнения
     * @param homeAngles — массив из 7 элементов для заполнения домашних углов
     * @return true если данные успешно загружены, false если данные невалидны
     */
    bool loadConfig(ManipulatorConfig &config, float homeAngles[7]);

    /**
     * saveHome — Сохранение только домашних углов в EEPROM.
     *
     * Если EEPROM уже содержит валидные данные (magic совпадает), они сохраняются,
     * а homeAngles обновляются. Если данные невалидны, создаётся новая структура
     * с конфигурацией по умолчанию (DEFAULT_CONFIG) и переданными homeAngles.
     *
     * @param homeAngles — массив из 7 домашних углов сервоприводов
     */
    void saveHome(const float homeAngles[7]);

    /**
     * loadHome — Загрузка только домашних углов из EEPROM.
     *
     * Если данные валидны — копирует homeAngles из EEPROM и возвращает true.
     * Если данные невалидны — заполняет homeAngles значениями по умолчанию
     * (DEFAULT_HOME_ANGLES) и возвращает false.
     *
     * @param homeAngles — массив из 7 элементов для заполнения домашних углов
     * @return true если данные успешно загружены, false если используются defaults
     */
    bool loadHome(float homeAngles[7]);

    /**
     * clear — Очистка EEPROM (заполнение нулями).
     * После вызова magic не будет совпадать с EEPROM_MAGIC,
     * что заставит все методы загрузки использовать значения по умолчанию.
     */
    void clear();

    /**
     * isConfigValid — Проверка корректности данных в EEPROM.
     * @return true если magic и checksum совпадают, false в противном случае
     */
    bool isConfigValid();

private:
    EEPROMData data;  // Внутренний буфер для хранения прочитанных данных

    /**
     * calculateChecksum — Вычисление контрольной суммы структуры EEPROMData.
     *
     * Алгоритм: XOR всех байтов структуры (кроме поля checksum) с ротацией
     * влево на 1 бит после каждого XOR. Это обеспечивает чувствительность
     * к порядку и количеству байтов.
     *
     * @param d — структура данных для вычисления контрольной суммы
     * @return 16-битная контрольная сумма
     */
    uint16_t calculateChecksum(const EEPROMData &d);

    /**
     * validateData — Валидация данных из EEPROM.
     *
     * Проверяет два условия:
     *   1. magic == EEPROM_MAGIC (0xABCD) — данные были записаны нашим кодом
     *   2. checksum == calculateChecksum(d) — данные не повреждены
     *
     * @param d — структура данных для валидации
     * @return true если данные валидны, false если повреждены или отсутствуют
     */
    bool validateData(const EEPROMData &d);
};
