/**
 * Project     : 6DOF arm
 * Codename    : CLAW
 * Author      : Vladimir V. Laykov aka Yago
 * Alma Mater  : Sevastopol State University
 * Group       : УТС/б-23-1-о
 * Mentor      : Lipko I.Y.
 * Date        : 2026-09-18
 * Last modify : 2026-10-01
 */

/*
 * eeprom_storage.cpp — Реализация модуля хранения данных в EEPROM
 *
 * Обеспечивает постоянное хранение конфигурации 6-DOF роботической манипуляции
 * в энергонезависимой памяти EEPROM. Данные записываются/читаются блоком
 * начиная с адреса 0. Для проверки целостности используется контрольная сумма.
 *
 * Структура данных в EEPROM (адрес 0):
 *   [EEPROMData — ~132 байта в зависимости от компилятора]
 *
 * Формат контрольной суммы:
 *   XOR всех байтов структуры (кроме checksum) с ротацией влево на 1 бит.
 *   Это простой, но эффективный способ обнаружения ошибок чтения/записи.
 *
 * При сохранении:
 *   1. Заполняется структура EEPROMData из переданных параметров
 *   2. Вычисляется checksum и записывается в поле checksum
 *   3. Структура записывается в EEPROM через EEPROM.put()
 *   4. Данные фиксируются через EEPROM.commit()
 *
 * При загрузке:
 *   1. Структура читается из EEPROM через EEPROM.get()
 *   2. Проверяется magic и checksum через validateData()
 *   3. Если данные валидны — копируются в выходные параметры
 *   4. Если невалидны — возвращается false, вызывающий код использует defaults
 */

#include "eeprom_storage.h"
#include <cstddef>

/**
 * begin — Инициализация EEPROM.
 *
 * Выделяет EEPROM_SIZE (512) байт памяти для операций чтения/записи.
 * Должен быть вызван один раз в начале работы программы (обычно в setup()).
 * Сразу грузит кэш одним EEPROM.get: дальше все чтения идут из RAM.
 */
void EEPROMStorage::begin() {
    EEPROM.begin(EEPROM_SIZE);
    ensureCache();
}

/**
 * ensureCache — ленивая подгрузка кэша (одно чтение на всё время жизни).
 *
 * Если кэш ещё не готов — один EEPROM.get(0) + validateData, результат
 * в data/cacheValid/cacheReady. Повторные вызовы ничего не читают.
 */
void EEPROMStorage::ensureCache() {
    if (cacheReady) return;
    EEPROMData d;
    EEPROM.get(0, d);
    data = d;
    cacheValid = validateData(d);
    cacheReady = true;
}

/**
 * saveConfig — Сохранение полной конфигурации + домашних углов в EEPROM.
 *
 * Алгоритм:
 *   1. Создаётся локальная структура EEPROMData и заполняется нулями.
 *   2. Устанавливается magic = EEPROM_MAGIC (0xABCD) для последующей валидации.
 *   3. Копируются конфигурации 7 сервоприводов из config.servos[].
 *   4. Копируются домашние углы из homeAngles[].
 *   5. Копируется компенсация люфта из config.servos[i].backlashCompensation.
 *   6. Копируются ограничения рабочей области и безопасности звеньев.
 *   7. Устанавливается длина пальца захвата и стратегия интерполяции.
 *   8. Вычисляется checksum через calculateChecksum() и записывается в структуру.
 *   9. Структура сохраняется во внутренний буфер data для дальнейшего использования.
 *  10. Структура записывается в EEPROM через EEPROM.put() и фиксируется через
 *      EEPROM.commit() для обеспеченияERSISTENTности данных.
 *
 * @param config — конфигурация манипуляции для сохранения
 * @param homeAngles — массив из 7 домашних углов сервоприводов
 */
void EEPROMStorage::saveConfig(const ManipulatorConfig &config, const float homeAngles[7], InterpolationStrategy strategy) {
    EEPROMData d;
    d.magic = EEPROM_MAGIC;
    d.fwVersion = FW_VERSION;

    for (int i = 0; i < 7; i++) {
        d.servos[i] = config.servos[i];
        d.homeAngles[i] = homeAngles[i];
        d.backlash[i] = config.servos[i].backlashCompensation;
    }

    d.workspace = config.workspace;
    d.linkSafety = config.linkSafety;
    d.fingerLength = config.fingerLength;
    d.strategy = strategy;
    for (int i = 0; i < MAX_FORBIDDEN_ZONES; i++) {
        d.zones[i] = config.forbiddenZones[i];
    }
    d.zoneCount = config.forbiddenZoneCount;

    d.checksum = calculateChecksum(d);
    data = d;
    cacheReady = true;
    cacheValid = true;

    EEPROM.put(0, d);
    EEPROM.commit();
}

/**
 * loadConfig — Загрузка полной конфигурации + домашних углов из EEPROM.
 *
 * Алгоритм:
 *   1. Данные берутся из кэша (одно чтение в begin()/ensureCache()).
 *   2. Валидация через validateData() — проверка magic и checksum.
 *   3. Если данные валидны:
  *      a. Копирование конфигураций сервоприводов в config.servos[].
  *      b. Копирование домашних углов в homeAngles[].
  *      c. Копирование ограничений рабочей области и безопасности.
  *      d. Возврат true.
 *   4. Если данные невалидны — возврат false (вызывающий код должен
 *      использовать значения по умолчанию).
 *
 * @param config — ссылка на структуру конфигурации для заполнения
 * @param homeAngles — массив из 7 элементов для заполнения домашних углов
 * @return true если данные успешно загружены, false если данные невалидны
 */
bool EEPROMStorage::loadConfig(ManipulatorConfig &config, float homeAngles[7]) {
    ensureCache();
    const EEPROMData &d = data;

    if (!cacheValid) {
        return false;
    }

    for (int i = 0; i < 7; i++) {
        config.servos[i] = d.servos[i];
        homeAngles[i] = d.homeAngles[i];
    }

    config.workspace = d.workspace;
    config.linkSafety = d.linkSafety;
    config.fingerLength = d.fingerLength;
    for (int i = 0; i < MAX_FORBIDDEN_ZONES; i++) {
        config.forbiddenZones[i] = d.zones[i];
    }
    config.forbiddenZoneCount = d.zoneCount;

    return true;
}

/**
 * saveHome — Сохранение только домашних углов в EEPROM.
 *
 * Этот метод позволяет обновить домашние углы без перезаписи всей конфигурации.
 *
 * Алгоритм:
 *   1. Чтение текущей структуры из EEPROM.
 *   2. Проверка magic — если данные уже записаны (magic совпадает), они
 *      сохраняются и обновляются только homeAngles.
 *   3. Если magic не совпадает (EEPROM пуста или повреждена):
 *      a. Создаётся новая структура с нулями.
 *      b. Устанавливается magic = EEPROM_MAGIC.
 *      c. Конфигурация сервоприводов заполняется из DEFAULT_CONFIG.
 *      d. Ограничения рабочей области и безопасности также из DEFAULT_CONFIG.
 *   4. Копируются переданные homeAngles в структуру.
 *   5. Вычисляется checksum и записывается в структуру.
 *   6. Структура сохраняется во внутренний буфер и записывается в EEPROM.
 *
 * @param homeAngles — массив из 7 домашних углов сервоприводов
 */
void EEPROMStorage::saveHome(const float homeAngles[7]) {
    ensureCache();
    EEPROMData d = data;

    // Как раньше: смотрим только magic (не полный validate). Если magic на месте,
    // но checksum битый — базу чиним: конфиг сохраняем, checksum пересчитаем ниже.
    if (d.magic != EEPROM_MAGIC) {
        d = {};
        d.magic = EEPROM_MAGIC;
        d.fwVersion = FW_VERSION;
        for (int i = 0; i < 7; i++) {
            d.servos[i] = DEFAULT_CONFIG.servos[i];
        }
        d.workspace = DEFAULT_CONFIG.workspace;
        d.linkSafety = DEFAULT_CONFIG.linkSafety;
        for (int i = 0; i < MAX_FORBIDDEN_ZONES; i++) {
            d.zones[i] = DEFAULT_CONFIG.forbiddenZones[i];
        }
        d.zoneCount = DEFAULT_CONFIG.forbiddenZoneCount;
    }
    d.fwVersion = FW_VERSION;   // любое сохранение штампует текущую версию

    for (int i = 0; i < 7; i++) {
        d.homeAngles[i] = homeAngles[i];
    }

    d.checksum = calculateChecksum(d);
    data = d;
    cacheReady = true;
    cacheValid = true;

    EEPROM.put(0, d);
    EEPROM.commit();
}

/**
 * loadHome — Загрузка только домашних углов из EEPROM.
 *
 * Алгоритм:
 *   1. Данные берутся из кэша.
 *   2. Валидация через validateData().
 *   3. Если данные валидны — копирование homeAngles из структуры, возврат true.
 *   4. Если данные невалидны — заполнение homeAngles значениями
 *      DEFAULT_HOME_ANGLES (значения по умолчанию), возврат false.
 *
 * @param homeAngles — массив из 7 элементов для заполнения домашних углов
 * @return true если данные успешно загружены, false если используются defaults
 */
bool EEPROMStorage::loadHome(float homeAngles[7]) {
    ensureCache();

    if (!cacheValid) {
        for (int i = 0; i < 7; i++) {
            homeAngles[i] = DEFAULT_HOME_ANGLES[i];
        }
        return false;
    }

    for (int i = 0; i < 7; i++) {
        homeAngles[i] = data.homeAngles[i];
    }

    return true;
}

/**
 * clear — Очистка EEPROM (заполнение нулями).
 *
 * Записывает нулевую структуру EEPROMData в EEPROM. После этого
 * magic будет равен 0 (не совпадает с EEPROM_MAGIC = 0xABCD),
 * что заставит все методы загрузки считать данные невалидными
 * и использовать значения по умолчанию.
 */
void EEPROMStorage::clear() {
    EEPROMData d = {};
    data = d;
    cacheReady = true;
    cacheValid = false;
    EEPROM.put(0, d);
    EEPROM.commit();
}

/**
 * isConfigValid — Проверка корректности данных в EEPROM.
 *
 * Идёт из кэша, EEPROM не читает. Не изменяет внутренний буфер data.
 *
 * @return true если magic и checksum совпадают, false в противном случае
 */
bool EEPROMStorage::isConfigValid() {
    ensureCache();
    return cacheValid;
}

/**
 * getStoredFwVersion — версия прошивки из EEPROM (0 = нет/битые/старый формат).
 */
uint32_t EEPROMStorage::getStoredFwVersion() {
    ensureCache();
    if (!cacheValid) return 0;
    return data.fwVersion;
}

/**
 * calculateChecksum — Вычисление контрольной суммы структуры EEPROMData.
 *
 * Алгоритм:
 *   1. Берём все байты структуры как массив uint8_t.
 *   2. Пропускаем байты поля checksum (2 байта по смещению offsetof(EEPROMData, checksum)).
 *   3. Для каждого оставшегося байта:
 *      a. Выполняем XOR с накопленным значением sum.
 *      b. Выполняем ротацию sum влево на 1 бит (циклический сдвиг).
 *         Это эквивалентно: (sum << 1) | (sum >> 15)
 *   4. Возвращаем итоговое 16-битное значение.
 *
 * Особенности:
 *   - Ротация после каждого XOR обеспечивает чувствительность к порядку байтов.
 *   - Если один байт изменится, checksum изменится существенно.
 *   - Поле checksum исключено из вычисления, чтобы его значение не влияло на себя.
 *
 * @param d — структура данных для вычисления контрольной суммы
 * @return 16-битная контрольная сумма
 */
uint16_t EEPROMStorage::calculateChecksum(const EEPROMData &d) {
    uint16_t sum = 0;
    const uint8_t *ptr = (const uint8_t *)&d;
    const size_t offset = offsetof(EEPROMData, checksum);
    const size_t total = sizeof(EEPROMData);
    // Два диапазона вместо бранча на каждый байт: до checksum и после.
    // Математика та же (XOR + ротация влево на 1 бит после каждого байта).
    for (size_t i = 0; i < offset; i++) {
        sum ^= ptr[i];
        sum = (sum << 1) | (sum >> 15);  // ротация влево на 1 бит
    }
    for (size_t i = offset + sizeof(uint16_t); i < total; i++) {
        sum ^= ptr[i];
        sum = (sum << 1) | (sum >> 15);
    }
    return sum;
}

/**
 * validateData — Валидация данных из EEPROM.
 *
 * Проверяет два критических условия:
 *   1. magic == EEPROM_MAGIC (0xABCD) — данные были записаны нашим кодом,
 *      а не являются результатом случайной инициализации EEPROM.
 *   2. checksum == calculateChecksum(d) — данные не повреждены при чтении/записи.
 *
 * Если хотя бы одно условие не выполнено, данные считаются невалидными
 * и вызывающий код должен использовать значения по умолчанию.
 *
 * @param d — структура данных для валидации
 * @return true если данные валидны, false если повреждены или отсутствуют
 */
bool EEPROMStorage::validateData(const EEPROMData &d) {
    if (d.magic != EEPROM_MAGIC) return false;
    if (d.checksum != calculateChecksum(d)) return false;
    return true;
}
