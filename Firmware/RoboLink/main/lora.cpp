#include "lora.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

#include "io.h"
#include "packet_queue.h"

static const char *TAG = "LoRa";

static spi_device_handle_t lora_spi = nullptr;

static uint8_t read_register(uint8_t reg)
{
    spi_transaction_t transaction = {};

    uint8_t tx[2] = {
        static_cast<uint8_t>(reg & 0x7F),
        0x00
    };

    uint8_t rx[2] = {};

    transaction.length = 16;
    transaction.tx_buffer = tx;
    transaction.rx_buffer = rx;

    ESP_ERROR_CHECK(
        spi_device_transmit(
            lora_spi,
            &transaction
        )
    );

    return rx[1];
}

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    spi_transaction_t transaction = {};

    uint8_t tx[2] = {
        static_cast<uint8_t>(reg | 0x80),
        value
    };

    transaction.length = 16;
    transaction.tx_buffer = tx;

    return spi_device_transmit(
        lora_spi,
        &transaction
    );
}

esp_err_t lora_spi_init()
{
    ESP_LOGI(TAG, "Initializing LoRa SPI");

    spi_bus_config_t bus_config = {};

    bus_config.mosi_io_num = RADIO_MOSI_PIN;
    bus_config.miso_io_num = RADIO_MISO_PIN;
    bus_config.sclk_io_num = RADIO_SCLK_PIN;
    bus_config.quadwp_io_num = -1;
    bus_config.quadhd_io_num = -1;
    bus_config.max_transfer_sz = 256;

    ESP_ERROR_CHECK(
        spi_bus_initialize(
            SPI2_HOST,
            &bus_config,
            SPI_DMA_DISABLED
        )
    );

    spi_device_interface_config_t device_config = {};

    device_config.clock_speed_hz = 1000000;
    device_config.mode = 0;
    device_config.spics_io_num = RADIO_CS_PIN;
    device_config.queue_size = 1;

    ESP_ERROR_CHECK(
        spi_bus_add_device(
            SPI2_HOST,
            &device_config,
            &lora_spi
        )
    );

    ESP_LOGI(
        TAG,
        "SPI ready: SCLK=%d MISO=%d MOSI=%d CS=%d",
        RADIO_SCLK_PIN,
        RADIO_MISO_PIN,
        RADIO_MOSI_PIN,
        RADIO_CS_PIN
    );

    return ESP_OK;
}

esp_err_t lora_check_radio()
{
    constexpr uint8_t REG_VERSION = 0x42;

    uint8_t version = read_register(REG_VERSION);

    ESP_LOGI(
        TAG,
        "Radio version register 0x42 = 0x%02X",
        version
    );

    if (version == 0x00 || version == 0xFF)
    {
        ESP_LOGE(
            TAG,
            "No valid response from LoRa radio"
        );

        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LoRa SPI communication OK");

    return ESP_OK;
}

esp_err_t lora_configure()
{
    constexpr uint8_t REG_OP_MODE = 0x01;
    constexpr uint8_t REG_FRF_MSB = 0x06;
    constexpr uint8_t REG_FRF_MID = 0x07;
    constexpr uint8_t REG_FRF_LSB = 0x08;
    constexpr uint8_t REG_PA_CONFIG = 0x09;
    constexpr uint8_t REG_OCP = 0x0B;
    constexpr uint8_t REG_LNA = 0x0C;
    constexpr uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
    constexpr uint8_t REG_FIFO_RX_BASE_ADDR = 0x0F;
    constexpr uint8_t REG_MODEM_CONFIG_2 = 0x1E;
    constexpr uint8_t REG_MODEM_CONFIG_3 = 0x26;
    constexpr uint8_t REG_VERSION = 0x42;
    constexpr uint8_t REG_PA_DAC = 0x4D;

    constexpr uint8_t MODE_LONG_RANGE = 0x80;
    constexpr uint8_t MODE_SLEEP = 0x00;
    constexpr uint8_t MODE_STANDBY = 0x01;

    ESP_LOGI(TAG, "Resetting SX127x");

    gpio_config_t reset_config = {};
    reset_config.pin_bit_mask = (1ULL << RADIO_RST_PIN);
    reset_config.mode = GPIO_MODE_OUTPUT;
    reset_config.pull_up_en = GPIO_PULLUP_DISABLE;
    reset_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    reset_config.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&reset_config));

    gpio_set_level((gpio_num_t)RADIO_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level((gpio_num_t)RADIO_RST_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t version = read_register(REG_VERSION);

    ESP_LOGI(TAG, "Radio version after reset = 0x%02X", version);

    if (version != 0x12)
    {
        ESP_LOGE(TAG, "Unexpected SX127x version");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Configuring RoboLora-compatible radio settings");

    ESP_ERROR_CHECK(
        write_register(
            REG_OP_MODE,
            MODE_LONG_RANGE | MODE_SLEEP
        )
    );

    vTaskDelay(pdMS_TO_TICKS(10));

    const uint64_t frf =
        ((uint64_t)LORA_FREQUENCY << 19) / 32000000ULL;

    ESP_ERROR_CHECK(
        write_register(
            REG_FRF_MSB,
            (uint8_t)(frf >> 16)
        )
    );

    ESP_ERROR_CHECK(
        write_register(
            REG_FRF_MID,
            (uint8_t)(frf >> 8)
        )
    );

    ESP_ERROR_CHECK(
        write_register(
            REG_FRF_LSB,
            (uint8_t)frf
        )
    );

    ESP_ERROR_CHECK(
        write_register(
            REG_FIFO_TX_BASE_ADDR,
            0x00
        )
    );

    ESP_ERROR_CHECK(
        write_register(
            REG_FIFO_RX_BASE_ADDR,
            0x00
        )
    );

    uint8_t lna = read_register(REG_LNA);

    ESP_ERROR_CHECK(
        write_register(
            REG_LNA,
            lna | 0x03
        )
    );

    ESP_ERROR_CHECK(
        write_register(
            REG_MODEM_CONFIG_3,
            0x04
        )
    );

    /*
     * Match RoboLora LoRa.setTxPower(17).
     *
     * PA_BOOST selected.
     * PA_DAC normal +17 dBm mode.
     * OCP approximately 100 mA.
     */
    ESP_ERROR_CHECK(
        write_register(
            REG_PA_DAC,
            0x84
        )
    );

    ESP_ERROR_CHECK(
        write_register(
            REG_OCP,
            0x2B
        )
    );

    ESP_ERROR_CHECK(
        write_register(
            REG_PA_CONFIG,
            0x8F
        )
    );

    /*
     * RoboLora explicitly calls LoRa.enableCrc().
     * This sets bit 2 of RegModemConfig2.
     */
    uint8_t modem_config_2 =
        read_register(REG_MODEM_CONFIG_2);

    ESP_ERROR_CHECK(
        write_register(
            REG_MODEM_CONFIG_2,
            modem_config_2 | 0x04
        )
    );

    /*
     * Do NOT transmit.
     * Leave the SX127x in LoRa standby mode.
     */
    ESP_ERROR_CHECK(
        write_register(
            REG_OP_MODE,
            MODE_LONG_RANGE | MODE_STANDBY
        )
    );

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t op_mode = read_register(REG_OP_MODE);
    uint8_t frf_msb = read_register(REG_FRF_MSB);
    uint8_t frf_mid = read_register(REG_FRF_MID);
    uint8_t frf_lsb = read_register(REG_FRF_LSB);
    uint8_t tx_base = read_register(REG_FIFO_TX_BASE_ADDR);
    uint8_t rx_base = read_register(REG_FIFO_RX_BASE_ADDR);
    uint8_t lna_readback = read_register(REG_LNA);
    uint8_t modem2 = read_register(REG_MODEM_CONFIG_2);
    uint8_t modem3 = read_register(REG_MODEM_CONFIG_3);
    uint8_t pa_config = read_register(REG_PA_CONFIG);
    uint8_t pa_dac = read_register(REG_PA_DAC);
    uint8_t ocp = read_register(REG_OCP);

    uint32_t frf_readback =
        ((uint32_t)frf_msb << 16) |
        ((uint32_t)frf_mid << 8) |
        frf_lsb;

    uint32_t frequency =
        ((uint64_t)frf_readback * 32000000ULL) >> 19;

    ESP_LOGI(TAG, "OpMode        = 0x%02X", op_mode);
    ESP_LOGI(TAG, "Frequency     = %lu Hz", (unsigned long)frequency);
    ESP_LOGI(TAG, "FIFO TX base  = 0x%02X", tx_base);
    ESP_LOGI(TAG, "FIFO RX base  = 0x%02X", rx_base);
    ESP_LOGI(TAG, "LNA           = 0x%02X", lna_readback);
    ESP_LOGI(TAG, "ModemConfig2  = 0x%02X", modem2);
    ESP_LOGI(TAG, "ModemConfig3  = 0x%02X", modem3);
    ESP_LOGI(TAG, "PA config     = 0x%02X", pa_config);
    ESP_LOGI(TAG, "PA DAC        = 0x%02X", pa_dac);
    ESP_LOGI(TAG, "OCP           = 0x%02X", ocp);

    if (op_mode != 0x81)
    {
        ESP_LOGE(TAG, "Radio is not in LoRa standby mode");
        return ESP_FAIL;
    }

    if (frequency < 432900000 ||
        frequency > 433100000)
    {
        ESP_LOGE(TAG, "433 MHz frequency verification failed");
        return ESP_FAIL;
    }

    if ((modem2 & 0x04) == 0)
    {
        ESP_LOGE(TAG, "LoRa CRC is not enabled");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RoboLora-compatible SX127x configuration OK");

    return ESP_OK;
}

esp_err_t lora_start_receive()
{
    constexpr uint8_t REG_OP_MODE = 0x01;
    constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0D;
    constexpr uint8_t REG_IRQ_FLAGS = 0x12;
    constexpr uint8_t REG_MODEM_CONFIG_1 = 0x1D;
    constexpr uint8_t REG_DIO_MAPPING_1 = 0x40;

    constexpr uint8_t MODE_LONG_RANGE = 0x80;
    constexpr uint8_t MODE_RX_CONTINUOUS = 0x05;

    ESP_LOGI(TAG, "Starting LoRa receive-only mode");

    // Explicit header mode, matching RoboLora's normal packet format.
    uint8_t modem_config_1 = read_register(REG_MODEM_CONFIG_1);

    ESP_ERROR_CHECK(
        write_register(
            REG_MODEM_CONFIG_1,
            modem_config_1 & 0xFE
        )
    );

    // Start reading RX packets from FIFO base address 0.
    ESP_ERROR_CHECK(
        write_register(
            REG_FIFO_ADDR_PTR,
            0x00
        )
    );

    // Clear any IRQ flags left from initialization.
    ESP_ERROR_CHECK(
        write_register(
            REG_IRQ_FLAGS,
            0xFF
        )
    );

    // DIO0 = RxDone. We are polling for now, but this prepares
    // the radio for GPIO26 interrupt operation later.
    ESP_ERROR_CHECK(
        write_register(
            REG_DIO_MAPPING_1,
            0x00
        )
    );

    // Continuous receive mode.
    ESP_ERROR_CHECK(
        write_register(
            REG_OP_MODE,
            MODE_LONG_RANGE | MODE_RX_CONTINUOUS
        )
    );

    uint8_t op_mode = read_register(REG_OP_MODE);

    ESP_LOGI(TAG, "LoRa RX OpMode = 0x%02X", op_mode);

    if (op_mode != 0x85)
    {
        ESP_LOGE(TAG, "Failed to enter continuous RX mode");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LoRa receive-only mode active");

    return ESP_OK;
}


void lora_receive_service()
{
    constexpr uint8_t REG_FIFO = 0x00;
    constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0D;
    constexpr uint8_t REG_IRQ_FLAGS = 0x12;
    constexpr uint8_t REG_RX_NB_BYTES = 0x13;
    constexpr uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
    constexpr uint8_t REG_PKT_RSSI_VALUE = 0x1A;

    constexpr uint8_t IRQ_RX_DONE = 0x40;
    constexpr uint8_t IRQ_PAYLOAD_CRC_ERROR = 0x20;

    uint8_t irq_flags = read_register(REG_IRQ_FLAGS);

    // Nothing received.
    if ((irq_flags & IRQ_RX_DONE) == 0)
    {
        return;
    }

    // Clear the IRQ flags we just read.
    ESP_ERROR_CHECK(
        write_register(
            REG_IRQ_FLAGS,
            irq_flags
        )
    );

    if ((irq_flags & IRQ_PAYLOAD_CRC_ERROR) != 0)
    {
        ESP_LOGW(TAG, "LoRa packet rejected: hardware CRC error");
        return;
    }

    uint8_t packet_length = read_register(REG_RX_NB_BYTES);
    uint8_t fifo_address = read_register(REG_FIFO_RX_CURRENT_ADDR);

    ESP_ERROR_CHECK(
        write_register(
            REG_FIFO_ADDR_PTR,
            fifo_address
        )
    );

    if (packet_length == 0)
    {
        return;
    }

    // RoboLora's first byte is its own application payload length.
    uint8_t declared_length = read_register(REG_FIFO);

    uint8_t received_length = packet_length - 1;

    // RoboLora currently uses a 160 byte raw packet buffer.
    char message[160];

    if (received_length >= sizeof(message))
    {
        ESP_LOGW(
            TAG,
            "LoRa packet too large: %u bytes",
            received_length
        );

        // Drain the remaining FIFO bytes.
        for (uint8_t i = 0; i < received_length; i++)
        {
            read_register(REG_FIFO);
        }

        return;
    }

    for (uint8_t i = 0; i < received_length; i++)
    {
        message[i] = (char)read_register(REG_FIFO);
    }

    message[received_length] = '\0';

    if (declared_length != received_length)
    {
        ESP_LOGW(
            TAG,
            "LoRa length mismatch: declared=%u received=%u",
            declared_length,
            received_length
        );

        return;
    }

    // RoboLora uses the LF RSSI offset below 525 MHz.
    int rssi = (int)read_register(REG_PKT_RSSI_VALUE) - 164;

    RoboPacket packet = {};

    packet.source = PacketSource::LORA;
    packet.length = received_length;
    packet.rssi = rssi;

    memcpy(
        packet.data,
        message,
        received_length + 1
    );

    if (!packet_queue_send(packet))
    {
        ESP_LOGW(TAG, "LoRa packet dropped: packet queue full");
    }
}

esp_err_t lora_send(const char *data, uint16_t length)
{
    constexpr uint8_t REG_FIFO = 0x00;
    constexpr uint8_t REG_OP_MODE = 0x01;
    constexpr uint8_t REG_FIFO_ADDR_PTR = 0x0D;
    constexpr uint8_t REG_FIFO_TX_BASE_ADDR = 0x0E;
    constexpr uint8_t REG_IRQ_FLAGS = 0x12;
    constexpr uint8_t REG_PAYLOAD_LENGTH = 0x22;
    constexpr uint8_t REG_DIO_MAPPING_1 = 0x40;

    constexpr uint8_t MODE_LONG_RANGE = 0x80;
    constexpr uint8_t MODE_STANDBY = 0x01;
    constexpr uint8_t MODE_TX = 0x03;

    constexpr uint8_t IRQ_TX_DONE = 0x08;

    constexpr uint16_t MAX_MESSAGE_LENGTH = 159;
    constexpr TickType_t TX_TIMEOUT = pdMS_TO_TICKS(2000);

    if (data == nullptr)
    {
        ESP_LOGE(TAG, "LoRa TX failed: null data");
        return ESP_ERR_INVALID_ARG;
    }

    if (length == 0 || length > MAX_MESSAGE_LENGTH)
    {
        ESP_LOGE(
            TAG,
            "LoRa TX failed: invalid length %u",
            length
        );

        return ESP_ERR_INVALID_SIZE;
    }

    ESP_LOGI(
        TAG,
        "LoRa TX start len=%u",
        length
    );

    ESP_ERROR_CHECK(
        write_register(
            REG_OP_MODE,
            MODE_LONG_RANGE | MODE_STANDBY
        )
    );

    uint8_t tx_base =
        read_register(REG_FIFO_TX_BASE_ADDR);

    ESP_ERROR_CHECK(
        write_register(
            REG_FIFO_ADDR_PTR,
            tx_base
        )
    );

    /*
     * RoboLora framing:
     *
     * FIFO byte 0   = raw Robo message length
     * FIFO bytes 1+ = "$...*CRC"
     */
    ESP_ERROR_CHECK(
        write_register(
            REG_FIFO,
            static_cast<uint8_t>(length)
        )
    );

    for (uint16_t i = 0; i < length; i++)
    {
        ESP_ERROR_CHECK(
            write_register(
                REG_FIFO,
                static_cast<uint8_t>(data[i])
            )
        );
    }

    /*
     * SX127x payload includes the application length byte.
     */
    ESP_ERROR_CHECK(
        write_register(
            REG_PAYLOAD_LENGTH,
            static_cast<uint8_t>(length + 1)
        )
    );

    /*
     * Clear pending IRQ flags before starting TX.
     */
    ESP_ERROR_CHECK(
        write_register(
            REG_IRQ_FLAGS,
            0xFF
        )
    );

    /*
     * DIO0 = TxDone.
     *
     * We still poll RegIrqFlags, but keeping the DIO mapping
     * correct makes the radio state consistent.
     */
    ESP_ERROR_CHECK(
        write_register(
            REG_DIO_MAPPING_1,
            0x40
        )
    );

    TickType_t tx_start =
        xTaskGetTickCount();

    ESP_ERROR_CHECK(
        write_register(
            REG_OP_MODE,
            MODE_LONG_RANGE | MODE_TX
        )
    );

    while (true)
    {
        uint8_t irq_flags =
            read_register(REG_IRQ_FLAGS);

        if ((irq_flags & IRQ_TX_DONE) != 0)
        {
            ESP_ERROR_CHECK(
                write_register(
                    REG_IRQ_FLAGS,
                    IRQ_TX_DONE
                )
            );

            ESP_LOGI(TAG, "LoRa TX done");

            esp_err_t rx_result =
                lora_start_receive();

            if (rx_result != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "LoRa TX completed but RX restore failed"
                );

                return rx_result;
            }

            return ESP_OK;
        }

        TickType_t now =
            xTaskGetTickCount();

        if ((now - tx_start) >= TX_TIMEOUT)
        {
            ESP_LOGE(
                TAG,
                "LoRa TX timeout"
            );

            /*
             * Force radio out of TX state before restoring RX.
             */
            write_register(
                REG_OP_MODE,
                MODE_LONG_RANGE | MODE_STANDBY
            );

            write_register(
                REG_IRQ_FLAGS,
                0xFF
            );

            esp_err_t rx_result =
                lora_start_receive();

            if (rx_result != ESP_OK)
            {
                ESP_LOGE(
                    TAG,
                    "LoRa RX recovery failed after TX timeout"
                );
            }

            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}