#ifndef __sampler_base_h__
#define __sampler_base_h__

#include <Arduino.h>
#include <driver/i2s.h>

/**
 * Base Class for both the ADC and I2S sampler
 **/
template<size_t WindowSize> class i2s_sampler {
   private:
    // double buffered audio - the processing task can be running against one buffer while we fill
    // the other buffer
    int16_t m_audioBuffer1[WindowSize];
    int16_t m_audioBuffer2[WindowSize];
    // current position in the audio buffer
    int32_t m_audioBufferPos = 0;
    // current audio buffer
    int16_t *m_currentAudioBuffer;
    // buffer containing samples that have been captured already
    int16_t *m_capturedAudioBuffer;
    // size of the audio buffers in bytes
    int32_t m_bufferSizeInBytes;
    // size of the audio buffer in samples
    int32_t m_bufferSizeInSamples;
    // I2S reader task
    TaskHandle_t m_readerTaskHandle;
    // processor task
    TaskHandle_t m_processorTaskHandle;
    // i2s reader queue
    QueueHandle_t m_i2sQueue;
    // i2s port
    i2s_port_t m_i2sPort;
    // i2s pins
    i2s_pin_config_t m_i2sPins;

   protected:
    void push_back(int16_t sample) {
        // add the sample to the current audio buffer
        m_currentAudioBuffer[m_audioBufferPos] = sample;
        m_audioBufferPos++;
        // have we filled the buffer with data?
        if (m_audioBufferPos == m_bufferSizeInSamples) {
            // swap to the other buffer
            std::swap(m_currentAudioBuffer, m_capturedAudioBuffer);
            // reset the buffer position
            m_audioBufferPos = 0;
            // tell the writer task to save the data
            xTaskNotify(m_processorTaskHandle, 1, eSetValueWithOverwrite);
        }
    }

   public:
    static constexpr const size_t window_size = WindowSize;
    size_t size_bytes() const {
        return m_bufferSizeInBytes;
    };
    const int16_t *buffer() const {
        return m_capturedAudioBuffer;
    }
    void initialize(i2s_port_t i2sPort,
                    const i2s_pin_config_t &i2sPins,
                    const i2s_config_t &i2sConfig,
                    TaskHandle_t processorTaskHandle) {
        m_i2sPort = i2sPort;
        m_processorTaskHandle = processorTaskHandle;
        m_bufferSizeInSamples = window_size;
        m_bufferSizeInBytes = sizeof(m_audioBuffer1);

        m_currentAudioBuffer = m_audioBuffer1;
        m_capturedAudioBuffer = m_audioBuffer2;

        // install and start i2s driver
        i2s_driver_install(m_i2sPort, &i2sConfig, 4, &m_i2sQueue);
        i2s_set_pin(i2sPort, &i2sPins);
        i2s_set_clk(i2sPort, i2sConfig.sample_rate, i2sConfig.bits_per_sample, I2S_CHANNEL_MONO);
        // i2s_set_pdm_rx_down_sample(i2sPort, I2S_PDM_DSR_8S);
        // start a task to read samples from the ADC
        TaskHandle_t readerTaskHandle;
        xTaskCreatePinnedToCore(
            i2s_reader_task, "i2s Reader Task", 8192, this, 1, &readerTaskHandle, 0);
    }

    static void i2s_reader_task(void *param) {
        i2s_sampler *sampler = (i2s_sampler *)param;
        while (true) {
            // wait for some data to arrive on the queue
            i2s_event_t evt;
            if (xQueueReceive(sampler->m_i2sQueue, &evt, portMAX_DELAY) != pdPASS) {
                continue;
            }

            if (evt.type != I2S_EVENT_RX_DONE) {
                continue;
            }

            size_t bytesRead = 0;
            do {
                // read data from the I2S peripheral
                int16_t i2sData[2048];
                // read from i2s
                i2s_read(sampler->m_i2sPort, i2sData, 4096, &bytesRead, 10);
                for (int i = 0; i < bytesRead / 2; i += 4) {
                    sampler->push_back(
                        (i2sData[i] + i2sData[i + 1] + i2sData[i + 2] + i2sData[i + 3]) / 4);
                }
            } while (bytesRead > 0);
        }
    }
};

#endif
