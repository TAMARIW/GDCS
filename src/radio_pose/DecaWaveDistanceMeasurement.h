#ifndef DECA_WAVE_DISTANCE_MEASUREMENT_H
#define DECA_WAVE_DISTANCE_MEASUREMENT_H

#include "rodos.h"
#include "matlib.h"

static Application module01("Decawave Distance Measurement Node", 2001);

#define FILTER_SIZE 4 // Anzahl Werte die der Filter aufnehmen kann
#define THRESHOLD 1 // Werte, die mehr als 2 Standardabweichungen vom Mittelwert entfernt sind, werden als Ausreißer betrachtet
#define SENSOR_ERROR 0.1f

template<typename T>
class MovingAverageFilter {
private:
    T buffer[FILTER_SIZE];
    T sum;
    int index;
    int count;

    T calculateMean() const {
        return sum / (count < FILTER_SIZE ? count : FILTER_SIZE);
    }

    T calculateStandardDeviation(T mean) const {
        T variance = 0;
        int validCount = count < FILTER_SIZE ? count : FILTER_SIZE;
        for (int i = 0; i < validCount; i++) {
            variance += (buffer[i] - mean) * (buffer[i] - mean);
        }
        return sqrt(variance / validCount);
    }

public:
    MovingAverageFilter() : sum(0), index(0), count(0) {
        for (int i = 0; i < FILTER_SIZE; ++i) {
            buffer[i] = 0;
        }
    }

    void add(T value) {
        if (value < 0) // check whether value is negative then apply zero instead (no calibration necessary because sensors precision is well known!)
            value = 0.0;

        if (count < FILTER_SIZE) {
            buffer[count++] = value;
            sum += value;
        } else {
            sum -= buffer[index];
            buffer[index] = value;
            sum += value;
            index = (index + 1) % FILTER_SIZE;
        }
    }

    T getAverage() const {
        if (count == 0) return 0;

        int validCount = count < FILTER_SIZE ? count : FILTER_SIZE;
        T mean = calculateMean();
        T stddev = calculateStandardDeviation(mean);
        
        T adjustedSum = 0;
        int adjustedCount = 0;
        for (int i = 0; i < validCount; ++i) {
            if (abs(buffer[i] - mean) <= THRESHOLD * stddev) {
                adjustedSum += buffer[i];
                adjustedCount++;
            }
        }

        if (adjustedCount == 0) return mean; // Wenn alle Werte Ausreißer sind, geben wir den bisherigen Mittelwert zurück

        return adjustedSum / adjustedCount;
    }
};

#include <cmath>

#define ALPHA 0.3 // Glättungsfaktor für den EMA (dieser Wert kann angepasst werden)

template<typename T>
class ExponentialMovingAverageFilter {
private:
    T buffer[FILTER_SIZE];
    T ema;
    bool initialized;
    int index;
    int count;

public:
    ExponentialMovingAverageFilter() : ema(0), initialized(false), index(0), count(0) {
        for (int i = 0; i < FILTER_SIZE; ++i) {
            buffer[i] = 0;
        }
    }

    void add(T value) {
        if (value < 0) // Check whether value is negative then apply zero instead (no calibration necessary because sensor's precision is well known!)
            value = 0.0;

        if (!initialized) {
            ema = value;
            initialized = true;
        } else {
            ema = ALPHA * value + (1 - ALPHA) * ema;
        }

        if (count < FILTER_SIZE) {
            buffer[count++] = value;
        } else {
            buffer[index] = value;
            index = (index + 1) % FILTER_SIZE;
        }
    }

    T getAverage() const {
        if (count == 0) return 0;

        return ema;
    }
};

class DecaWaveDistanceMeasurement : public StaticThread<>, public IOEventReceiver {
   public:
    DecaWaveDistanceMeasurement(const char* name, uint8_t _spi_num, int32_t _redNodeId, int32_t _sendNodeId, int32_t _waitoffset, CommBuffer<float>& _cbuf) : StaticThread<>(name, 1000), spi_num(_spi_num), redNodeId(_redNodeId), sendNodeId(_sendNodeId), waitoffset(_waitoffset), cbuf(_cbuf) {}
    void init();
    void run();

    void send_dist(float distance, uint8_t destId);

    void start_twr(uint8_t destId);
    void send_response(uint8_t destId);
    void send_final(uint8_t destId);

    void receiveMessages();

    float calculate_distance();

    uint64_t poll_tx_ts;   // DS-TWR poll transmit timestamp
    uint64_t poll_rx_ts;   // DS-TWR poll receive timestamp
    uint64_t resp_tx_ts;   // DS-TWR response transmit timestamp
    uint64_t resp_rx_ts;   // DS-TWR response receive timestamps
    uint64_t final_tx_ts;  // DS-TWR final transmit timestamp
    uint64_t final_rx_ts;  // DS-TWR final receive timestamp

    int64_t nextTime2Measure;  // timestamp when the next message should be send (RODOS time)

    uint8_t redNodeId;  // unique ID of each node
    uint8_t sendNodeId;
    int32_t waitoffset;

    uint8_t spi_num; // number of used spi controller (either 0 or 1)

    //MovingAverageFilter<float> mfilter;
    ExponentialMovingAverageFilter<float> efilter;

    CommBuffer<float>& cbuf;
};

#endif /* VaMEx_DWN_H_ */
