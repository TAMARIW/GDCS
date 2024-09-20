#ifndef DECA_WAVE_DISTANCE_MEASUREMENT_H
#define DECA_WAVE_DISTANCE_MEASUREMENT_H

#include "rodos.h"
#include "matlib.h"

static Application module01("Decawave Distance Measurement Node", 2001);

#define FILTER_SIZE 10 // Anzahl der Werte im Filter
#define THRESHOLD 2   // Schwellenwert für Ausreißer in Standardabweichungen

template<typename T>
class MovingAverageFilter {
private:
    T buffer[FILTER_SIZE];
    T sum;
    int index;
    int count;

    T calculateMean() const {
        // Berechnet den Mittelwert der Werte im Puffer
        int validCount = count < FILTER_SIZE ? count : FILTER_SIZE;
        return (validCount > 0) ? (sum / validCount) : 0;
    }

    T calculateStandardDeviation(T mean) const {
        // Berechnet die Standardabweichung der Werte im Puffer
        T variance = 0;
        int validCount = count < FILTER_SIZE ? count : FILTER_SIZE;
        if (validCount <= 1) return 0; // Vermeidet Division durch Null oder zu kleine Werte

        for (int i = 0; i < validCount; i++) {
            variance += (buffer[i] - mean) * (buffer[i] - mean);
        }
        return sqrt(variance / validCount);
    }

public:
    MovingAverageFilter() : sum(0), index(0), count(0) {
        for (int i = 0; i < FILTER_SIZE; ++i) {
            buffer[i] = 0; // Initialisiert den Puffer mit Nullwerten
        }
    }

    void add(T value) {
        if (value < 0)
            value = 0.0; // Negative Werte werden durch Null ersetzt

        // Wenn der Filter leer ist, initialisiere ihn vollständig mit dem ersten Wert
        if (count == 0) {
            for (int i = 0; i < FILTER_SIZE; ++i) {
                buffer[i] = value;
            }
            sum = value * FILTER_SIZE;
            count = FILTER_SIZE;
            index = 0;
        } else if (count < FILTER_SIZE) {
            // Normale Aufnahme in den Puffer bei nicht vollständiger Füllung
            buffer[count++] = value;
            sum += value;
        } else {
            // Zyklische Aufnahme neuer Werte bei vollem Puffer
            sum -= buffer[index];
            buffer[index] = value;
            sum += value;
            index = (index + 1) % FILTER_SIZE; // Kreisförmiges Pufferverhalten
        }
    }

    T getAverage() const {
        if (count == 0) return 0; // Keine Werte im Puffer

        T mean = calculateMean();
        T stddev = calculateStandardDeviation(mean);

        T adjustedSum = 0;
        int adjustedCount = 0;

        // Berechnet den angepassten Durchschnitt ohne Ausreißer
        int validCount = count < FILTER_SIZE ? count : FILTER_SIZE;
        for (int i = 0; i < validCount; ++i) {
            if (abs(buffer[i] - mean) <= THRESHOLD * stddev) {
                adjustedSum += buffer[i];
                adjustedCount++;
            }
        }

        // Rückgabe des angepassten Durchschnitts oder des bisherigen Mittelwerts
        return adjustedCount > 0 ? (adjustedSum / adjustedCount) : mean;
    }
};



class DecaWaveDistanceMeasurement : public StaticThread<3000>, public IOEventReceiver {
   public:
    DecaWaveDistanceMeasurement(const char* name, uint8_t _spi_num, int32_t _redNodeId, int32_t _sendNodeId, int32_t _waitoffset, CommBuffer<float>& _cbuf) : StaticThread<3000>(name, 2000), spi_num(_spi_num), redNodeId(_redNodeId), sendNodeId(_sendNodeId), waitoffset(_waitoffset), cbuf(_cbuf) {}
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
    uint8_t sendNodeId; // ID used to identify the correct recipient
    int32_t waitoffset;

    uint8_t spi_num; // number of used spi controller (either 0 or 1)

    MovingAverageFilter<float> mfilter; // stores all measured distances and calculates an average value

    CommBuffer<float>& cbuf; // stores the mean values from the filter
};

#endif /* VaMEx_DWN_H_ */
