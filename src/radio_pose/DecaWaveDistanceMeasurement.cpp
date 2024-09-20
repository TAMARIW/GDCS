#include "DecaWaveDistanceMeasurement.h"

#include <stdio.h>
#define __cminpack_double__
#include "cminpack/cminpack.h"
#include "decaWaveModule.h"
#include "decadriver/deca_device_api.h"
#include "decadriver/deca_regs.h"
#include "defines_resp.h"
#include "hal.h"
#include "matlib.h"


// Set this macro to either 0 or 1 depending on the board number.
#define BOARD_ID 1

#if BOARD_ID == 0
#define SENSOR1_ID1 1
#define SENSOR1_ID2 3
#define SENSOR2_ID1 5
#define SENSOR2_ID2 7
#define DEST_SENSOR1_ID1 2
#define DEST_SENSOR1_ID2 6
#define DEST_SENSOR2_ID1 4
#define DEST_SENSOR2_ID2 8
#else
#define SENSOR1_ID1 2
#define SENSOR1_ID2 6
#define SENSOR2_ID1 4
#define SENSOR2_ID2 8
#define DEST_SENSOR1_ID1 1
#define DEST_SENSOR1_ID2 3
#define DEST_SENSOR2_ID1 5
#define DEST_SENSOR2_ID2 7
#endif



extern HAL_UART uart_stdout;

extern HAL_GPIO UWBirq1;
extern HAL_GPIO UWBirq2;
HAL_GPIO ledb(GPIO_060);
HAL_GPIO ledr(GPIO_061);
HAL_GPIO ledo(GPIO_062);
HAL_GPIO ledg(GPIO_063);
HAL_GPIO led(GPIO_078);

// Store the measured distance values coming from the sensors (one combuffer for each distance).
CommBuffer<float> com_dist_0_to_0, com_dist_1_to_0, com_dist_0_to_1, com_dist_1_to_1;

Topic<Vector3D_F> uwbPositionTopic(-1, "UWB Position");

void DecaWaveDistanceMeasurement::init() {
    ledb.init(1, 1, 0);
    ledr.init(1, 1, 0);
    ledo.init(1, 1, 0);
    ledg.init(1, 1, 0);
    led.init(1, 1, 1);
}

struct lsData {
    Vector3D uwbPosition0_0; // position of own board (sensor 1)
    Vector3D uwbPosition0_1; // position of own board (sensor 2)
    Quaternion sat1Rotation; // rotation of 2nd board
    double dists[4]; // distances / residuals
};

int fcn(void *p, int m, int n, const double *x, double *fvec, int iflag) {
    // Calculation of residuals.
    lsData *data = (lsData *)p;

    Vector3D uwbPosition1_0 = Vector3D(x[0], x[1], x[2]); // estimated position of sensor 1 of board 2.
    Vector3D uwbPosition1_1 = uwbPosition1_0 + data->sat1Rotation * Vector3D(0.3, 0.0, 0.0); // estimated position of sensor 2 of board 2

    Vector3D vecDifference = data->uwbPosition0_0 - uwbPosition1_0;
    fvec[0] = data->dists[0] - (vecDifference.x * vecDifference.x + vecDifference.y * vecDifference.y + vecDifference.z * vecDifference.z);

    vecDifference = data->uwbPosition0_0 - uwbPosition1_1;
    fvec[1] = data->dists[1] - (vecDifference.x * vecDifference.x + vecDifference.y * vecDifference.y + vecDifference.z * vecDifference.z);

    vecDifference = data->uwbPosition0_1 - uwbPosition1_0;
    fvec[2] = data->dists[2] - (vecDifference.x * vecDifference.x + vecDifference.y * vecDifference.y + vecDifference.z * vecDifference.z);

    vecDifference = data->uwbPosition0_1 - uwbPosition1_1;
    fvec[3] = data->dists[3] - (vecDifference.x * vecDifference.x + vecDifference.y * vecDifference.y + vecDifference.z * vecDifference.z);

    return 0;
}

/**
 * IMPORTANT: Adjust DEFAULT_STACKSIZE in platform-parameter.h (e.g. in rodos/src/bare-metal/stm32f4/platform-parameter/skith/platform-parameter.h)
 * to at least 2600 bytes (default is 2000 bytes) to avoid stack overflow. - DONE
 */

Vector3D findOrthogonal(const Vector3D& vec) {
    // If vec is not parallel to the x-axis, use (1, 0, 0)
    if (fabs(vec.x) <= fabs(vec.y) && fabs(vec.x) <= fabs(vec.z)) {
        return vec.cross(Vector3D(1.0, 0.0, 0.0)).normalize();
    } 
    // If vec is not parallel to the y-axis, use (0, 1, 0)
    else if (fabs(vec.y) <= fabs(vec.x) && fabs(vec.y) <= fabs(vec.z)) {
        return vec.cross(Vector3D(0.0, 1.0, 0.0)).normalize();
    } 
    // Otherwise, vec is not parallel to the z-axis, use (0, 0, 1)
    else {
        return vec.cross(Vector3D(0.0, 0.0, 1.0)).normalize();
    }
}



Quaternion calculateRotation(Vector3D oldPos0, Vector3D oldPos1, Vector3D newPos0, Vector3D newPos1) {
    Vector3D initialDirection = (oldPos0 - oldPos1).normalize();
    Vector3D newDirection = (newPos0 - newPos1).normalize();

    double dot = initialDirection.dot(newDirection);
    Vector3D cross = initialDirection.cross(newDirection);

    Quaternion q;

    if (dot >= 1.0) {
        return Quaternion(1.0, 0.0, 0.0, 0.0); // Identity quaternion
    } else if (dot <= -1.0) {
        Vector3D arbitraryAxis = findOrthogonal(initialDirection);
        q = Quaternion(0.0, arbitraryAxis.x, arbitraryAxis.y, arbitraryAxis.z); // 180 degree rotation around the orthogonal axis
    } else {
        double w = sqrt(1.0 + dot) * 0.5;
        double scale = sqrt(1.0 - w * w);
        q = Quaternion(w, cross.x * scale, cross.y * scale, cross.z * scale);
    }

    // Normalize the quaternion to prevent numerical drift
    return q.normalize();
}


void DecaWaveDistanceMeasurement::run() {
    int64_t startTime, endTime;

    //suspendCallerUntil(END_OF_TIME);
    
    PRINTF("RedNodId:%d, SendNodeId:%d dw %d\n", redNodeId, sendNodeId, spi_num);
    init_decaWaveModule(spi_num, &config);
    PRINTF("Init done\n");

    nextTime2Measure = NOW();

    // set TX Power
    dwt_setsmarttxpower(spi_num, 0);
    dwt_write32bitreg(spi_num, TX_POWER_ID, 0x009A9A00);

    // main loop
    while (1) {
        //ledb.setPins(~ledb.readPins());

        if (nextTime2Measure <= NOW()) {
            start_twr(sendNodeId);
            nextTime2Measure = NOW() + 500 * MILLISECONDS + waitoffset * MILLISECONDS; // vorher: 250
        }

        // enable receiver
        uint16_t states_reg = dwt_read16bitoffsetreg(spi_num, SYS_STATE_ID, 2);
        if (!(states_reg & 0x04)) dwt_rxenable(spi_num, 0);  // check if receiver is not already enabled

        //suspendCallerUntil(nextTime2Measure - 10*MILLISECONDS);
        // Wait for new measurement depending on board sensor.
        if (spi_num == 0) {
            UWBirq1.suspendUntilDataReady(nextTime2Measure);
            UWBirq1.resetInterruptEventStatus();
        } else {
            UWBirq2.suspendUntilDataReady(nextTime2Measure);
            UWBirq2.resetInterruptEventStatus();
        }

        PRINTF("Received data. Time: %f, node: %d\n", SECONDS_NOW(), redNodeId);

        //
        //yield();

        receiveMessages();


    }
}
//DecaWaveDistanceMeasurement DecaWaveDistanceMeasurement_Thread0("Decawave Distance Measurement Node Thread - SPI0", 0, SENSOR1_ID1, DEST_SENSOR1_ID1, 0, com_dist_0_to_0);
//DecaWaveDistanceMeasurement DecaWaveDistanceMeasurement_Thread11("Decawave Distance Measurement Node Thread 2 - SPI1", 1, SENSOR2_ID2, DEST_SENSOR2_ID2, 200, com_dist_1_to_1);
//DecaWaveDistanceMeasurement DecaWaveDistanceMeasurement_Thread01("Decawave Distance Measurement Node Thread 2 - SPI0", 0, SENSOR1_ID2, DEST_SENSOR2_ID1, 400, com_dist_0_to_1);
//DecaWaveDistanceMeasurement DecaWaveDistanceMeasurement_Thread1("Decawave Distance Measurement Node Thread - SPI1", 1, SENSOR2_ID1, DEST_SENSOR1_ID2, 600, com_dist_1_to_0);


void DecaWaveDistanceMeasurement::send_dist(float distance, uint8_t destId) {
    memcpy(&tx_distance_msg[DISTANCE_MSG_DISTANCE_IDX], &distance, sizeof(distance));

    uwb_write(spi_num, redNodeId, destId, tx_distance_msg, sizeof(tx_distance_msg));
}

void DecaWaveDistanceMeasurement::start_twr(uint8_t destId) {
    // initiatie ds twr process
    uwb_write(spi_num, redNodeId, destId, tx_poll_msg, sizeof(tx_poll_msg));
    poll_tx_ts = get_tx_timestamp_u64(spi_num);
}

void DecaWaveDistanceMeasurement::send_response(uint8_t destId) {
    uwb_write(spi_num, redNodeId, destId, tx_resp_msg, sizeof(tx_resp_msg));

    resp_tx_ts = get_tx_timestamp_u64(spi_num);
}

void DecaWaveDistanceMeasurement::send_final(uint8_t destId) {
    // compute final message transmission time
    uint64_t final_tx_time = get_systemtime_deca(spi_num) + RESP_RX_TO_FINAL_TX_DLY_DTU_U64;
    dwt_setdelayedtrxtime(spi_num, final_tx_time >> 8);
    // final TX timestamp is the transmission time we programmed plus the TX antenna delay
    uint64_t final_tx_ts = final_tx_time + TX_ANT_DLY;
    // write all timestamps in the final message
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
    final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

    uwb_write(spi_num, redNodeId, destId, tx_final_msg, sizeof(tx_final_msg), DWT_START_TX_DELAYED);

}

float DecaWaveDistanceMeasurement::calculate_distance() {
    double dist;
    double Ra, Rb, Da, Db;
    double tof_dtu;

    // retrieve final reception timestamps.
    final_rx_ts = get_rx_timestamp_u64(spi_num);

    // get timestamps embedded in the final message.
    final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
    final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

    constexpr uint64_t MAX_VALUE_40_BIT = 0xFFFFFFFFFF;

    if (resp_rx_ts < poll_tx_ts) {
        resp_rx_ts += MAX_VALUE_40_BIT;
        final_tx_ts += MAX_VALUE_40_BIT;
    }
    if (final_tx_ts < resp_rx_ts) {
        final_tx_ts += MAX_VALUE_40_BIT;
    }
    if (resp_tx_ts < poll_rx_ts) {
        resp_tx_ts += MAX_VALUE_40_BIT;
        final_rx_ts += MAX_VALUE_40_BIT;
    }
    if (final_rx_ts < resp_tx_ts) {
        final_rx_ts += MAX_VALUE_40_BIT;
    }

    // compute time of flight.
    Ra = (double)(resp_rx_ts - poll_tx_ts);
    Rb = (double)(final_rx_ts - resp_tx_ts);
    Da = (double)(final_tx_ts - resp_rx_ts);
    Db = (double)(resp_tx_ts - poll_rx_ts);
    tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

    double tof = tof_dtu * DWT_TO_SECONDS;
    dist = tof * SPEED_OF_LIGHT;

    return (float)dist;
}

void DecaWaveDistanceMeasurement::receiveMessages() {
    if (uwb_read(spi_num, rx_buffer)) {
        //led.setPins(~led.readPins());
        uint8_t sourceId = rx_buffer[ALL_MSG_SOURCE_ID_IDX];
        uint8_t destId = rx_buffer[ALL_MSG_DEST_ID_IDX];
        uint8_t msgSN = rx_buffer[ALL_MSG_SN_IDX];

        switch (rx_buffer[ALL_MSG_TYPE_IDX]) {
            case MSG_TYPE_POLL: {  // TWR poll message
                if (destId == redNodeId) {
                    // save poll reception timestamp.
                    poll_rx_ts = get_rx_timestamp_u64(spi_num);

                    send_response(sourceId);
                }
                break;
            }
            case MSG_TYPE_RESP: {           // TWR resp message
                if (destId == redNodeId) {  // check if message is for this node
                    resp_rx_ts = get_rx_timestamp_u64(spi_num);
                    send_final(sourceId);
                }
                break;
            }
            case MSG_TYPE_FINAL: {  // TWR final message
                if (destId == redNodeId) {
                    float distance = calculate_distance();

                    send_dist(distance, sourceId);
}
                break;
            }
            case MSG_TYPE_DISTANCE: {  // distance message
                if (redNodeId == destId) {
                    float distance;
                    memcpy(&distance, &rx_buffer[DISTANCE_MSG_DISTANCE_IDX], sizeof(distance));

                    mfilter.add(distance);
                    cbuf.put(mfilter.getAverage());
                    
                    PRINTF("%d_%d: distance: %f m\n", redNodeId, spi_num, distance);
                }
                break;
            }
            default:
                PRINTF("Received unidentified message type at Node %d, sourceId %d, destId %d, msgSN %d, %d\n", redNodeId, sourceId, destId, msgSN, spi_num);
                break;
        }
    }
}

// Calculates the estimated position for the other board relative to us.
class EstimatedPositionCalculator : StaticThread<3000>, IOEventReceiver {
    Vector3D posInSat0 = Vector3D(0.0, 0.0, 0.0); // sensor 1 of our board (marks 0,0,0)
    Vector3D posInSat1 = Vector3D(0.3, 0.0, 0.0); // sensor 2 of our board within the boards coordinate system

    Vector3D satPosition0 = Vector3D(0.0, 0.0, 0.0); // our board
    Vector3D satPosition1 = Vector3D(0.0, 0.0, 0.0); // other boards initial coordinates (set them to 0,0,0)

    //Quaternion sat1Rotation = Quaternion(YPR(-M_PI, 0.0 / 180.0 * M_PI, 0.0 / 180.0 * M_PI));
    Quaternion sat1Rotation = Quaternion(1.0, 0.0, 0.0, 0.0); // set initially rotation parallel to board 1

    Vector3D uwbPosition0_0; // pos of sensor 1 of our board
    Vector3D uwbPosition0_1; // pos of sensor 2 of our board
    Vector3D uwbPosition1_0; // pos of sensor 1 of the other board
    Vector3D uwbPosition1_1; // pos of sensor 2 of the other board

    lsData data; // contains our position and rotation of the other board as well as measured distances
public:
    unsigned int cnt;

    EstimatedPositionCalculator() : StaticThread<3000>("EstimatedPositionCalculator", 1000) {}

    void init() {
        // Make initially calculations.
        uwbPosition0_0 = satPosition0 + posInSat0;
        uwbPosition0_1 = satPosition0 + posInSat1;
        uwbPosition1_0 = satPosition1 + sat1Rotation * posInSat0;
        uwbPosition1_1 = satPosition1 + sat1Rotation * posInSat1;

        data.uwbPosition0_0 = uwbPosition0_0; // init position of sensor 1 of board 2
        data.uwbPosition0_1 = uwbPosition0_1; // init position of sensor 2 of board 2
        data.sat1Rotation = sat1Rotation; // init rotation

        PRINTF("EstimatedPositionCalculator started\n");

        cnt = 0;

        return;
    }

    void run() {
        // print initially position of both boards and sensors.
        PRINTF("UWB0_0: %f %f %f\n", uwbPosition0_0.x, uwbPosition0_0.y, uwbPosition0_0.z);
        PRINTF("UWB0_1: %f %f %f\n", uwbPosition0_1.x, uwbPosition0_1.y, uwbPosition0_1.z);
        PRINTF("UWB1_0: %f %f %f\n", uwbPosition1_0.x, uwbPosition1_0.y, uwbPosition1_0.z);
        PRINTF("UWB1_1: %f %f %f\n", uwbPosition1_1.x, uwbPosition1_1.y, uwbPosition1_1.z);

        while (1)
        {
            // get latest measurements from Commbuffers.
            float dist_measurements[4];
            com_dist_0_to_0.get(dist_measurements[0]);
            com_dist_1_to_0.get(dist_measurements[1]);
            com_dist_0_to_1.get(dist_measurements[2]);
            com_dist_1_to_1.get(dist_measurements[3]);

            // DEBUG OUTPUT: prints all distances.
            //PRINTF("dist: %.2f, %.2f, %.2f, %.2f\n", dist_measurements[0], dist_measurements[1], dist_measurements[2], dist_measurements[3]);

            // Calculate four residuals.
            const double dist[4] = {
                dist_measurements[0] * dist_measurements[0],
                dist_measurements[1] * dist_measurements[1],
                dist_measurements[2] * dist_measurements[2],
                dist_measurements[3] * dist_measurements[3],
            };

            /*double dist[4] = {
                (uwbPosition0_0 - uwbPosition1_0).getLen() * (uwbPosition0_0 - uwbPosition1_0).getLen(),
                (uwbPosition0_0 - uwbPosition1_1).getLen() * (uwbPosition0_0 - uwbPosition1_1).getLen(),
                (uwbPosition0_1 - uwbPosition1_0).getLen() * (uwbPosition0_1 - uwbPosition1_0).getLen(),
                (uwbPosition0_1 - uwbPosition1_1).getLen() * (uwbPosition0_1 - uwbPosition1_1).getLen()
            };*/

            // Should be correct!
            data.dists[0] = dist[0]; /* uwbPosition0_0 to uwbPosition1_0 */
            data.dists[1] = dist[2]; /* uwbPosition0_0 to uwbPosition1_1 */
            data.dists[2] = dist[1]; /* uwbPosition0_1 to uwbPosition1_0 */
            data.dists[3] = dist[3]; /* uwbPosition0_1 to uwbPosition1_1 */

            constexpr uint8_t m = 4; // Number of residuals (measured distances)
            constexpr uint8_t n = 3; // Number of variables (x, y, z)

            // Variables for optimization function.
            int i, j, maxfev, mode, nprint, info, info2, nfev, ldfjac;
            int ipvt[n];
            double ftol, xtol, gtol, epsfcn, factor, fnorm;
            double x0[n], x1[n], fvec[m], diag[n], fjac[m * n], qtf[n], wa1[n], wa2[n], wa3[n], wa4[m];
            int k;
            ldfjac = m;

            ftol = 1e-8; // termination occurs when both the actual and predicted relative reductions in the sum of squares are at most ftol
            xtol = 1e-8; // termination occurs when the relative error between two consecutive iterates is at most xtol
            gtol = 1e-8; // termination occurs when the cosine of the angle between fvec and any column of the jacobian is at most gtol
            maxfev = 1000; // termination occurs when number of calls to fcn is at least maxfev
            epsfcn = 0.1; // determines suitable step length for forward-difference approximation
            mode = 1;
            factor = 5; // positive input variable used in determining the initial step bound
            nprint = 1; // enables controlled printing of iterates

            // Assigning current coordinates to optimisation variables.
            x0[0] = uwbPosition1_0.x;
            x0[1] = uwbPosition1_0.y;
            x0[2] = uwbPosition1_0.z;

//            x1[0] = uwbPosition1_1.x;
//            x1[1] = uwbPosition1_1.y;
//            x1[2] = uwbPosition1_1.z;

            // estimate position of x0 (sensor 1 of board 2)
            info = __cminpack_func__(lmdif)(fcn, &data, m, n, x0, fvec, ftol, xtol, gtol, maxfev, epsfcn, diag, mode, factor, nprint, &nfev, fjac, ldfjac, ipvt, qtf, wa1, wa2, wa3, wa4);

            // result is only valid if [1,4] ^ [6,8]:
            if (info == 1 || info == 2 || info == 3 || info == 4 || info == 6 || info == 7 || info == 8) {
                const float vec_val = sqrt(x0[0] * x0[0] + x0[1] * x0[1] + x0[2] * x0[2]);

                // Glättefilter (testweise)
//                const double alpha = 0.35;
//                double x0_filtered[3] = {alpha * x0[0] + (1 - alpha) * uwbPosition1_0.x, alpha * x0[1] + (1 - alpha) * uwbPosition1_0.y, alpha * x0[2] + (1 - alpha) * uwbPosition1_0.z};
                
                //PRINTF("%i [%.1f, %.1f, %.1f]\t%.1f\n", info, x0[0], x0[1], x0[2], vec_val); // print estimated position of sensor 1
//                PRINTF("%i [%.1f, %.1f, %.1f]\t%.1f\n", info, x0_filtered[0], x0_filtered[1], x0_filtered[2], vec_val); // print estimated position of sensor 1

                // Calculate rotation based on previous and estimated position.
                //data.sat1Rotation = calculateRotation(uwbPosition1_0, uwbPosition1_1, x0, x1);
                data.sat1Rotation = sat1Rotation;


                // Update estimated position.
                //uwbPosition1_0 = x0_filtered;
                uwbPosition1_0 = x0;
                uwbPosition1_1 = x0 + sat1Rotation * posInSat1;

                // Publish estimated position.
                Vector3D_F position = Vector3D_F(uwbPosition1_0.x, uwbPosition1_0.y, uwbPosition1_0.z);
                uwbPositionTopic.publish(position);

                //PRINTF("%i [%.1f, %.1f, %.1f][%.1f, %.1f, %.1f] %.1f\n", info, x0[0], x0[1], x0[2], uwbPosition1_1.x, uwbPosition1_1.y, uwbPosition1_1.z, vec_val); // print estimated position of sensor 1

            } else {
                PRINTF("No solution found. Info: %d\n", info);
            }

            suspendCallerUntil(NOW() + 500 * MILLISECONDS);
        }
        
    }
} EstimatedPositionCalculatorThread0;