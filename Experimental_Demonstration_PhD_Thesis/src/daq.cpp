#include "daq.h"
#include "experiment_config.h"

#include <NIDAQmx.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

// Macro para verificar errores de DAQ (patron original del proyecto).
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

// -----------------------------------------------------------------------------
// Adquisicion cruda desde la DAQ
// -----------------------------------------------------------------------------
int daqAcquire(int nSamples, int fSample, double* data) {
    int32       error = 0;
    TaskHandle  taskHandle = 0;
    int32       read = 0;
    char        errBuff[2048] = { '\0' };

    try {
        DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
        DAQmxErrChk(DAQmxCreateAIVoltageChan(
            taskHandle,
            cfg::DAQ_CHANNEL,   // canal de entrada
            "",
            DAQmx_Val_RSE,      // modo Single-Ended
            cfg::DAQ_V_MIN,
            cfg::DAQ_V_MAX,
            DAQmx_Val_Volts,
            NULL
        ));

        DAQmxErrChk(DAQmxCfgSampClkTiming(
            taskHandle,
            "",                 // reloj interno
            (float64)fSample,
            DAQmx_Val_Rising,
            DAQmx_Val_FiniteSamps,
            nSamples
        ));

        DAQmxErrChk(DAQmxStartTask(taskHandle));

        DAQmxErrChk(DAQmxReadAnalogF64(
            taskHandle,
            nSamples,
            200.0,              // timeout [s]
            DAQmx_Val_GroupByChannel,
            data,               // buffer de salida (double == float64)
            nSamples,
            &read,
            NULL
        ));

    Error:
        if (DAQmxFailed(error))
            DAQmxGetExtendedErrorInfo(errBuff, 2048);

        if (taskHandle != 0) {
            DAQmxStopTask(taskHandle);
            DAQmxClearTask(taskHandle);
        }

        if (DAQmxFailed(error)) {
            std::cerr << "Error DAQmx: " << errBuff << std::endl;
            return 0;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "Excepcion estandar en daqAcquire: " << ex.what() << std::endl;
        return 0;
    }

    return read;
}

// -----------------------------------------------------------------------------
// Adquisicion + resumen estadistico
// -----------------------------------------------------------------------------
DaqStats daqAcquireStats(int nSamples, int fSample) {
    DaqStats out;

    std::vector<double> buf(nSamples);
    int read = daqAcquire(nSamples, fSample, buf.data());
    if (read <= 0) {
        out.ok = false;
        return out;
    }

    buf.resize(read);
    out.n  = read;
    out.ok = true;

    // Media
    double sum = 0.0;
    for (double v : buf) sum += v;
    out.mean = sum / read;

    // Mediana (sobre copia ordenada)
    std::vector<double> sorted(buf);
    std::sort(sorted.begin(), sorted.end());
    if (read % 2 == 0)
        out.median = (sorted[read / 2 - 1] + sorted[read / 2]) / 2.0;
    else
        out.median = sorted[read / 2];

    // Desviacion estandar muestral (n-1). Con read==1 -> 0.
    if (read > 1) {
        double acc = 0.0;
        for (double v : buf) {
            double d = v - out.mean;
            acc += d * d;
        }
        out.std = std::sqrt(acc / (read - 1));
    } else {
        out.std = 0.0;
    }

    return out;
}
