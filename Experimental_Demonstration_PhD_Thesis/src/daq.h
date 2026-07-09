#pragma once
#ifndef DAQ_H
#define DAQ_H

// =============================================================================
// daq.h
// Adquisicion de la DAQ (NI-DAQmx) y resumen estadistico.
// Unifica la logica que antes estaba duplicada en exp_cone_mapping.cpp y exp_3D.cpp.
// Se usan tipos estandar (double/int) para no acoplar la cabecera a NIDAQmx.h.
// =============================================================================

// Resumen de una adquisicion. Sustituye el guardado de las N muestras crudas:
// std permite recuperar la varianza empirica (sigma^2) por medida.
struct DaqStats {
    double mean   = 0.0;  // media
    double median = 0.0;  // mediana
    double std    = 0.0;  // desviacion estandar muestral (n-1)
    int    n      = 0;    // numero de muestras efectivas
    bool   ok     = false; // true si la adquisicion fue valida
};

// Adquiere nSamples a fSample Hz del canal cfg::DAQ_CHANNEL.
// data debe estar preasignado con al menos nSamples elementos.
// Devuelve el numero de muestras leidas (0 si hubo error).
int daqAcquire(int nSamples, int fSample, double* data);

// Adquiere nSamples a fSample Hz y devuelve el resumen estadistico.
// No conserva las muestras crudas.
DaqStats daqAcquireStats(int nSamples, int fSample);

#endif // DAQ_H
