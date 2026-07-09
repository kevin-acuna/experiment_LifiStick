#pragma once
#ifndef DATALOG_H
#define DATALOG_H

// =============================================================================
// datalog.h
// Utilidades de almacenamiento del dataset:
//  - timestamps
//  - creacion de directorios de sesion
//  - metadata de sesion (key=value)  -> I_LED, temperaturas, V_dark, etc. (manual)
//  - escritura CSV
//  - estado de reanudacion (resume) para experimentos largos
// =============================================================================

#include <string>
#include <vector>
#include <utility>
#include <fstream>

namespace datalog {

// ---- Timestamps -------------------------------------------------------------
std::string stamp();      // YYYYMMDD_HHMMSS (nombres de archivo / session_id)
std::string date();       // YYYY-MM-DD
std::string clockTime();  // HH:MM:SS

// ---- Directorios ------------------------------------------------------------
// Crea el directorio (recursivamente). Devuelve true si existe o se creo.
bool ensureDir(const std::string& path);

// ---- Metadata de sesion (key=value) ----------------------------------------
// Escribe un archivo de texto legible con la metadata de trazabilidad de la sesion.
class Metadata {
public:
    void set(const std::string& key, const std::string& value);
    void set(const std::string& key, double value);
    void set(const std::string& key, int value);
    bool write(const std::string& path) const;
private:
    std::vector<std::pair<std::string, std::string>> items_;
};

// ---- CSV writer -------------------------------------------------------------
// Envoltura ligera sobre ofstream: escribe la cabecera al crear y expone el
// stream para volcar filas. Usa append=true para reanudar.
class CsvWriter {
public:
    bool open(const std::string& path, const std::string& header, bool append = false);
    std::ofstream& stream() { return f_; }
    void flush() { if (f_.is_open()) f_.flush(); }
    void close() { if (f_.is_open()) f_.close(); }
    bool isOpen() const { return f_.is_open(); }
private:
    std::ofstream f_;
};

// ---- Estado de reanudacion (resume) -----------------------------------------
struct ResumeState {
    int         nextIndex    = 0;   // proximo indice a procesar
    std::string csvPath;            // CSV en curso
    std::string sessionStamp;       // timestamp de la sesion
    int         counter      = 0;   // contador de muestras
    bool        valid        = false;
};

void        saveState(const std::string& stateFile, const ResumeState& s);
ResumeState loadState(const std::string& stateFile);
void        deleteState(const std::string& stateFile);

} // namespace datalog

#endif // DATALOG_H
