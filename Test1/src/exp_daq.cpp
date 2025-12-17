#define NOMINMAX
#include <Windows.h>
#include <iostream>
#include <string>
#include <NIDAQmx.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <limits>
#include <cctype>

using namespace std;

#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

const int STABILIZATION_TIME_MS = 2000;
const int ORIENTATION_TIME_SEC = 1;
int32 fSample = 1000;
int32 nSamples = ORIENTATION_TIME_SEC * fSample;

static const double PREDEFINED_ORIENTATIONS[][2] = {
    {20,0},
    {20,120},
    {20,240}, 
};

#define K_ORIENTATIONS (sizeof(PREDEFINED_ORIENTATIONS) / sizeof(PREDEFINED_ORIENTATIONS[0]))

int AcquireDataFromDAQ(int32 nSamples, int32 fSample, float64* data) {
    int32       error = 0;
    TaskHandle  taskHandle = 0;
    int32       read = 0;
    char        errBuff[2048] = { '\0' };

    try {
        DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
        DAQmxErrChk(DAQmxCreateAIVoltageChan(
            taskHandle,
            "Dev1/ai1",
            "",
            DAQmx_Val_RSE,
            -10.0,
            10.0,
            DAQmx_Val_Volts,
            NULL
        ));
        
        DAQmxErrChk(DAQmxCfgSampClkTiming(
            taskHandle,
            "",
            fSample,
            DAQmx_Val_Rising,
            DAQmx_Val_FiniteSamps,
            nSamples
        ));

        DAQmxErrChk(DAQmxStartTask(taskHandle));

        DAQmxErrChk(DAQmxReadAnalogF64(
            taskHandle,
            nSamples,
            200.0,
            DAQmx_Val_GroupByChannel,
            data,
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
        std::cerr << "Excepción estándar: " << ex.what() << std::endl;
        return 0;
    }
    
    return read;
}

std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::tm timeinfo;
    localtime_s(&timeinfo, &in_time_t);
    
    std::ostringstream oss;
    oss << std::put_time(&timeinfo, "%Y%m%d_%H%M%S");
    return oss.str();
}

int main()
{
    system("chcp 65001 > nul");
    
    cout << "====================================================\n";
    cout << " DAQ Data Acquisition - " << K_ORIENTATIONS << " Orientations\n";
    cout << "====================================================\n\n";
    
    cout << "Configuración:\n";
    cout << "- Frecuencia de muestreo: " << fSample << " Hz\n";
    cout << "- Tiempo por orientación: " << ORIENTATION_TIME_SEC << " segundos\n";
    cout << "- Muestras por orientación: " << nSamples << "\n";
    cout << "- Tiempo de estabilización: " << STABILIZATION_TIME_MS << " ms\n\n";
    
    cout << "Orientaciones a muestrear:\n";
    for (int i = 0; i < (int)K_ORIENTATIONS; i++) {
        cout << "  " << (i + 1) << ") Inclinación: " << PREDEFINED_ORIENTATIONS[i][0] 
             << "°, Azimuth: " << PREDEFINED_ORIENTATIONS[i][1] << "°\n";
    }
    cout << "\n";
    
    char option;
    while (true) {
        cout << "Presione C para comenzar o Q para salir: ";
        cin >> option;
        option = toupper(option);
        if (option == 'C' || option == 'Q')
            break;
        cout << "Opción inválida. Intente nuevamente.\n";
    }
    
    if (option == 'Q') {
        cout << "Programa terminado por el usuario.\n";
        return 0;
    }
    
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    
    std::string timestamp = getTimestamp();
    std::string csv_filename = "data_daq_" + timestamp + ".csv";
    std::ofstream csv_file(csv_filename);
    
    if (!csv_file.is_open()) {
        std::cerr << "Error al crear archivo CSV: " << csv_filename << std::endl;
        return 1;
    }
    
    csv_file << "sample_id,inclinacion,azimuth,medida_daq" << std::endl;
    cout << "\nArchivo CSV creado: " << csv_filename << "\n\n";
    
    int sample_counter = 0;
    
    cout << "================================================\n";
    cout << "Iniciando adquisición de datos...\n";
    cout << "================================================\n\n";
    
    for (int i = 0; i < (int)K_ORIENTATIONS; i++) {
        double inclination = PREDEFINED_ORIENTATIONS[i][0];
        double azimuth = PREDEFINED_ORIENTATIONS[i][1];
        
        sample_counter++;
        std::string sample_id = timestamp + "_" + std::to_string(sample_counter);
        
        cout << "------------------------------------------------\n";
        cout << "Orientación " << (i + 1) << "/" << K_ORIENTATIONS << "\n";
        cout << "Sample ID: " << sample_id << "\n";
        cout << "Inclinación: " << inclination << "°, Azimuth: " << azimuth << "°\n";
        cout << "------------------------------------------------\n";
        
        cout << "Ajuste manualmente el transmisor a la orientación indicada.\n";
        cout << "Presione ENTER cuando esté listo para adquirir datos...";
        cin.get();
        
        cout << "Esperando estabilización (" << STABILIZATION_TIME_MS << " ms)...\n";
        Sleep(STABILIZATION_TIME_MS);
        
        float64* daq_data = new float64[nSamples];
        
        cout << "Adquiriendo datos por " << ORIENTATION_TIME_SEC << " segundos...\n";
        
        int read_samples = AcquireDataFromDAQ(nSamples, fSample, daq_data);
        
        if (read_samples > 0) {
            cout << "✓ Adquisición exitosa: " << read_samples << " muestras.\n";
            
            for (int j = 0; j < read_samples; j++) {
                csv_file << sample_id << ","
                         << inclination << "," 
                         << azimuth << ","
                         << daq_data[j] << std::endl;
            }
        } else {
            cout << "✗ Error al adquirir datos de la DAQ.\n";
            csv_file << sample_id << ","
                     << inclination << "," 
                     << azimuth << ",NA" << std::endl;
        }
        
        delete[] daq_data;
        cout << "\n";
    }
    
    csv_file.close();
    
    cout << "================================================\n";
    cout << "Adquisición completada.\n";
    cout << "Datos guardados en: " << csv_filename << "\n";
    cout << "Total de orientaciones: " << K_ORIENTATIONS << "\n";
    cout << "================================================\n";
    
    return 0;
}
