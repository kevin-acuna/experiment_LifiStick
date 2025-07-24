

#include <iostream>
#include <NIDAQmx.h>
#include <cwchar>
#include <fstream>
#include <string>
#include <thread>


#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

int AcquireData(int32 nSamples, int32 fSample, float64* data);

int main()
{
    int fSample = 1000;
    for (int i = 1; i < 6; i++) {
        int nSamples = 100 * i;
        float64* data = new float64[nSamples];

        // Open a CSV file for writing
        std::ofstream file("data_function_" + std::to_string(i) + ".csv");

        if (!file.is_open()) {
            std::cerr << "Error opening the file!" << std::endl;
            delete[] data;
            return -1;
        }

        int read = AcquireData(nSamples, fSample, data);

        if (read <= 0) {
            std::cerr << "Acquisition failed or no data acquired for iteration " << i << std::endl;
        }
        // Write data to CSV file
        if (read > 0) {
            for (int j = 0; j < read; ++j) {
                file << data[j] << std::endl;
            }
            std::cout << "Data saved to 'data_function_" << i << ".csv'." << std::endl;
        }
        else {
            std::cerr << "No samples acquired for iteration " << i << std::endl;
        }
        delete[] data;
        file.close(); // Ensure the file is closed

        std::cout << "Wait 10s" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
    std::cout << "End of program. Press Enter to quit." << std::endl;
    std::cin.get();
    return 0;
}

int AcquireData(int32  nSamples, int32 fSample, float64* data) {
    int32       error = 0;
    TaskHandle  taskHandle = 0;
    int32       read = 0;
    char        errBuff[2048] = { '\0' };

    try
    {
        /*********************************************/
        // DAQmx Configure Code
        /*********************************************/
        DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
        DAQmxErrChk(DAQmxCreateAIVoltageChan(
            taskHandle,
            "Dev1/ai1",  // "Dev1/ai1": Dev1 is the default name of the NI-DAQ, and ai1 is the port used on the NI-DAQ
            "",
            DAQmx_Val_RSE,  // Connection mode (RSE - Single-Ended)
            -10.0, // Minimum voltage range
            10.0, // Maximum voltage range
            DAQmx_Val_Volts, // Measurement units (Volts)
            NULL
        ));
        DAQmxErrChk(DAQmxCfgSampClkTiming( // Sampling clock configuration
            taskHandle,
            "",                  // Use of internal clock
            fSample,              // Sampling frequency (internal)
            DAQmx_Val_Rising,
            DAQmx_Val_FiniteSamps,
            nSamples                // Number of samples to measure
        ));

        /*********************************************/
        // DAQmx Start Code
        /*********************************************/
        DAQmxErrChk(DAQmxStartTask(taskHandle));

        /*********************************************/
        // DAQmx Read Code
        /*********************************************/
        DAQmxErrChk(DAQmxReadAnalogF64(
            taskHandle,
            nSamples, // Read exactly nSamples
            200.0, //  timeout 
            DAQmx_Val_GroupByChannel,
            data, // Array to store the acquired data
            nSamples, // Read up to nSamples
            &read,
            NULL
        ));

    Error:  // Error handling
        if (DAQmxFailed(error))
            DAQmxGetExtendedErrorInfo(errBuff, 2048);

        if (taskHandle != 0)
        {
            DAQmxStopTask(taskHandle);
            DAQmxClearTask(taskHandle);
        }

        if (DAQmxFailed(error))
        {
            std::cerr << "DAQmx Error: " << errBuff << std::endl;
        }
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Standard exception: " << ex.what() << std::endl;
    }
    return read;
}