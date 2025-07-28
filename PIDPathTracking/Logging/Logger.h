#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>

#define MAX_LOG_ENTRIES 1000
#define MAX_COLUMNS     4

class Logger {
public:
    Logger();
    void addLog(float time, float x, float y, float yaw);
    void saveCsv(const char* filename);

private:
    int log_count;
    float log_data[MAX_LOG_ENTRIES][MAX_COLUMNS];
};

#endif
