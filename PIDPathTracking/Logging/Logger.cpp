#include "Logger.h"

Logger::Logger() : log_count(0) {}

void Logger::addLog(float time, float x, float y, float yaw) {
    if (log_count >= MAX_LOG_ENTRIES) return;
    log_data[log_count][0] = time;
    log_data[log_count][1] = x;
    log_data[log_count][2] = y;
    log_data[log_count][3] = yaw;
    log_count++;
}

void Logger::saveCsv(const char* filename) {
    FILE* fp = fopen(filename, "w");
    if (fp == NULL) {
        perror("fopen failed");
        return;
    }

    fprintf(fp, "time,x,y,yaw\n");
    for (int i = 0; i < log_count; i++) {
        fprintf(fp, "%.2f,%.2f,%.2f,%.2f\n",
                log_data[i][0], log_data[i][1],
                log_data[i][2], log_data[i][3]);
    }
    printf("Saving to: %s\n", filename);

    fclose(fp);
}
