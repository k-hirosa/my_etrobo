#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <vector>
#include <map>

class Logger {
public:
    void addLog(const std::map<std::string, double>& values);
    void saveCsv(const std::string& filename) const;

private:
    std::vector<std::map<std::string, double>> logs_;
};

#endif