#include "logger.h"
#include <fstream>
#include <set>

void Logger::addLog(const std::map<std::string, double>& values) {
    logs_.push_back(values);
}

void Logger::saveCsv(const std::string& filename) const {
    std::ofstream ofs(filename);
    if (!ofs) return;

    // ���ׂẴL�[�����W
    std::set<std::string> keys;
    for (const auto& row : logs_) {
        for (const auto& kv : row) {
            keys.insert(kv.first);
        }
    }

    // �w�b�_�o��
    bool first = true;
    for (const auto& key : keys) {
        if (!first) ofs << ",";
        ofs << key;
        first = false;
    }
    ofs << "\n";

    // �f�[�^�o��
    for (const auto& row : logs_) {
        first = true;
        for (const auto& key : keys) {
            if (!first) ofs << ",";
            auto it = row.find(key);
            if (it != row.end()) {
                ofs << it->second;
            } else {
                ofs << "";
            }
            first = false;
        }
        ofs << "\n";
    }
}