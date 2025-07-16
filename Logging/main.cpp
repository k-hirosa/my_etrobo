#include "logger.h"
#include <chrono>
#include <thread>

int main() {
    Logger logger;

    // ダミーデータをログ
    for (int i = 0; i < 1000; ++i) {
        double t = i * 0.01; // 時間 [s]
        logger.addLog({
            {"time", t},
            {"x", 0.1 * i},
            {"y", 0.05 * i},
            {"theta", 0.01 * i},
            {"v", 1.0},
            {"w", 0.1}
        });

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 終了時に書き出し
    logger.saveCsv("log.csv");

    return 0;
}
