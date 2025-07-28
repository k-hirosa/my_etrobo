#ifndef LOCAL_PATH_CONFIG_H
#define LOCAL_PATH_CONFIG_H

#define LOCAL_PATH_COUNT 3

typedef struct {
    float x;
    float y;
    float yaw;
} LocalPathPoint;

static const LocalPathPoint LOCAL_PATH[LOCAL_PATH_COUNT] = {
    { 0.0f, 0.0f, 0.0f },
    { 0.5f, 0.0f, 0.0f },
    { 1.0f, 0.0f, 0.0f }
};

#endif // LOCAL_PATH_CONFIG_H
