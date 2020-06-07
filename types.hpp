#ifndef RECORDER_TYPES_H_
#define RECORDER_TYPES_H_

namespace cv { class Mat; } // fwd decl

namespace recorder {
struct Vector3d {
    double x, y, z;
};

struct Quaternion {
    double x, y, z, w;
};

struct Pose {
    /** Timestamp in seconds. Monotonically increasing */
    double time;

    /**
     * 3D position in a right-handed metric coordinate system
     * where the z-axis points up
     */
    Vector3d position;

    /** Orientation quaternion in the same coordinate system as position */
    Quaternion orientation;
};

struct FrameData {
    double t;
    int cameraInd;
    double focalLength;
    double px;
    double py;
    /** Optional: Frame data as an OpenCV matrix. If present, recorded to a video file */
    const cv::Mat *frameData = nullptr;
};
} // namespace recorder

#endif
