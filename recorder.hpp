#ifndef RECORDER_H_
#define RECORDER_H_

#include <fstream>
#include <memory>
#include <string>
#include <vector>

// can also use json_fwd here, but then this is less convenient to use as the
// proper include is needed on every file
#include <nlohmann/json.hpp>
#include "types.hpp"

namespace recorder {
class Recorder {
public:
    /**
     * Create new recording. To do multiple recordings during one session, call the constructor multiple times.
     *
     * @param outputPath Platform path to a non-existing file which has write permissions.
     */
    static std::unique_ptr<Recorder> build(const std::string& outputPath);

    /**
     * @ param output Stream to which output will be written.
     */
    static std::unique_ptr<Recorder> build(std::ostream& output);
    virtual ~Recorder();

    /**
     * Flush and close output file.
     */
    virtual void closeOutputFile() = 0;
    virtual void addGyroscope(double t, double x, double y, double z) = 0;
    virtual void addAccelerometer(double t, double x, double y, double z) = 0;
    virtual void addFrame(const FrameData& f) = 0;
    virtual void addFrameGroup(double t, const std::vector<FrameData>& frames) = 0;
    virtual void addARKit(const Pose &pose) = 0;
    virtual void addGroundTruth(const Pose &pose) = 0;
    virtual void addOdometryOutput(const Pose &pose, const Vector3d &velocity) = 0;
    virtual void addGps(
        double t,
        double latitude,
        double longitude,
        double horizontalUncertainty,
        double altitude) = 0;

    /**
     * Write arbitrary serialized JSON into the recording.
     *
     * @param line The serialized data. Needs to be valid JSON.
     */
    virtual void addJsonString(const std::string &line) = 0;

    /**
     * Write arbitrary JSON object into the recording.
     *
     * @param j The JSON object.
     */
    virtual void addJson(const nlohmann::json &j) = 0;

protected:
    Recorder(const Recorder &other);
    Recorder();
};

} // namespace recorder

#endif // RECORDER_H_
