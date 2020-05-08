#include <cstdio>
#include "recorder.hpp"

#define log_warn std::printf

namespace {
using namespace recorder;
using json = nlohmann::json;

struct RecorderImplementation : public Recorder {
    std::ofstream fileOutput;
    std::ostream& output;
    int frameNumberGroup = 0;
    std::map<int, int> frameNumbers = {};

    // Preallocate.
    struct Workspace {
        json jGyroscope = R"({
            "time": 0.0,
            "sensor": {
                "type": "gyroscope",
                "values": [0.0, 0.0, 0.0]
            }
        })"_json;
        json jAccelerometer = R"({
            "time": 0.0,
            "sensor": {
                "type": "accelerometer",
                "values": [0.0, 0.0, 0.0]
            }
        })"_json;
        json jGps = R"({
            "time": 0.0,
            "gps": {
                "latitude": 0.0,
                "longitude": 0.0,
                "accuracy": 0.0,
                "altitude": 0.0
            }
        })"_json;
        json jOutput = R"({
            "time": 0.0,
            "output": {
                "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
                "orientation": { "w": 0.0, "x": 0.0, "y": 0.0, "z": 0.0 },
                "velocity": { "x": 0.0, "y": 0.0, "z": 0.0 }
            }
        })"_json;
        json jGroundTruth = R"({
            "time": 0.0,
            "groundTruth": {
                "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
                "orientation": { "w": 0.0, "x": 0.0, "y": 0.0, "z": 0.0 }
            }
        })"_json;
        json jARKit = R"({
            "time": 0.0,
            "ARKit": {
                "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
                "orientation": { "w": 0.0, "x": 0.0, "y": 0.0, "z": 0.0 }
            }
        })"_json;
        json jFrame = R"({
            "time": 0.0,
            "cameraInd": 0,
            "number": 0,
            "cameraParameters": {
                "focalLength": 0.0,
                "principalPointX": 0.0,
                "principalPointY": 0.0
            }
        })"_json;
        json jFrameGroup = R"({
            "time": 0.0,
            "number": 0,
            "frames": []
        })"_json;
    } workspace;

    RecorderImplementation(std::ostream& output) :
        fileOutput(),
        output(output)
    {
        init();
    }

    RecorderImplementation(const std::string& outputPath) :
        fileOutput(outputPath),
        output(this->fileOutput)
    {
        init();
    }

    void init() {
        output.precision(10);
    }

    void closeOutputFile() {
        fileOutput.close();
    }

    void addGyroscope(double t, double x, double y, double z) {
        workspace.jGyroscope["time"] = t;
        workspace.jGyroscope["sensor"]["values"] = { x, y, z };
        output << workspace.jGyroscope.dump() << std::endl;
    }

    void addAccelerometer(double t, double x, double y, double z) {
        workspace.jAccelerometer["time"] = t;
        workspace.jAccelerometer["sensor"]["values"] = { x, y, z };
        output << workspace.jAccelerometer.dump() << std::endl;
    }

    void setFrame(const FrameData& f) {
        workspace.jFrame["time"] = f.t;
        workspace.jFrame["cameraInd"] = f.cameraInd;

        workspace.jFrame.erase("cameraParameters");
        if (f.focalLength > 0.0) {
            workspace.jFrame["cameraParameters"]["focalLength"] = f.focalLength;
        }
        if (f.px > 0.0 && f.py > 0.0) {
            workspace.jFrame["cameraParameters"]["principalPointX"] = f.px;
            workspace.jFrame["cameraParameters"]["principalPointY"] = f.py;
        }
    }

    void addFrame(const FrameData& f) {
        setFrame(f);
        workspace.jFrame["number"] = frameNumberGroup;
        workspace.jFrameGroup["time"] = f.t;
        workspace.jFrameGroup["number"] = frameNumberGroup;
        workspace.jFrameGroup["frames"] = {};
        workspace.jFrameGroup["frames"].push_back(workspace.jFrame);
        output << workspace.jFrameGroup.dump() << std::endl;
        frameNumberGroup++;
    }

    void addFrameGroup(double t, const std::vector<FrameData>& frames) {
        workspace.jFrameGroup["time"] = t;
        workspace.jFrameGroup["number"] = frameNumberGroup;
        workspace.jFrameGroup["frames"] = {};
        for (auto f : frames) {
            // Track frame numbers of each camera because some frame groups
            // may only contain output from some of the cameras (happens on iOS).
            try {
                frameNumbers.at(f.cameraInd)++;
            } catch (const std::out_of_range &e) {
                frameNumbers[f.cameraInd] = 0;
            }
            setFrame(f);
            workspace.jFrame["number"] = frameNumbers[f.cameraInd];
            workspace.jFrameGroup["frames"].push_back(workspace.jFrame);
        }
        output << workspace.jFrameGroup.dump() << std::endl;
        frameNumberGroup++;
    }

    void setPose(const Pose &pose, json &j, const std::string &name, bool hasOrientation) {
        j["time"] = pose.time;
        j[name]["position"]["x"] = pose.position.x;
        j[name]["position"]["y"] = pose.position.y;
        j[name]["position"]["z"] = pose.position.z;

        j[name].erase("orientation");
        if (hasOrientation) {
            j[name]["orientation"]["w"] = pose.orientation.w;
            j[name]["orientation"]["x"] = pose.orientation.x;
            j[name]["orientation"]["y"] = pose.orientation.y;
            j[name]["orientation"]["z"] = pose.orientation.z;
        }
    }

    void addARKit(const Pose &pose) {
        setPose(pose, workspace.jARKit, "ARKit", false);
        output << workspace.jARKit.dump() << std::endl;
    }

    void addGroundTruth(const Pose &pose) {
        setPose(pose, workspace.jGroundTruth, "groundTruth", false);
        output << workspace.jGroundTruth.dump() << std::endl;
    }

    void addOdometryOutput(const Pose &pose, const Vector3d &velocity) {
        setPose(pose, workspace.jOutput, "output", true);
        workspace.jOutput["output"]["velocity"]["x"] = velocity.x;
        workspace.jOutput["output"]["velocity"]["y"] = velocity.y;
        workspace.jOutput["output"]["velocity"]["z"] = velocity.z;
        output << workspace.jOutput.dump() << std::endl;
    }

    void addGps(
        double t,
        double latitude,
        double longitude,
        double horizontalUncertainty,
        double altitude)
    {
        workspace.jGps["time"] = t;
        workspace.jGps["gps"]["latitude"] = latitude;
        workspace.jGps["gps"]["longitude"] = longitude;
        // We have no standard for what "accuracy" means.
        workspace.jGps["gps"]["accuracy"] = horizontalUncertainty;
        workspace.jGps["gps"]["altitude"] = altitude;
        output << workspace.jGps.dump() << std::endl;
    }

    void addJsonString(const std::string &line) {
        json j;
        try {
            j = json::parse(line);
        } catch (const nlohmann::detail::parse_error &e) {
            log_warn("recorder addLine(): Skipping invalid JSON: %s", line.c_str());
            return;
        }

        // Make sure output is exactly one line.
        size_t n = line.find('\n');
        if (n == std::string::npos) {
            output << line << std::endl;
        } else if (n + 1 < line.size()) {
            // Re-serialize multiline input.
            output << j.dump() << std::endl;
        } else {
            output << line;
        }
    }

    void addJson(const json &j) {
        output << j.dump() << std::endl;
    }
};

} // anonymous namespace

namespace recorder {

Recorder::~Recorder() = default;
Recorder::Recorder(const Recorder &other) = default;

std::unique_ptr<Recorder> Recorder::build(const std::string& outputPath) {
    return std::unique_ptr<Recorder>(new RecorderImplementation(outputPath));
}

std::unique_ptr<Recorder> Recorder::build(std::ostream& output) {
    return std::unique_ptr<Recorder>(new RecorderImplementation(output));
}

Recorder::Recorder() {}

} // namespace recorder
