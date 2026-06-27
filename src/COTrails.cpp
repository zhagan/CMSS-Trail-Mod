#include "plugin.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ctime>

static time_t gpxTimeToEpoch(const std::string& s) {
    std::tm tm = {};
    if (sscanf(s.c_str(), "%d-%d-%dT%d:%d:%d",
        &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
        &tm.tm_hour, &tm.tm_min, &tm.tm_sec) != 6) {
        return 0;
    }

    tm.tm_year -= 1900;
    tm.tm_mon -= 1;
    tm.tm_isdst = 0;

#if defined(_WIN32)
    return _mkgmtime(&tm);
#else
    return timegm(&tm);
#endif
}

struct TrailPoint {
    float lon;
    float lat;
    float ele;

    TrailPoint() : lon(0.f), lat(0.f), ele(0.f) {}
    TrailPoint(float lon, float lat, float ele)
        : lon(lon), lat(lat), ele(ele) {}
};

struct COTrails : Module {
    enum ParamId {
        RATE_PARAM,
        PARAMS_LEN
    };
    enum InputId {
        INPUTS_LEN
    };
    enum OutputId {
        LON_OUTPUT,
        LAT_OUTPUT,
        ELEVATION_OUTPUT,
        PEAK_GATE_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId {
        LIGHTS_LEN
    };

    std::vector<TrailPoint> trail;

    float minLon = 0.f, maxLon = 1.f;
    float minLat = 0.f, maxLat = 1.f;
    float minEle = 0.f, maxEle = 1.f;

    float playhead = 0.f;
    float trailDurationSec = 3600.f;

    int lastIndex = -1;
    dsp::PulseGenerator peakPulse;

    COTrails() {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(RATE_PARAM, 0.f, 1.f, 0.f, "Trail rate", "x", 0.f, 5000.f);

        configOutput(LON_OUTPUT, "Longitude X");
        configOutput(LAT_OUTPUT, "Latitude Y");
        configOutput(ELEVATION_OUTPUT, "Elevation Z");
        configOutput(PEAK_GATE_OUTPUT, "Peak gate");

        trail = {
            {-105.00f, 38.80f, 2200.f},
            {-105.01f, 38.81f, 2400.f},
            {-105.03f, 38.82f, 2600.f},
            {-105.02f, 38.84f, 3000.f},
            {-105.05f, 38.86f, 3400.f},
            {-105.07f, 38.88f, 4300.f}
        };

        recomputeBounds();
        loadGPXFile(asset::plugin(pluginInstance, "res/pikes_peak.gpx"));
    }

    static float norm(float v, float mn, float mx) {
        float d = mx - mn;
        if (std::abs(d) < 0.000001f)
            return 0.5f;
        return clamp((v - mn) / d, 0.f, 1.f);
    }

    void recomputeBounds() {
        if (trail.empty())
            return;

        minLon = maxLon = trail[0].lon;
        minLat = maxLat = trail[0].lat;
        minEle = maxEle = trail[0].ele;

        for (const TrailPoint& p : trail) {
            minLon = std::min(minLon, p.lon);
            maxLon = std::max(maxLon, p.lon);
            minLat = std::min(minLat, p.lat);
            maxLat = std::max(maxLat, p.lat);
            minEle = std::min(minEle, p.ele);
            maxEle = std::max(maxEle, p.ele);
        }

        if (std::abs(maxLon - minLon) < 0.000001f) maxLon = minLon + 1.f;
        if (std::abs(maxLat - minLat) < 0.000001f) maxLat = minLat + 1.f;
        if (std::abs(maxEle - minEle) < 0.000001f) maxEle = minEle + 1.f;
    }

    static bool parseFloatAttr(const std::string& tag, const std::string& attr, float& out) {
        std::string key = attr + "=\"";
        size_t start = tag.find(key);
        if (start == std::string::npos)
            return false;

        start += key.size();
        size_t end = tag.find("\"", start);
        if (end == std::string::npos)
            return false;

        try {
            out = std::stof(tag.substr(start, end - start));
            return true;
        }
        catch (...) {
            return false;
        }
    }

    bool loadGPXFile(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open())
            return false;

        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string text = buffer.str();

        std::vector<TrailPoint> parsedTrail;
        std::vector<time_t> parsedTimes;

        size_t pos = 0;

        while (true) {
            size_t trkptStart = text.find("<trkpt", pos);
            if (trkptStart == std::string::npos)
                break;

            size_t trkptTagEnd = text.find(">", trkptStart);
            if (trkptTagEnd == std::string::npos)
                break;

            size_t trkptEnd = text.find("</trkpt>", trkptTagEnd);
            if (trkptEnd == std::string::npos)
                break;

            std::string tag = text.substr(trkptStart, trkptTagEnd - trkptStart + 1);
            std::string body = text.substr(trkptTagEnd + 1, trkptEnd - trkptTagEnd - 1);

            TrailPoint p;

            bool hasLat = parseFloatAttr(tag, "lat", p.lat);
            bool hasLon = parseFloatAttr(tag, "lon", p.lon);

            size_t eleStart = body.find("<ele>");
            size_t eleEnd = body.find("</ele>");

            if (hasLat && hasLon && eleStart != std::string::npos && eleEnd != std::string::npos) {
                eleStart += 5;
                try {
                    p.ele = std::stof(body.substr(eleStart, eleEnd - eleStart));
                    parsedTrail.push_back(p);
                }
                catch (...) {
                }
            }

            size_t timeStart = body.find("<time>");
            size_t timeEnd = body.find("</time>");

            if (timeStart != std::string::npos && timeEnd != std::string::npos) {
                timeStart += 6;
                time_t t = gpxTimeToEpoch(body.substr(timeStart, timeEnd - timeStart));
                if (t > 0)
                    parsedTimes.push_back(t);
            }

            pos = trkptEnd + 8;
        }

        if (parsedTrail.size() < 3)
            return false;

        trail = parsedTrail;
        recomputeBounds();

        if (parsedTimes.size() >= 2) {
            double duration = difftime(parsedTimes.back(), parsedTimes.front());
            if (duration > 1.0)
                trailDurationSec = (float)duration;
        }

        playhead = 0.f;
        lastIndex = -1;

        return true;
    }

    bool isPeak(int i) {
        if (i <= 0 || i >= (int)trail.size() - 1)
            return false;

        return trail[i].ele > trail[i - 1].ele &&
               trail[i].ele > trail[i + 1].ele;
    }

    void process(const ProcessArgs& args) override {
        if (trail.size() < 3)
            return;

        float knob = params[RATE_PARAM].getValue();
        float rateMultiplier = std::pow(5000.f, knob);

        playhead += args.sampleTime * rateMultiplier / trailDurationSec;

        while (playhead >= 1.f)
            playhead -= 1.f;

        int maxIndex = (int)trail.size() - 1;
        int i = clamp((int)std::round(playhead * maxIndex), 1, maxIndex - 1);

        TrailPoint current = trail[i];

        float nx = norm(current.lon, minLon, maxLon);
        float ny = norm(current.lat, minLat, maxLat);
        float nz = norm(current.ele, minEle, maxEle);

        outputs[LON_OUTPUT].setVoltage(nx * 10.f);
        outputs[LAT_OUTPUT].setVoltage(ny * 10.f);
        outputs[ELEVATION_OUTPUT].setVoltage(nz * 10.f);

        if (i != lastIndex) {
            if (isPeak(i))
                peakPulse.trigger(0.01f);

            lastIndex = i;
        }

        bool gateHigh = peakPulse.process(args.sampleTime);
        outputs[PEAK_GATE_OUTPUT].setVoltage(gateHigh ? 10.f : 0.f);
    }
};

struct TrailDisplay : TransparentWidget {
    COTrails* module = nullptr;

    Vec projectPoint(const TrailPoint& p, float w, float h) {
        float x = COTrails::norm(p.lon, module->minLon, module->maxLon);
        float y = COTrails::norm(p.lat, module->minLat, module->maxLat);
        float z = COTrails::norm(p.ele, module->minEle, module->maxEle);

        x = (x - 0.5f) * 2.f;
        y = (y - 0.5f) * 2.f;

        float sx = (x - y) * 0.5f;
        float sy = (x + y) * 0.25f - z * 0.9f;

        return Vec(
            w * 0.5f + sx * w * 0.42f,
            h * 0.72f + sy * h * 0.55f
        );
    }

    void draw(const DrawArgs& args) override {
        if (!module || module->trail.size() < 2)
            return;

        const float w = box.size.x;
        const float h = box.size.y;

        nvgSave(args.vg);

        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, w, h);
        nvgFillColor(args.vg, nvgRGB(20, 30, 20));
        nvgFill(args.vg);

        nvgBeginPath(args.vg);

        for (size_t i = 0; i < module->trail.size(); i++) {
            Vec p = projectPoint(module->trail[i], w, h);

            if (i == 0)
                nvgMoveTo(args.vg, p.x, p.y);
            else
                nvgLineTo(args.vg, p.x, p.y);
        }

        nvgStrokeColor(args.vg, nvgRGB(0, 220, 80));
        nvgStrokeWidth(args.vg, 1.5f);
        nvgStroke(args.vg);

        int maxIndex = (int)module->trail.size() - 1;
        int index = clamp((int)std::round(module->playhead * maxIndex), 0, maxIndex);

        Vec dot = projectPoint(module->trail[index], w, h);

        nvgBeginPath(args.vg);
        nvgCircle(args.vg, dot.x, dot.y, 2.5f);
        nvgFillColor(args.vg, nvgRGB(0, 0, 0));
        nvgFill(args.vg);

        nvgRestore(args.vg);
    }
};

struct COTrailsWidget : ModuleWidget {
    COTrailsWidget(COTrails* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/COTrails_v3.svg")));
        box.size = mm2px(Vec(30.48, 128.5));

        TrailDisplay* display = new TrailDisplay;
        display->module = module;
        display->box.pos = mm2px(Vec(4, 18));
        display->box.size = mm2px(Vec(22.48, 18));
        addChild(display);

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addParam(createParamCentered<RoundBlackKnob>(
            mm2px(Vec(15.24, 45)),
            module,
            COTrails::RATE_PARAM
        ));

        addOutput(createOutputCentered<PJ301MPort>(
            mm2px(Vec(15.24, 60)),
            module,
            COTrails::LON_OUTPUT
        ));

        addOutput(createOutputCentered<PJ301MPort>(
            mm2px(Vec(15.24, 75)),
            module,
            COTrails::LAT_OUTPUT
        ));

        addOutput(createOutputCentered<PJ301MPort>(
            mm2px(Vec(15.24, 90)),
            module,
            COTrails::ELEVATION_OUTPUT
        ));

        addOutput(createOutputCentered<PJ301MPort>(
            mm2px(Vec(15.24, 105)),
            module,
            COTrails::PEAK_GATE_OUTPUT
        ));
    }
};

Model* modelCOTrails = createModel<COTrails, COTrailsWidget>("COTrails");