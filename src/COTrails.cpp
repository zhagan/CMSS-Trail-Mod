#include "plugin.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <ctime>
#include <array>
#include <atomic>

static time_t gpxTimeToEpoch(const std::string &s)
{
    std::tm tm = {};
    if (sscanf(s.c_str(), "%d-%d-%dT%d:%d:%d",
               &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
               &tm.tm_hour, &tm.tm_min, &tm.tm_sec) != 6)
    {
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

struct TrailPoint
{
    float lon;
    float lat;
    float ele;

    TrailPoint() : lon(0.f), lat(0.f), ele(0.f) {}
    TrailPoint(float lon, float lat, float ele)
        : lon(lon), lat(lat), ele(ele) {}
};

struct TrailDefinition
{
    std::string name;
    std::string resourcePath;
};

struct CachedTrail
{
    std::vector<TrailPoint> points;
    float minLon = 0.f;
    float maxLon = 1.f;
    float minLat = 0.f;
    float maxLat = 1.f;
    float minEle = 0.f;
    float maxEle = 1.f;
    float durationSec = 3600.f;
};

struct COTrails : Module
{
    enum ParamId
    {
        RATE_PARAM,
        TRAIL_PARAM,
        PARAMS_LEN
    };
    enum InputId
    {
        INPUTS_LEN
    };
    enum OutputId
    {
        LON_OUTPUT,
        LAT_OUTPUT,
        ELEVATION_OUTPUT,
        PEAK_GATE_OUTPUT,
        OUTPUTS_LEN
    };
    enum LightId
    {
        LIGHTS_LEN
    };

    std::array<TrailDefinition, 12> trailDefinitions = {{{"Pikes Peak", "res/pikes_peak.gpx"},
                                                         {"Barr Trail", "res/bar_trail.gpx"},
                                                         {"Bear Lakes", "res/bear.gpx"},
                                                         {"Mt Bierstadt", "res/mt_bierstadt.gpx"},
                                                         {"Quandary Peak", "res/quandary_peak.gpx"},
                                                         {"South Reservoir", "res/south_reservoir.gpx"},
                                                         {"Trail 7", ""},
                                                         {"Trail 8", ""},
                                                         {"Trail 9", ""},
                                                         {"Trail 10", ""},
                                                         {"Trail 11", ""},
                                                         {"Trail 12", ""}}};
    std::array<CachedTrail, 12> cachedTrails;

    float playhead = 0.f;
    int lastIndex = -1;
    bool trailsLoaded = false;
    std::atomic<int> activeTrailIndex;
    dsp::PulseGenerator peakPulse;

    COTrails()
        : activeTrailIndex(0)
    {
        config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

        configParam(RATE_PARAM, 0.f, 1.f, 0.f, "Trail rate", "x", 0.f, 5000.f);
        configSwitch(
            TRAIL_PARAM,
            0.f,
            11.f,
            0.f,
            "Trail select",
            {"Pikes Peak",
             "Barr Trail",
             "Bear Lakes",
             "Mt Bierstadt",
             "Quandary Peak",
             "South Reservoir",
             "Trail 7",
             "Trail 8",
             "Trail 9",
             "Trail 10",
             "Trail 11",
             "Trail 12"});

        configOutput(LON_OUTPUT, "Longitude X");
        configOutput(LAT_OUTPUT, "Latitude Y");
        configOutput(ELEVATION_OUTPUT, "Elevation Z");
        configOutput(PEAK_GATE_OUTPUT, "Peak gate");

        paramQuantities[TRAIL_PARAM]->snapEnabled = true;
        initFallbackTrails();
    }

    void initFallbackTrails()
    {
        CachedTrail fallback;
        fallback.points = {
            {-105.00f, 38.80f, 2200.f},
            {-105.01f, 38.81f, 2400.f},
            {-105.03f, 38.82f, 2600.f},
            {-105.02f, 38.84f, 3000.f},
            {-105.05f, 38.86f, 3400.f},
            {-105.07f, 38.88f, 4300.f}};
        recomputeBounds(fallback);
        for (size_t i = 0; i < cachedTrails.size(); i++)
            cachedTrails[i] = fallback;
    }

    static float norm(float v, float mn, float mx)
    {
        float d = mx - mn;
        if (std::abs(d) < 0.000001f)
            return 0.5f;
        return clamp((v - mn) / d, 0.f, 1.f);
    }

    static void recomputeBounds(CachedTrail &trail)
    {
        if (trail.points.empty())
            return;

        trail.minLon = trail.maxLon = trail.points[0].lon;
        trail.minLat = trail.maxLat = trail.points[0].lat;
        trail.minEle = trail.maxEle = trail.points[0].ele;

        for (const TrailPoint &p : trail.points)
        {
            trail.minLon = std::min(trail.minLon, p.lon);
            trail.maxLon = std::max(trail.maxLon, p.lon);
            trail.minLat = std::min(trail.minLat, p.lat);
            trail.maxLat = std::max(trail.maxLat, p.lat);
            trail.minEle = std::min(trail.minEle, p.ele);
            trail.maxEle = std::max(trail.maxEle, p.ele);
        }

        if (std::abs(trail.maxLon - trail.minLon) < 0.000001f)
            trail.maxLon = trail.minLon + 1.f;
        if (std::abs(trail.maxLat - trail.minLat) < 0.000001f)
            trail.maxLat = trail.minLat + 1.f;
        if (std::abs(trail.maxEle - trail.minEle) < 0.000001f)
            trail.maxEle = trail.minEle + 1.f;
    }

    static bool parseFloatAttr(const std::string &tag, const std::string &attr, float &out)
    {
        std::string key = attr + "=\"";
        size_t start = tag.find(key);
        if (start == std::string::npos)
            return false;

        start += key.size();
        size_t end = tag.find("\"", start);
        if (end == std::string::npos)
            return false;

        try
        {
            out = std::stof(tag.substr(start, end - start));
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool loadGPXFile(const std::string &path, CachedTrail &cachedTrail)
    {
        std::ifstream file(path);
        if (!file.is_open())
            return false;

        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string text = buffer.str();

        std::vector<TrailPoint> parsedTrail;
        std::vector<time_t> parsedTimes;

        size_t pos = 0;

        while (true)
        {
            size_t trkptStart = text.find("<trkpt", pos);
            if (trkptStart == std::string::npos)
                break;

            size_t trkptTagEnd = text.find(">", trkptStart);
            if (trkptTagEnd == std::string::npos)
                break;

            std::string tag = text.substr(trkptStart, trkptTagEnd - trkptStart + 1);
            bool selfClosing = tag.size() >= 2 && tag[tag.size() - 2] == '/';
            size_t trkptEnd = std::string::npos;
            std::string body;

            if (!selfClosing)
            {
                trkptEnd = text.find("</trkpt>", trkptTagEnd);
                if (trkptEnd == std::string::npos)
                    break;
                body = text.substr(trkptTagEnd + 1, trkptEnd - trkptTagEnd - 1);
            }

            TrailPoint p;

            bool hasLat = parseFloatAttr(tag, "lat", p.lat);
            bool hasLon = parseFloatAttr(tag, "lon", p.lon);
            bool hasEle = parseFloatAttr(tag, "ele", p.ele);

            if (!hasEle && !selfClosing)
            {
                size_t eleStart = body.find("<ele>");
                size_t eleEnd = body.find("</ele>");

                if (eleStart != std::string::npos && eleEnd != std::string::npos)
                {
                    eleStart += 5;
                    try
                    {
                        p.ele = std::stof(body.substr(eleStart, eleEnd - eleStart));
                        hasEle = true;
                    }
                    catch (...)
                    {
                    }
                }
            }

            if (hasLat && hasLon && hasEle)
                parsedTrail.push_back(p);

            if (!selfClosing)
            {
                size_t timeStart = body.find("<time>");
                size_t timeEnd = body.find("</time>");

                if (timeStart != std::string::npos && timeEnd != std::string::npos)
                {
                    timeStart += 6;
                    time_t t = gpxTimeToEpoch(body.substr(timeStart, timeEnd - timeStart));
                    if (t > 0)
                        parsedTimes.push_back(t);
                }
            }

            pos = selfClosing ? trkptTagEnd + 1 : trkptEnd + 8;
        }

        if (parsedTrail.size() < 3)
            return false;

        cachedTrail.points = parsedTrail;
        recomputeBounds(cachedTrail);

        if (parsedTimes.size() >= 2)
        {
            double duration = difftime(parsedTimes.back(), parsedTimes.front());
            if (duration > 1.0)
                cachedTrail.durationSec = (float)duration;
        }
        else
        {
            cachedTrail.durationSec = 3600.f;
        }

        return true;
    }

    void loadAllTrails()
    {
        if (trailsLoaded)
            return;

        for (size_t i = 0; i < cachedTrails.size(); i++)
        {
            const TrailDefinition &definition = trailDefinitions[i];
            if (!definition.resourcePath.empty())
                loadGPXFile(asset::plugin(pluginInstance, definition.resourcePath), cachedTrails[i]);
        }
        trailsLoaded = true;
    }

    int getSelectedTrailIndex()
    {
        return clamp((int)std::round(params[TRAIL_PARAM].getValue()), 0, (int)trailDefinitions.size() - 1);
    }

    std::string getSelectedTrailName()
    {
        int index = clamp(activeTrailIndex.load(), 0, (int)trailDefinitions.size() - 1);
        return trailDefinitions[index].name;
    }

    const CachedTrail &getActiveTrail() const
    {
        int index = clamp(activeTrailIndex.load(), 0, (int)cachedTrails.size() - 1);
        return cachedTrails[index];
    }

    void selectTrail(int index)
    {
        index = clamp(index, 0, (int)cachedTrails.size() - 1);
        if (index == activeTrailIndex.load())
            return;

        activeTrailIndex.store(index);
        playhead = 0.f;
        lastIndex = -1;
    }

    static bool isPeak(const std::vector<TrailPoint> &trail, int i)
    {
        if (i <= 0 || i >= (int)trail.size() - 1)
            return false;

        return trail[i].ele > trail[i - 1].ele &&
               trail[i].ele > trail[i + 1].ele;
    }

    void process(const ProcessArgs &args) override
    {
        loadAllTrails();
        selectTrail(getSelectedTrailIndex());

        const CachedTrail &trail = getActiveTrail();
        if (trail.points.size() < 3)
            return;

        float knob = params[RATE_PARAM].getValue();
        float rateMultiplier = std::pow(5000.f, knob);

        playhead += args.sampleTime * rateMultiplier / trail.durationSec;

        while (playhead >= 1.f)
            playhead -= 1.f;

        int maxIndex = (int)trail.points.size() - 1;
        int i = clamp((int)std::round(playhead * maxIndex), 1, maxIndex - 1);

        TrailPoint current = trail.points[i];

        float nx = norm(current.lon, trail.minLon, trail.maxLon);
        float ny = norm(current.lat, trail.minLat, trail.maxLat);
        float nz = norm(current.ele, trail.minEle, trail.maxEle);

        outputs[LON_OUTPUT].setVoltage(nx * 10.f);
        outputs[LAT_OUTPUT].setVoltage(ny * 10.f);
        outputs[ELEVATION_OUTPUT].setVoltage(nz * 10.f);

        if (i != lastIndex)
        {
            if (isPeak(trail.points, i))
                peakPulse.trigger(0.01f);

            lastIndex = i;
        }

        bool gateHigh = peakPulse.process(args.sampleTime);
        outputs[PEAK_GATE_OUTPUT].setVoltage(gateHigh ? 10.f : 0.f);
    }
};

struct TrailLabelDisplay : TransparentWidget
{
    COTrails *module = nullptr;

    void draw(const DrawArgs &args) override
    {
        if (!module)
            return;

        std::shared_ptr<Font> font = APP->window->loadFont(asset::system("res/fonts/DejaVuSans.ttf"));
        if (!font)
            return;

        nvgSave(args.vg);

        nvgBeginPath(args.vg);
        nvgRoundedRect(args.vg, 0, 0, box.size.x, box.size.y, 3.f);
        nvgFillColor(args.vg, nvgRGB(18, 18, 18));
        nvgFill(args.vg);

        nvgFontSize(args.vg, 11.f);
        nvgFontFaceId(args.vg, font->handle);
        nvgTextLetterSpacing(args.vg, 0.1f);
        nvgFillColor(args.vg, nvgRGB(210, 235, 210));
        nvgTextAlign(args.vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
        nvgText(args.vg, box.size.x * 0.5f, box.size.y * 0.5f, module->getSelectedTrailName().c_str(), nullptr);

        nvgRestore(args.vg);
    }
};

struct TrailDisplay : TransparentWidget
{
    COTrails *module = nullptr;

    Vec projectPoint(const TrailPoint &p, float w, float h)
    {
        const CachedTrail &trail = module->getActiveTrail();
        float x = COTrails::norm(p.lon, trail.minLon, trail.maxLon);
        float y = COTrails::norm(p.lat, trail.minLat, trail.maxLat);
        float z = COTrails::norm(p.ele, trail.minEle, trail.maxEle);

        x = (x - 0.5f) * 2.f;
        y = (y - 0.5f) * 2.f;

        float sx = (x - y) * 0.5f;
        float sy = (x + y) * 0.25f - z * 0.9f;

        return Vec(
            w * 0.5f + sx * w * 0.42f,
            h * 0.72f + sy * h * 0.55f);
    }

    void draw(const DrawArgs &args) override
    {
        if (!module)
            return;

        const CachedTrail &trail = module->getActiveTrail();
        if (trail.points.size() < 2)
            return;

        const float w = box.size.x;
        const float h = box.size.y;

        nvgSave(args.vg);

        nvgBeginPath(args.vg);
        nvgRect(args.vg, 0, 0, w, h);
        nvgFillColor(args.vg, nvgRGB(20, 30, 20));
        nvgFill(args.vg);

        nvgBeginPath(args.vg);

        for (size_t i = 0; i < trail.points.size(); i++)
        {
            Vec p = projectPoint(trail.points[i], w, h);

            if (i == 0)
                nvgMoveTo(args.vg, p.x, p.y);
            else
                nvgLineTo(args.vg, p.x, p.y);
        }

        nvgStrokeColor(args.vg, nvgRGB(0, 220, 80));
        nvgStrokeWidth(args.vg, 1.5f);
        nvgStroke(args.vg);

        int maxIndex = (int)trail.points.size() - 1;
        int index = clamp((int)std::round(module->playhead * maxIndex), 0, maxIndex);

        Vec dot = projectPoint(trail.points[index], w, h);

        nvgBeginPath(args.vg);
        nvgCircle(args.vg, dot.x, dot.y, 2.5f);
        nvgFillColor(args.vg, nvgRGB(220, 40, 40));
        nvgFill(args.vg);

        nvgRestore(args.vg);
    }
};

struct COTrailsWidget : ModuleWidget
{
    COTrailsWidget(COTrails *module)
    {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/COTrails_v3.svg")));

        TrailDisplay *display = new TrailDisplay;
        display->module = module;
        display->box.pos = mm2px(Vec(4, 18));
        display->box.size = mm2px(Vec(22.48, 18));
        addChild(display);

        TrailLabelDisplay *trailLabel = new TrailLabelDisplay;
        trailLabel->module = module;
        trailLabel->box.pos = mm2px(Vec(4, 38));
        trailLabel->box.size = mm2px(Vec(22.48, 6));
        addChild(trailLabel);

        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        addParam(createParamCentered<RoundBlackKnob>(
            mm2px(Vec(15.24, 11.5)),
            module,
            COTrails::TRAIL_PARAM));

        addParam(createParamCentered<RoundBlackKnob>(
            mm2px(Vec(15.24, 52)),
            module,
            COTrails::RATE_PARAM));

        addOutput(createOutputCentered<PJ301MPort>(
            mm2px(Vec(15.24, 66.65)),
            module,
            COTrails::LON_OUTPUT));

        addOutput(createOutputCentered<PJ301MPort>(
            mm2px(Vec(15.24, 81.65)),
            module,
            COTrails::LAT_OUTPUT));

        addOutput(createOutputCentered<PJ301MPort>(
            mm2px(Vec(15.24, 96.65)),
            module,
            COTrails::ELEVATION_OUTPUT));

        addOutput(createOutputCentered<PJ301MPort>(
            mm2px(Vec(15.24, 111.65)),
            module,
            COTrails::PEAK_GATE_OUTPUT));
    }
};
// struct COTrailsWidget : ModuleWidget
// {
//     COTrailsWidget(COTrails *module)
//     {
//         setModule(module);
//         setPanel(createPanel(asset::plugin(pluginInstance, "res/COTrails_v2.svg")));
//     }
// };
Model *modelCOTrails = createModel<COTrails, COTrailsWidget>("COTrails");
